"""The only /cmd_vel publisher (to NALA's MCU). Runs locally on the robot computer."""
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty as EmptyMsg
from std_srvs.srv import Trigger, SetBool
from diagnostic_msgs.msg import DiagnosticArray
from .health import Ownership
from .params import node_settings
from .ros_common import Inputs, LATEST, parameter, decode, json_msg, diagnostic


class SafetyGate(Node):
    def __init__(self):
        super().__init__('velocity_safety_gate')
        self.mapping=parameter(self,'mapping_mode',False)
        self.external_leases=not self.mapping
        self.settings=node_settings(self);m=self.settings['motion']
        self.inputs=Inputs(self);self.policy=Ownership(m['linear_m_s'],m['angular_rad_s'],m['linear_accel_m_s2'],m['angular_accel_rad_s2'],self.settings)
        self.last_trust=0.;self.trusted=False;self.peer_hash=''
        self.last_lease=0.;self.last_manual_heartbeat=0.;self.graph_ok=False;self.graph_reason='GRAPH_NOT_CHECKED';self.graph_conflicts=[]
        self.graph_started=time.monotonic()
        self.pub=self.create_publisher(Twist,'/cmd_vel',LATEST)
        self.state_pub=self.create_publisher(String,'/safety/state',LATEST)
        self.diag=self.create_publisher(DiagnosticArray,'/safety/status',10)
        self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.command('NAV',m),LATEST)
        self.create_subscription(Twist,'/cmd_vel_remote',lambda m:self.command('MANUAL',m),LATEST)
        self.create_subscription(String,'/coverage/lease',self.lease,LATEST)
        self.create_subscription(String,'/coverage/trust',self.trust,LATEST)
        self.create_subscription(EmptyMsg,'/control/manual_heartbeat',self.manual_heartbeat,10)
        self.create_service(Trigger,'/safety/reset',self.reset)
        self.create_service(Trigger,'/safety/estop',self.stop_service)
        if self.mapping:self.create_service(SetBool,'/control/manual',self.manual)
        self.create_timer(1/self.settings['health']['gate_rate_hz'],self.tick);self.create_timer(self.settings['health']['graph_period_s'],self.check_graph)

    def command(self,source,msg):
        self.policy.receive(source,(msg.linear.x,msg.angular.z),time.monotonic())

    def lease(self,msg):
        if not self.external_leases:return
        try:
            d=decode(msg)
            self.peer_hash=d.get('config_hash','')
            self.policy.lease(d['owner'],str(d['epoch']),time.monotonic())
            self.last_lease=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def trust(self,msg):
        try:self.trusted=bool(decode(msg)['trusted']);self.last_trust=time.monotonic()
        except (ValueError,KeyError,TypeError):self.trusted=False

    def manual_heartbeat(self,_msg):
        self.last_manual_heartbeat=time.monotonic()
        if self.mapping and self.policy.owner=='MANUAL':self.policy.lease_time=self.last_manual_heartbeat

    def manual(self,req,res):
        health_reason=self.inputs.reason()
        if req.data and (health_reason or not self.graph_ok or self.policy.fault):
            blockers=[]
            for blocker in (health_reason, self.graph_reason if not self.graph_ok else '', self.policy.fault):
                if blocker and blocker not in blockers: blockers.append(blocker)
            res.success=False;res.message='Manual ownership denied: '+','.join(blockers);return res
        now=time.monotonic();self.policy.lease('MANUAL' if req.data else 'NONE',str(now),now)
        if req.data:self.last_manual_heartbeat=now
        res.success=True;res.message='Manual ownership changed; fresh commands required';return res

    def check_graph(self):
        publishers=self.get_publishers_info_by_topic('/cmd_vel')
        nodes=self.get_node_names_and_namespaces()
        own=[item for item in publishers if item.node_name==self.get_name() and item.node_namespace==self.get_namespace()]
        matching_nodes=[(name,namespace) for name,namespace in nodes if name==self.get_name()]
        duplicate=[f'{namespace}/{name}' for name,namespace in matching_nodes[1:]]
        extra=[f'{item.node_namespace}/{item.node_name}:{bytes(item.endpoint_gid).hex()}' for item in publishers if item not in own]
        names=[name for name,_ in nodes]
        localization_conflict=('amcl' in names if self.mapping else any('slam_toolbox' in name for name in names))
        self.graph_conflicts=extra+duplicate+(['LOCALIZATION_OWNER_CONFLICT'] if localization_conflict else [])
        self.graph_ok=len(publishers)==1 and len(own)==1 and not self.graph_conflicts
        self.graph_reason='' if self.graph_ok else 'CMD_VEL_OR_NODE_OWNERSHIP_CONFLICT'
        if not self.graph_ok and time.monotonic()-self.graph_started>=1.0:self.policy.latch(self.graph_reason)

    def stop_service(self,req,res):
        self.policy.latch('OPERATOR_OR_ACTION_STOP');res.success=True
        res.message='Stop latched; reset and resume are separate';return res

    def reset(self,req,res):
        cause=self.inputs.reason()
        if cause or not self.inputs.stopped() or not self.graph_ok:
            res.success=False;res.message=cause or 'Motion or graph conflict remains';return res
        if self.policy.owner!='NONE':
            res.success=False;res.message='Pause/cancel or relinquish manual ownership first';return res
        self.policy.fault=''
        res.success=True;res.message='Reset done; resume is separate'
        return res

    def tick(self):
        now=time.monotonic();reason=self.inputs.reason()
        if self.mapping and self.policy.owner=='MANUAL' and now-self.last_manual_heartbeat>self.settings['health']['lease_age_s']:
            reason=reason or 'MANUAL_HEARTBEAT_LOST'
        if not self.graph_ok:reason=reason or self.graph_reason
        if self.policy.owner=='NAV' and (not self.trusted or now-self.last_trust>self.settings['health']['gate_age_s']):
            reason=reason or 'LOCALIZATION_LOST'
        if self.external_leases and self.policy.owner!='NONE' and now-self.last_lease>self.settings['health']['lease_age_s']:
            reason=reason or 'SUPERVISOR_HEARTBEAT_LOST'
        if self.external_leases and self.policy.owner!='NONE' and self.peer_hash!=self.settings.hash:
            reason=reason or 'CONFIG_HASH_MISMATCH'
        command=self.policy.tick(now,not reason,self.inputs.stopped(),dt=1/self.settings['health']['gate_rate_hz'])
        if command is not None:
            # linear.y stays 0: NALA drives like a differential-drive robot for now (no sideways motion).
            msg=Twist();msg.linear.x,msg.angular.z=command;self.pub.publish(msg)
        if self.policy.stopping and now-self.policy.stop_at>self.settings['deadlines']['action_cancel_s'] and not self.inputs.stopped():
            self.policy.latch('STOP_NOT_CONFIRMED')
        state={'config_hash':self.settings.hash,'owner':self.policy.owner,'epoch':self.policy.epoch,'healthy':not reason and not self.policy.fault,
               'reason':reason,'fault':self.policy.fault,'stopped':self.inputs.stopped(),
               'graph_ok':self.graph_ok,'graph_conflicts':self.graph_conflicts,'events':self.policy.events}
        self.state_pub.publish(json_msg(state))
        diagnostic(self,self.diag,'velocity_safety_gate',self.policy.fault or reason or 'READY',state,2 if self.policy.fault else int(bool(reason)))


def _stop_before_shutdown(node):
    if node.policy.owner not in ('NAV','MANUAL'):
        return
    started=time.monotonic();sent=0
    while rclpy.ok() and (sent<node.settings['health']['shutdown_stop_messages'] or
                          (not node.inputs.stopped() and time.monotonic()-started<node.settings['health']['stop_confirm_timeout_s'])):
        node.pub.publish(Twist());sent+=1
        rclpy.spin_once(node,timeout_sec=1/node.settings['health']['gate_rate_hz'])
    if not node.inputs.stopped():node.policy.latch('SHUTDOWN_STOP_NOT_CONFIRMED')


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=SafetyGate()
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    stopping=False
    def interrupt(_signum,_frame):
        nonlocal stopping
        stopping=True
    for sig in previous:signal.signal(sig,interrupt)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        _stop_before_shutdown(node)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':main()
