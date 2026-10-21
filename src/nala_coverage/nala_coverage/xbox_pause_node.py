"""App 3 (Pi): the Xbox controller's B button pauses the coverage task (service /coverage/pause).

ros2 run nala_coverage xbox_pause --ros-args --params-file config/teleop.yaml
Started by app3_coverage_pi.launch.py together with joy_node. It never publishes /cmd_vel: the velocity gate
stays the only /cmd_vel publisher in app 3. The supervisor pauses like the panel's Pause: Nav2 is cancelled,
the gate sends zero, the rest of the plan is kept. Resume on the robot screen (or /coverage/resume).
The node never waits in a callback: the request is sent with call_async and the reply is only logged.
"""

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from .buttons import rising
from .ros_common import run_until_stopped


class XboxPause(Node):

    def __init__(self):
        super().__init__('xbox_pause')
        # No default: config/teleop.yaml (xbox_pause) must set it.
        self.button = self.declare_parameter('pause_button', Parameter.Type.INTEGER).value
        self.client = self.create_client(Trigger, '/coverage/pause')
        self.previous = []      # buttons of the last /joy message
        self.pending = None     # the open pause request (at most one)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.get_logger().info(f'Button {self.button} (B) pauses the coverage task.')

    def on_joy(self, msg):
        buttons = list(msg.buttons)
        press = rising(self.previous, buttons, self.button)
        self.previous = buttons
        if not press:
            return
        if self.pending is not None and not self.pending.done():
            self.get_logger().info('B pressed: the last pause request has no reply yet, not sent again.')
            return
        if not self.client.service_is_ready():
            self.get_logger().warn('B pressed, but /coverage/pause is not available (supervisor not running): '
                                   'pause not sent.')
            return
        self.get_logger().info('B pressed: pausing the coverage task.')
        self.pending = self.client.call_async(Trigger.Request())
        self.pending.add_done_callback(self.on_reply)

    def on_reply(self, future):
        try:
            response = future.result()
        except Exception as error:   # the call itself failed (for example at shutdown): log it, keep running
            self.get_logger().warn(f'Pause request failed: {error}')
            return
        if response is None:
            self.get_logger().warn('Pause request was cancelled.')
        elif response.success:
            self.get_logger().info(f'Supervisor: {response.message}')
        else:
            self.get_logger().info(f'Supervisor refused the pause: {response.message}')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = XboxPause()
    try:
        run_until_stopped(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
