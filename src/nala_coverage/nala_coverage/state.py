"""Deterministic mission bookkeeping: clocks and actions are supplied by adapters."""
from dataclasses import dataclass, field
import uuid


@dataclass
class Mission:
    generation: str = ''
    token: int = 0
    state: str = 'IDLE'
    started: float = 0.
    deadline: float = 1800.
    reason: str = ''
    root_cause: str = ''
    blocked: list = field(default_factory=list)
    failures: dict = field(default_factory=dict)
    events: list = field(default_factory=list)
    coverage_finished: bool = False

    def start(self, now, timeout=1800.):
        if self.state not in ('IDLE','CANCELED','FINISHED','FAILED'):
            raise ValueError('Cancel or finish the current task before starting another')
        self.generation=uuid.uuid4().hex;self.token+=1;self.state='PREPARING'
        self.started=now;self.deadline=timeout;self.reason='';self.root_cause=''
        self.blocked.clear();self.failures.clear();self.events.clear()
        self.coverage_finished=False

    def action_id(self):
        self.token+=1
        return (self.generation,self.token)

    def current(self, identity):
        return identity == (self.generation,self.token)

    def change(self, state, reason='', now=0.):
        self.token+=1;self.state=state;self.reason=reason
        if reason:
            if not self.root_cause:self.root_cause=reason
            self.events.append({'time':now,'state':state,'reason':reason})

    def pause(self, reason, now):
        self.change('PAUSED',reason,now)

    def resume(self):
        if self.state!='PAUSED':raise ValueError('Task is not paused')
        self.change('PREPARING')  # Preserve generation, start time, exclusions and retries.

    def expired(self, now):
        return bool(self.generation and not self.coverage_finished and
                    self.state not in ('IDLE','CANCELED','FINISHED','FAILED') and
                    now-self.started>=self.deadline)
