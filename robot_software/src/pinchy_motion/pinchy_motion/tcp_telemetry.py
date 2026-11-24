"""Read-only TCP speed estimates; no control, planning or motor interfaces."""
from collections import deque
import threading
import time
import numpy as np
from pinchy_bringup.control_contract import JOINTS


class TCPSpeedTelemetry:
    window=.20
    stale=.50

    def __init__(self):
        self.lock=threading.Lock();self.history=deque(maxlen=128)
        self.feedback=None;self.reference=None

    @staticmethod
    def ordered(names,values):
        if len(names)!=len(values) or len(set(names))!=len(names):return None
        mapping=dict(zip(names,values))
        if not all(n in mapping and np.isfinite(mapping[n]) for n in JOINTS):return None
        return np.array([mapping[n] for n in JOINTS],dtype=float)

    def add_feedback(self,names,positions,velocities=(),stamp=None,now=None):
        now=time.monotonic() if now is None else now
        q=self.ordered(names,positions);dq=self.ordered(names,velocities)
        with self.lock:
            if q is None:
                self.feedback=None;self.history.clear();return
            sample=now if stamp is None or not np.isfinite(stamp) or stamp<=0 else stamp
            if self.history and (sample<=self.history[-1][0] or now-self.history[-1][2]>self.stale):self.history.clear()
            self.history.append((sample,q,now))
            while self.history and sample-self.history[0][0]>self.window:self.history.popleft()
            source='joint_velocity'
            if dq is None:
                source='encoder_position_window'
                if len(self.history)>=3 and sample-self.history[0][0]>=.05:
                    t=np.array([r[0]-sample for r in self.history]);t-=np.mean(t)
                    values=np.array([r[1] for r in self.history]);dq=t@values/(t@t)
            self.feedback=(now,q,dq,source)

    def add_reference(self,names,positions,velocities,now=None):
        q=self.ordered(names,positions);dq=self.ordered(names,velocities)
        with self.lock:self.reference=None if q is None or dq is None else (time.monotonic() if now is None else now,q,dq)

    def snapshot(self,solver,now=None):
        now=time.monotonic() if now is None else now
        with self.lock:feedback=self.feedback;reference=self.reference
        result=dict(current_tcp_speed_m_s=None,commanded_tcp_speed_m_s=None,tcp_speed_source=None,
                    tcp_speed_estimation_window_s=self.window)
        if solver is None:return result
        for row,key in ((feedback,'current_tcp_speed_m_s'),(reference,'commanded_tcp_speed_m_s')):
            if row is None or row[2] is None or not 0<=now-row[0]<=self.stale:continue
            try:
                _,jac,_=solver.evaluate(row[1]);speed=float(np.linalg.norm(jac@row[2]))
                if np.isfinite(speed):
                    result[key]=speed
                    if key=='current_tcp_speed_m_s':result['tcp_speed_source']=row[3]
            except (ValueError,ArithmeticError):pass
        return result
