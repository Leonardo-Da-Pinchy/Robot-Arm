#!/usr/bin/python3
"""Live speed display inputs on a fresh isolated mock/generated PTY, never USB."""
import json,os,time,unittest
from test_cartesian import CartesianFixture


class TCPTelemetrySystem(CartesianFixture,unittest.TestCase):
    hardware=os.environ.get('PINCHY_TCP_TEST_EMULATED')=='1'

    def test_live_feedback_reference_and_final_hold(self):
        plan=self.plan([.1,-.25,.5,-.25,.4,.4],scale=.1)
        handle=self.execute(plan,wait=False);future=handle.get_result_async();samples=[]
        deadline=time.monotonic()+plan.duration+30
        while not future.done() and time.monotonic()<deadline:
            self.pump(.04)
            samples.append({k:self.status.get(k) for k in ('current_tcp_speed_m_s','commanded_tcp_speed_m_s','tcp_speed_source')})
        self.assertTrue(future.done());result=future.result();self.assertTrue(result.result.success,result.result.message)
        self.assertTrue(any((r['current_tcp_speed_m_s'] or 0)>.001 for r in samples),'No moving encoder/model speed')
        self.assertTrue(any((r['commanded_tcp_speed_m_s'] or 0)>.001 for r in samples),'No moving controller-reference speed')
        self.pump(1.)
        for key in ('current_tcp_speed_m_s','commanded_tcp_speed_m_s'):
            self.assertIsNotNone(self.status.get(key));self.assertLess(self.status[key],.0001)
        (self.folder/'tcp_speed_report.json').write_text(json.dumps(dict(success=True,hardware='PTY emulator' if self.hardware else 'mock',physical_motors_moved=False,
            peak_current_m_s=max(r['current_tcp_speed_m_s'] or 0 for r in samples),peak_reference_m_s=max(r['commanded_tcp_speed_m_s'] or 0 for r in samples),
            source=self.status.get('tcp_speed_source'),final={k:self.status.get(k) for k in ('current_tcp_speed_m_s','commanded_tcp_speed_m_s')},samples=samples),indent=2)+'\n')


def load_tests(loader,tests,pattern):return unittest.TestSuite([TCPTelemetrySystem('test_live_feedback_reference_and_final_hold')])
if __name__=='__main__':unittest.main()
