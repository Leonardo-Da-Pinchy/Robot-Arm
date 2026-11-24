#!/usr/bin/python3
"""Telemetry is read-only, ordered, fresh and explicit about missing data."""
import unittest
import numpy as np
from pinchy_bringup.control_contract import JOINTS
from pinchy_motion.tcp_telemetry import TCPSpeedTelemetry


class Kinematics:
    def evaluate(self,q):
        # Last joint changes only orientation, not translational speed.
        jac=np.zeros((3,6));jac[0,0]=.2;jac[1,1]=.1
        return np.eye(4),jac,np.zeros((3,6))


class TCPRateTests(unittest.TestCase):
    def test_joint_order_and_controller_reference_are_independent(self):
        m=TCPSpeedTelemetry();q=np.zeros(6);dq=np.array([.1,.2,0,0,0,100.])
        m.add_feedback(JOINTS[::-1],q[::-1],dq[::-1],now=1.)
        m.add_reference(JOINTS,q,[.3,0,0,0,0,0],now=1.)
        report=m.snapshot(Kinematics(),now=1.1)
        self.assertAlmostEqual(report['current_tcp_speed_m_s'],np.hypot(.02,.02))
        self.assertAlmostEqual(report['commanded_tcp_speed_m_s'],.06)
        self.assertEqual(report['tcp_speed_source'],'joint_velocity')

    def test_position_only_feedback_fits_recent_encoder_window(self):
        m=TCPSpeedTelemetry()
        for i in range(21):
            q=np.zeros(6);q[0]=i*.01*.1
            m.add_feedback(JOINTS,q,stamp=10+i*.01,now=20+i*.01)
        self.assertAlmostEqual(m.snapshot(Kinematics(),now=20.2)['current_tcp_speed_m_s'],.02)
        self.assertEqual(m.snapshot(Kinematics(),now=20.2)['tcp_speed_source'],'encoder_position_window')
        self.assertIsNone(m.snapshot(Kinematics(),now=21.)['current_tcp_speed_m_s'])

    def test_unknown_stale_bad_or_restarted_data_never_becomes_fake_zero(self):
        m=TCPSpeedTelemetry();self.assertIsNone(m.snapshot(Kinematics(),now=1.)['current_tcp_speed_m_s'])
        for t in (1.,1.04,1.08):m.add_feedback(JOINTS,[0.]*6,stamp=t,now=t)
        self.assertEqual(m.snapshot(Kinematics(),now=1.1)['current_tcp_speed_m_s'],0.)
        m.add_feedback(JOINTS,[0.]*6,stamp=.5,now=1.2)
        self.assertIsNone(m.snapshot(Kinematics(),now=1.2)['current_tcp_speed_m_s'])
        m.add_reference(JOINTS,[0.]*6,[0.]*6,now=1.2)
        self.assertIsNone(m.snapshot(None,now=1.2)['commanded_tcp_speed_m_s'])
        self.assertIsNone(m.snapshot(Kinematics(),now=1.8)['commanded_tcp_speed_m_s'])
        m.add_feedback(JOINTS,[float('nan')]*6,now=2.)
        self.assertIsNone(m.snapshot(Kinematics(),now=2.)['current_tcp_speed_m_s'])
        m.add_reference(JOINTS,[0.]*6,[],now=2.)
        self.assertIsNone(m.snapshot(Kinematics(),now=2.)['commanded_tcp_speed_m_s'])


if __name__=='__main__':unittest.main()
