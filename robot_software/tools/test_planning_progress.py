#!/usr/bin/python3
"""Planning telemetry and UI lifetime/priority; isolated ROS, no motor I/O."""
import json,time,unittest
from types import SimpleNamespace
from pinchy_motion.planning_progress import PlanningProgress,progress_text


class ProgressTests(unittest.TestCase):
    def test_stage_eta_resets_on_retry_and_never_becomes_whole_job_eta(self):
        now=[100.];p=PlanningProgress(clock=lambda:now[0]);p.begin('request','ordered path')
        p.update('Collision checks',0,100,'checks');now[0]+=10;p.update('Collision checks',25,100,'checks')
        state=p.snapshot();self.assertEqual(state['stage_remaining_s'],30.)
        self.assertIn('25/100 checks (25% of this stage)',progress_text(state))
        self.assertIn('left in this stage',progress_text(state));self.assertNotIn('percent',state)
        p.update('Collision checks',0,100,'checks');self.assertIsNone(p.snapshot()['stage_remaining_s'])
        p.update('Search',configuration_attempt=2,configuration_limit=8)
        self.assertIsNone(p.snapshot()['total']);self.assertNotIn('%',progress_text(p.snapshot()))
        p.end();self.assertEqual(p.snapshot(),dict(active=False))
        p.begin('next','pose');self.assertNotIn('configuration_attempt',p.snapshot())

    def test_partial_collision_batch_counts_all_samples_with_active_tracker(self):
        from test_trajectory_validation import ValidationTests
        server,seen,_=ValidationTests().server();server.planning_progress=PlanningProgress()
        server.planning_progress.begin('case','path');server.planning_progress.update('Collision check')
        server.validate_states(([i/100.]*6 for i in range(79)))
        state=server.planning_progress.snapshot()
        self.assertEqual((state['completed'],state['total']),(79,79));self.assertEqual(len(seen),79)


class PanelProgressTests(unittest.TestCase):
    def test_counts_stale_status_stop_and_result_do_not_overwrite_each_other(self):
        import rclpy
        from rclpy.context import Context
        from python_qt_binding.QtWidgets import QApplication
        from pinchy_motion.panel import PosePanel
        app=QApplication.instance() or QApplication([]);context=Context();rclpy.init(context=context,domain_id=221)
        node=rclpy.create_node('planning_progress_panel_test',context=context);window=PosePanel(node)
        try:
            window.timer.stop();window.status_data=dict(hardware=False,safety='READY',planning_progress=dict(active=True,request_id='old',phase='OLD STAGE',elapsed_s=200))
            window.last_status=time.monotonic();window.busy=True;window.begin_planning_display()
            self.assertNotIn('OLD STAGE',window.notice.text());self.assertEqual(window.progress_bar.maximum(),0)
            state=dict(active=True,request_id='new',phase='Checking stroke collisions',elapsed_s=65,completed=120,total=400,unit='collision checks',stage_remaining_s=14,configuration_attempt=1,configuration_limit=8)
            window.status_data['planning_progress']=state;window.update_planning_display()
            self.assertIn('120/400',window.notice.text());self.assertIn('1m 05s elapsed',window.notice.text())
            self.assertEqual((window.progress_bar.value(),window.progress_bar.maximum()),(120,400));self.assertTrue(window.stop_button.isEnabled())
            window.last_status=time.monotonic()-2;window.update_planning_display()
            self.assertIn('stale',window.notice.text());self.assertEqual(window.progress_bar.maximum(),0)
            window.last_status=time.monotonic();window.planning_interrupted=True;window.notice.setText('Requesting powered hold…');window.update_planning_display()
            self.assertEqual(window.notice.text(),'Requesting powered hold…');self.assertTrue(window.progress_bar.isHidden())
            window.planned(SimpleNamespace(result=lambda:SimpleNamespace(success=False,plan_id='',message='Stopped during planning')))
            window.update_planning_display();self.assertEqual(window.notice.text(),'Stopped during planning');self.assertTrue(window.progress_bar.isHidden())
        finally:
            window.target_timer.stop();window.queue_timer.stop();window.executor.shutdown();window.close();app.processEvents();node.destroy_node();context.shutdown()


if __name__=='__main__':unittest.main()
