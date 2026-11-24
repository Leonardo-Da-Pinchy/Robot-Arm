#!/usr/bin/python3
"""Actual panel/PlanPath progress on isolated mocks. No execution requests."""
import json,os,time,unittest
from pathlib import Path
from python_qt_binding.QtWidgets import QApplication
import rclpy
from pinchy_motion.panel import PosePanel
from pinchy_motion import point_sets
from test_cartesian import CartesianFixture,ROOT


class Runner(CartesianFixture,unittest.TestCase):
    def check_progress(self,kind):
        app=QApplication.instance() or QApplication([]);node=rclpy.create_node('live_planning_progress_panel',context=self.context)
        window=PosePanel(node);window.show();events=[];last=None;screenshot=False
        def pump():
            app.processEvents();self.executor.spin_once(timeout_sec=.01)
        try:
            deadline=time.monotonic()+15
            while not window.plan_button.isEnabled() and time.monotonic()<deadline:pump()
            self.assertTrue(window.plan_button.isEnabled(),window.notice.text())
            name='04_cylinder_continuous_2in.json' if kind=='cylinder' else '05_plane_continuous_2in.json'
            window.apply_point_set(point_sets.read(ROOT/'src/pinchy_motion/examples/point_sets'/name));window.plan_queued()
            self.assertTrue(window.busy);deadline=time.monotonic()+480;next_update=time.monotonic()+20
            while window.busy and time.monotonic()<deadline:
                pump();p=window.status_data.get('planning_progress',{})
                if p.get('active'):
                    key=(p['phase'],p.get('completed'),p.get('total'))
                    if key!=last:events.append(p);last=key
                    if p.get('total') and p.get('completed',0)>p['total']*.1 and not screenshot:
                        window.grab().save(str(self.folder/'planning-progress.png'));screenshot=True
                if time.monotonic()>next_update:print(kind,window.notice.text().replace('\n',' | '),flush=True);next_update+=20
            self.assertFalse(window.busy,'Panel planning timeout');self.assertTrue(window.plan_id,window.notice.text())
            self.assertTrue(screenshot,'No live counted progress reached the panel')
            known=[p for p in events if p.get('total')];self.assertTrue(any('collision' in p['phase'].lower() for p in known))
            self.assertTrue(any('speed' in p['phase'].lower() for p in known));self.assertTrue(any(p.get('configuration_attempt') for p in events))
            self.assertTrue(window.progress_bar.isHidden());self.assertIn('Ordered spline path ready',window.notice.text())
            (self.folder/'planning_progress.json').write_text(json.dumps(dict(surface=kind,success=True,planning_elapsed_s=time.monotonic()-window.planning_started,physical_motors_moved=False,execution_requested=False,progress=events,final_message=window.notice.text()),indent=2)+'\n')
            print('PASS',kind,'live progress samples:',len(events),'artifacts:',self.folder,flush=True)
        finally:
            window.timer.stop();window.target_timer.stop();window.queue_timer.stop();window.executor.shutdown();window.close();app.processEvents();node.destroy_node()


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--surface',choices=['plane','cylinder'],required=True);args=parser.parse_args()
    os.environ.update(PINCHY_TEST_CALIBRATED_MOCK='1',PINCHY_TEST_SCENE=str(ROOT/f'src/pinchy_moveit_config/config/nominal_{args.surface}_dry_run.yaml'))
    Runner.setUpClass()
    try:Runner('runTest').check_progress(args.surface)
    finally:Runner.tearDownClass()


if __name__=='__main__':main()
