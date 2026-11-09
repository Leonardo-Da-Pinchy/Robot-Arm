#!/usr/bin/env python3
"""Live physical startup dashboard: sequential zero holds, then ROS/UI.

No autonomous recovery from a folded pose. Support the arm; bring each selected
joint near its CAD zero for automatic capture. Previously locked joints remain
powered. Faults retain established holds; explicit abort/exit attempts torque
release, so keep support available.
The original calibration file is never changed. No EEPROM writes are made.
"""
import argparse
import copy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

from calibrate import JOINTS, ROOT, CONFIG_KEYS, TICK, load_profile
from servo_chain import health, preflight_settings
from servo_setup import SetupBus
from servo_bench import error_text
from servo_diagnostics import temperature_spike_unconfirmed

ZERO_TOLERANCE = 10 # Encoder ticks: approximately 0.88 degrees


def validate_profile(profile):
    if profile.get('capture_backend') != 'hardware':
        raise ValueError('Guided physical startup requires hardware calibration captures')
    model = ET.parse(ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf').getroot()
    for servo_id, name in JOINTS.items():
        j = profile['joints'][name]
        if any(j.get(k) is None for k in ['zero_tick', 'direction', 'min_tick', 'max_tick',
                                         'min_radians', 'max_radians', 'max_velocity']):
            raise ValueError(f'ID {servo_id}: finish zero, direction and limits capture first')
        if not all(type(j[k]) is int for k in ['servo_id', 'zero_tick', 'direction', 'min_tick', 'max_tick']):
            raise ValueError(f'ID {servo_id}: invalid integer calibration fields')
        if not (j['servo_id'] == servo_id and j['direction'] in (-1, 1) and
                0 <= j['min_tick'] <= j['zero_tick'] <= j['max_tick'] <= 4095 and
                j['min_tick'] < j['max_tick']):
            raise ValueError(f'ID {servo_id}: invalid ID, sign or encoder range')
        limit = model.find(f"joint[@name='{name}']/limit")
        low, high = j['min_radians'], j['max_radians']
        if not (all(math.isfinite(v) for v in [low, high, j['max_velocity']]) and
                float(limit.get('lower')) <= low <= 0 <= high <= float(limit.get('upper')) and
                low < high and 0 < j['max_velocity'] <= float(limit.get('velocity'))):
            raise ValueError(f'ID {servo_id}: invalid calibrated angle/velocity bounds')
        for angle in [low, high]:
            ticks = round(j['zero_tick']+j['direction']*angle/TICK)
            if not j['min_tick'] <= ticks <= j['max_tick']:
                raise ValueError(f'ID {servo_id}: angle and encoder bounds disagree')
        observation = profile.get('observations', {}).get(name, {})
        if 'zero' not in observation or 'positive_direction' not in observation:
            raise ValueError(f'ID {servo_id}: missing physical zero/direction evidence')
        delta = observation['positive_direction'].get('encoder_delta', 0)
        if not 8 <= abs(delta) <= 1024 or (1 if delta > 0 else -1) != j['direction']:
            raise ValueError(f'ID {servo_id}: direction evidence disagrees with calibration')
    if not (1 <= profile['speed'] <= 600 and 1 <= profile['acceleration'] <= 40 and
            1 <= profile['torque_limit'] <= 1000):
        raise ValueError('Require raw speed <=600, acceleration <=40 and torque limit in 1..1000; startup uses caps 100/5')
    return model


class GuidedStartup:
    def __init__(self, bus, profile, max_start_degrees=25, log=lambda *a, **k: None,
                 clock=time.monotonic, sleep=time.sleep):
        self.bus, self.profile = bus, profile
        self.model = validate_profile(profile)
        if not 0 < max_start_degrees <= 25:
            raise ValueError('Near-zero startup window must be in (0, 25] degrees')
        self.max_start = math.radians(max_start_degrees)
        self.log, self.clock, self.sleep = log, clock, sleep
        self.held = []; self.touched = set()
        self.hot_counts = dict.fromkeys(JOINTS, 0)
        self.observer = lambda servo_id, state: None

    def call(self, servo_id, method, *args):
        self.bus.servo_id = servo_id
        try:
            return getattr(self.bus, method)(*args)
        except RuntimeError as exc:
            raise RuntimeError(f'ID {servo_id} ({JOINTS[servo_id]}), {method}: {exc}') from exc

    def state(self, servo_id, full=False):
        state = self.call(servo_id, 'setup_state' if full else 'read')
        if not full: state['status_raw'] = self.call(servo_id, 'byte', 65)
        if state['temperature_raw'] >= 60:
            state['temperature_recheck_raw'] = self.call(servo_id, 'byte', 63)
            state['temperature_block_raw'] = self.call(servo_id, 'word', 62) >> 8
            state['status_recheck_raw'] = self.call(servo_id, 'byte', 65)
            state['temperature_spike_unconfirmed'] = temperature_spike_unconfirmed(state)
            self.hot_counts[servo_id] = 0 if state['temperature_spike_unconfirmed'] else self.hot_counts[servo_id]+1
            self.log('temperature_recheck', id=servo_id, **state)
        else: self.hot_counts[servo_id] = 0
        check = dict(state)
        if 0 < self.hot_counts[servo_id] < 4:
            # Defer only the temperature cutoff; retain voltage/mode/alarm checks.
            # Return and display the original temperature, never this check copy.
            check['temperature_raw'] = 59
        health(check, servo_id)
        if not 0 <= state['position'] <= 4095: raise ValueError('Encoder wrap is unsupported')
        if full:
            name = JOINTS[servo_id]
            expected = self.profile['observations'][name]['zero']['configuration']
            if any(state[key] != expected[key] for key in CONFIG_KEYS):
                raise ValueError(f'ID {servo_id}: servo configuration changed since calibration')
        self.observer(servo_id, state)
        return state

    def angle(self, servo_id, ticks):
        j = self.profile['joints'][JOINTS[servo_id]]
        return j['direction']*(ticks-j['zero_tick'])*TICK

    def preflight(self):
        for servo_id in JOINTS:
            state = self.state(servo_id, full=True)
            preflight_settings(state, servo_id)
            self.log('preflight', id=servo_id, ticks=state['position'])

    def monitor(self):
        for servo_id in self.held:
            state = self.state(servo_id)
            zero = self.profile['joints'][JOINTS[servo_id]]['zero_tick']
            if state['torque'] == 1 and ZERO_TOLERANCE < abs(state['position']-zero) <= 32:
                # Confirm small excursions rather than dropping every hold on
                # one noisy reading. No new motor goals are sent during rechecks.
                self.log('hold_recheck', id=servo_id, target=zero, position=state['position'])
                for _ in range(2):
                    self.sleep(0.05)
                    state = self.state(servo_id)
                    if state['torque'] != 1 or abs(state['position']-zero) > 32: break
                    if abs(state['position']-zero) <= ZERO_TOLERANCE: break
            if state['torque'] != 1 or abs(state['position']-zero) > ZERO_TOLERANCE:
                raise RuntimeError(f'ID {servo_id}: lost zero hold; '
                                   f"target={zero}, position={state['position']}, "
                                   f"error={state['position']-zero:+d} ticks (allowed +/-{ZERO_TOLERANCE}), "
                                   f"torque={state['torque']}; support arm and inspect load/feedback")

    def describe(self, servo_id):
        state = self.state(servo_id)
        print(f"ID {servo_id} {JOINTS[servo_id]}: {math.degrees(self.angle(servo_id, state['position'])):+.2f} degrees; "
              f"zero tick={self.profile['joints'][JOINTS[servo_id]]['zero_tick']}. "
              f"Holding IDs: {self.held}", flush=True)

    def lock_motor(self, servo_id):
        self.monitor()
        state = self.state(servo_id, full=True)
        preflight_settings(state, servo_id)
        name = JOINTS[servo_id]; j = self.profile['joints'][name]
        start = state['position']; angle = self.angle(servo_id, start)
        limit = self.model.find(f"joint[@name='{name}']/limit")
        if abs(angle) > self.max_start or not float(limit.get('lower')) <= angle <= float(limit.get('upper')):
            raise ValueError(f'ID {servo_id}: {math.degrees(angle):+.2f} degrees is not near zero. '
                             'Manually support/position this joint near zero, then try again; no motor write sent.')
        # This explicit operator-approved startup path only moves inward to zero
        # within the small CAD-bounded corridor, even if the free joint drifted
        # slightly beyond an operational bound at zero. ROS retains normal limits.
        self.touched.add(servo_id)
        self.log('locking', id=servo_id, from_ticks=start, to_ticks=j['zero_tick'])
        self.call(servo_id, 'set_torque_limit', self.profile['torque_limit'])
        self.call(servo_id, 'move', start, min(100, self.profile['speed']), min(5, self.profile['acceleration']))
        if self.call(servo_id, 'word', 42) != start: raise RuntimeError('Current-pose staging readback failed')
        self.call(servo_id, 'enable')
        current = self.state(servo_id)
        if current['torque'] != 1 or abs(current['position']-start) > 5:
            raise RuntimeError('Unexpected movement during current-pose hold')
        duration = max(0.5, abs(angle)/min(0.075, j['max_velocity']))
        began = self.clock(); last = start
        while True:
            fraction = min(1.0, (self.clock()-began)/duration)
            target = round(start+(j['zero_tick']-start)*fraction)
            if target != last:
                self.call(servo_id, 'move', target, min(100, self.profile['speed']), min(5, self.profile['acceleration']))
                last = target
            self.monitor(); current = self.state(servo_id)
            if current['torque'] != 1:
                raise RuntimeError(f'ID {servo_id} ({name}): motor torque switched off while moving to zero')
            if abs(current['position']-target) > 16:
                # Freeze the commanded pose and trajectory clock while the servo
                # catches up. Never accumulate more motion against a stalled joint.
                paused = self.clock(); deadline = paused+3
                print(f'ID {servo_id} ({name}): pausing motion to let motor catch up '
                      f"({abs(current['position']-target)*TICK*180/math.pi:.2f} degrees behind target).", flush=True)
                self.log('catch_up', id=servo_id, target=target, position=current['position'])
                while abs(current['position']-target) > 8:
                    if current['torque'] != 1:
                        raise RuntimeError(f'ID {servo_id} ({name}): torque switched off during catch-up')
                    if abs(current['position']-target) > 64 or self.clock() >= deadline:
                        raise RuntimeError(f'ID {servo_id} ({name}): motor could not catch up to the paused target; '
                                           f"target={target}, position={current['position']}, "
                                           f"error={current['position']-target:+d} ticks. "
                                           'Support this joint and check for contact or insufficient holding force before retrying.')
                    self.sleep(0.05); self.monitor(); current = self.state(servo_id)
                if current['torque'] != 1:
                    raise RuntimeError(f'ID {servo_id} ({name}): torque switched off during catch-up')
                began += self.clock()-paused
                print(f'ID {servo_id} ({name}): caught up; continuing to zero.', flush=True)
            if fraction >= 1: break
            self.sleep(0.05)
        deadline = self.clock()+3
        while True:
            self.monitor(); current = self.state(servo_id)
            if current['torque'] == 1 and abs(current['position']-j['zero_tick']) <= ZERO_TOLERANCE: break
            if self.clock() >= deadline:
                raise TimeoutError(f'ID {servo_id}: zero not reached after 3 s settling; '
                                   f"target={j['zero_tick']}, position={current['position']}, "
                                   f"error={current['position']-j['zero_tick']:+d} ticks (allowed +/-{ZERO_TOLERANCE}), "
                                   f"torque={current['torque']}")
            self.sleep(0.05)
        self.held.append(servo_id); self.monitor()
        self.log('locked', id=servo_id, measured_ticks=current['position'])
        print(f'ID {servo_id} locked at calibrated zero (feedback within {ZERO_TOLERANCE} ticks). Holding IDs: {self.held}', flush=True)

    def release(self):
        errors = []
        for servo_id in sorted(self.touched):
            try:
                self.call(servo_id, 'release')
                if self.call(servo_id, 'read')['torque'] != 0: raise RuntimeError('Torque remains enabled')
            except Exception as exc: errors.append(f'ID {servo_id}: {exc}')
        self.held.clear()
        if errors: raise RuntimeError('Release unconfirmed: '+'; '.join(errors))


class TerminalPrompt:
    def __init__(self): self.buffer = ''

    def __call__(self, message, monitor):
        print(message, flush=True)
        while True:
            monitor()
            if '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1); return line.strip().lower()
            if select.select([sys.stdin], [], [], 0.1)[0]:
                data = os.read(sys.stdin.fileno(), 4096)
                if not data: raise EOFError('Terminal input closed')
                self.buffer += data.decode(errors='replace')


class StartupCancelled(Exception):
    pass


def walkthrough(startup, ask):
    startup.preflight()
    print('\nSupport the arm and keep motor power-cut reachable. Abort/fault/exit releases the motors you locked.\n'
          'For each ID, bring ONLY that joint close to its CAD zero, clear of contact.\n'
          'Enter authorizes a slow inward move to zero, then powered hold.\n'
          'Other unlocked joints remain free. Previously locked joints stay powered.\n'
          'This startup corridor does not widen your normal ROS movement limits.', flush=True)
    for servo_id in JOINTS:
        while True:
            startup.describe(servo_id)
            answer = ask(f'ID {servo_id}: Enter = ZERO + HOLD, p = refresh angle, q = abort/release:', startup.monitor)
            if answer == 'q': return False
            if answer == 'p': continue
            if answer not in ('', 'lock', f'lock {servo_id}'):
                print('Use Enter, p or q.', flush=True); continue
            # Check proximity before the write-owning method, so a positioning
            # rejection can be retried without dropping the earlier zero holds.
            state = startup.state(servo_id, full=True)
            angle = startup.angle(servo_id, state['position'])
            limit = startup.model.find(f"joint[@name='{JOINTS[servo_id]}']/limit")
            if abs(angle) > startup.max_start or not float(limit.get('lower')) <= angle <= float(limit.get('upper')):
                print(f'Not locked: {math.degrees(angle):+.2f} degrees. Position this joint within '
                      f'{math.degrees(startup.max_start):.1f} degrees of zero and inside its CAD range, then retry.', flush=True)
                continue
            startup.lock_motor(servo_id); break
    return True


def stop_process(process):
    if process.poll() is not None: return
    os.killpg(process.pid, signal.SIGINT)
    try: process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)


def ros_command(args, *command):
    ros_setup = Path('/opt/ros')/args.ros_distro/'setup.bash'
    workspace_setup = ROOT/'install/setup.bash'
    if not ros_setup.is_file() or not workspace_setup.is_file():
        raise RuntimeError('ROS/workspace setup missing; build the workspace before startup')
    return ['/bin/bash', '-c', 'source "$1"\nsource "$2"\nshift 2\nexec "$@"',
            'pinchy-guided-startup', str(ros_setup), str(workspace_setup), *map(str, command)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', help='Required: enable physical startup')
    parser.add_argument('--plain', action='store_true', help='Use the original Enter-per-joint prompts instead of the live dashboard')
    parser.add_argument('--calibration', type=Path, default=ROOT/'calibration/arm.json')
    parser.add_argument('--domain', type=int, default=42)
    parser.add_argument('--ros-distro', default=os.environ.get('ROS_DISTRO', 'jazzy'))
    parser.add_argument('--no-gui', action='store_true', help='Start ROS after zero holds, without RViz/sliders')
    parser.add_argument('--wizard-only', action='store_true', help='Keep prompted holds without starting ROS; q releases')
    parser.add_argument('--max-start-degrees', type=float, default=25,
                        help='Maximum starting distance from zero in degrees (default: 25; CAD limits still apply)')
    parser.add_argument('--log', type=Path, default=Path('/tmp/pinchy-arm-startup.jsonl'))
    args = parser.parse_args()
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    if not args.hardware: parser.error('Add --hardware only when ready for the prompted physical zero-and-hold steps')
    if not 0 <= args.domain <= 232: parser.error('ROS domain must be in 0..232')
    if not math.isfinite(args.max_start_degrees) or not 0 < args.max_start_degrees <= 25:
        parser.error('--max-start-degrees must be in (0, 25]')
    raw = json.loads(args.calibration.read_text())
    profile = load_profile(args.calibration, True, raw['port'])
    validate_profile(profile)
    if not args.wizard_only:
        # Refuse a second robot graph before opening the serial port or enabling torque.
        env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
        check = subprocess.run(ros_command(args, 'ros2', 'run', 'pinchy_bringup',
            'startup_ready.py', '--assert-idle'), env=env, timeout=10)
        if check.returncode != 0:
            raise RuntimeError('ROS domain is not available for guided startup; no motors enabled')
    args.log.parent.mkdir(parents=True, exist_ok=True)
    log = args.log.open('a'); token = uuid.uuid4().hex
    def record(event, **values):
        log.write(json.dumps(dict(time=datetime.now(timezone.utc).isoformat(), session=token,
                                  event=event, **values))+'\n'); log.flush()
    bus = SetupBus(profile['port'], 1_000_000, None)
    def diagnostics(bus):
        bus.sdk.read_retries = 2
        def packet_trace(packet):
            if packet['communication_result'] != 0 or packet['servo_error']:
                record('serial_error', **packet)
        bus.sdk.trace = packet_trace
    diagnostics(bus)
    startup = GuidedStartup(bus, profile, args.max_start_degrees, record)
    prompt = TerminalPrompt(); processes = []; code = 1
    try:
        print('PHYSICAL ARM STARTUP — using saved zeros, signs and bounds. No recalibration or EEPROM writes.', flush=True)
        if not args.plain and sys.stdin.isatty() and sys.stdout.isatty():
            from startup_dashboard import run_dashboard
            run_dashboard(startup)
        elif not walkthrough(startup, prompt):
            raise StartupCancelled()
        print('\nAll six motors are at zero and holding.', flush=True)
        if args.wizard_only:
            while prompt('Holding all zeros. p = status, q = release and quit:', startup.monitor) != 'q':
                for servo_id in JOINTS: startup.describe(servo_id)
            raise StartupCancelled()
        startup.monitor()
        with tempfile.TemporaryDirectory(prefix='pinchy-zero-handoff-') as folder:
            session = copy.deepcopy(profile)
            session.update(profile_kind='zero_hold_handoff', calibration_confirmed=True, handoff_token=token)
            path = Path(folder)/'session.json'; path.write_text(json.dumps(session, indent=2)+'\n')
            env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
            print('Starting ROS while keeping all six zero holds powered...', flush=True)
            bus.close(); startup.bus = None
            process = subprocess.Popen(ros_command(args, 'ros2', 'launch', 'pinchy_bringup',
                'hardware_control.launch.py', f'calibration:={path}', 'motion_enabled:=true',
                'adopt_zero_hold:=true'), env=env, start_new_session=True)
            processes.append(process)
            ready = subprocess.Popen(ros_command(args, 'ros2', 'run', 'pinchy_bringup',
                'startup_ready.py', '--token', token, '--timeout', '30'), env=env, start_new_session=True)
            processes.append(ready)
            deadline = time.monotonic()+35
            while ready.poll() is None:
                if process.poll() is not None: raise RuntimeError('ROS exited during zero-hold handoff')
                if time.monotonic() >= deadline: raise RuntimeError('ROS handoff did not complete')
                time.sleep(0.1)
            result = ready.returncode
            if result not in (0, 2) or process.poll() is not None: raise RuntimeError('ROS did not adopt the zero holds')
            record('ros_ready' if result == 0 else 'ros_safety_hold')
            if not args.no_gui:
                for command in [('ros2', 'run', 'rviz2', 'rviz2', '-d', ROOT/'src/pinchy_bringup/config/arm.rviz'),
                                ('ros2', 'run', 'pinchy_bringup', 'joint_sliders.py', '--hardware')]:
                    processes.append(subprocess.Popen(ros_command(args, *command), env=env, start_new_session=True))
            if result == 0:
                print('READY: ROS owns the arm. Use sliders. Keep this terminal open; Ctrl+C stops ROS and releases torque.', flush=True)
            else:
                print('SAFETY HOLD: movement is blocked; torque remains on. Inspect/support the arm. Ctrl+C explicitly releases torque.', flush=True)
            while process.poll() is None: time.sleep(0.2)
            if process.returncode != 0: raise RuntimeError(f'ROS launch exited {process.returncode}')
            code = 0
    except (KeyboardInterrupt, EOFError, StartupCancelled):
        print('\nStopping. Support the arm: releasing guided startup holds.', flush=True); code = 0
    except Exception as exc:
        record('failed', error=str(exc)); print(error_text(exc), file=sys.stderr, flush=True)
        if startup.bus is not None and startup.touched:
            print('SAFETY HOLD: no further movement commands; existing torque holds are preserved. '
                  'Support/inspect the arm. Press q or Ctrl+C only when ready to release torque.', flush=True)
            try:
                while True:
                    if select.select([sys.stdin], [], [], 0.2)[0]:
                        value = os.read(sys.stdin.fileno(), 4096)
                        if b'q' in value.lower(): break
                        if not value: time.sleep(0.2)
            except KeyboardInterrupt:
                pass
    finally:
        # Let bounded cleanup finish if a terminal sends repeated stop signals.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        for process in reversed(processes):
            try: stop_process(process)
            except Exception as exc: print(f'Could not stop child process: {exc}', file=sys.stderr)
        if startup.bus is None:
            try:
                startup.bus = SetupBus(profile['port'], 1_000_000, None)
                diagnostics(startup.bus)
            except Exception as exc: print(f'Could not reopen bus to confirm release: {exc}. Use motor-power cut.', file=sys.stderr)
        if startup.bus is not None:
            try: startup.release(); record('released', ids=sorted(startup.touched))
            except Exception as exc: print(error_text(exc)+'. Use motor-power cut.', file=sys.stderr); code = 1
            finally: startup.bus.close()
        log.close()
    return code


if __name__ == '__main__':
    try: raise SystemExit(main())
    except Exception as exc: raise SystemExit(error_text(exc))
