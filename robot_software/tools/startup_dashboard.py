"""Live terminal display and automatic sequential zero capture; no bus ownership."""
from contextlib import redirect_stdout
import curses
import math

from calibrate import JOINTS

CAPTURE_DWELL_SECONDS = 1.0


class CaptureWindow:
    """Require a stationary joint, not merely a pass through the capture window."""
    def __init__(self, dwell=CAPTURE_DWELL_SECONDS):
        self.dwell = dwell
        self.reset()

    def reset(self):
        self.since = None
        self.anchor = None

    def update(self, now, ticks, eligible):
        if not eligible:
            self.reset()
            return 0.0
        if self.since is None or abs(ticks-self.anchor) > 3:
            self.since, self.anchor = now, ticks
        return min(1.0, max(0.0, (now-self.since)/self.dwell))


class Dashboard:
    def __init__(self, screen, startup):
        self.screen, self.startup = screen, startup
        self.states = {}
        self.active = 1
        self.armed = False
        self.locking = False
        self.progress = 0.0
        self.message = 'Support the arm, clear joint travel, and keep the power disconnect reachable.'
        self.last_draw = -math.inf
        self.screen.nodelay(True)
        try: curses.curs_set(0)
        except curses.error: pass
        self.colors = {}
        if curses.has_colors():
            curses.start_color()
            for index, (label, foreground) in enumerate((('locked', curses.COLOR_GREEN),
                    ('locking', curses.COLOR_YELLOW), ('active', curses.COLOR_CYAN)), 1):
                curses.init_pair(index, foreground, curses.COLOR_BLACK)
                self.colors[label] = curses.color_pair(index) | curses.A_BOLD

    def write(self, value):
        # Existing movement notices become a fixed dashboard status line.
        if value.strip(): self.message = value.strip()
        return len(value)

    def flush(self): pass

    def key(self):
        key = self.screen.getch()
        if key in (ord('q'), ord('Q'), 27): raise KeyboardInterrupt()
        if key in (ord('s'), ord('S')) and not self.locking:
            self.armed = True
        if key in (ord('p'), ord('P')) and not self.locking:
            self.armed = False
            self.progress = 0.0

    def observe(self, servo_id, state):
        self.states[servo_id] = dict(state)
        self.key()
        self.draw()

    def line(self, row, text, style=0):
        height, width = self.screen.getmaxyx()
        if row < height-1 and width > 1:
            try: self.screen.addnstr(row, 0, text, width-1, style)
            except curses.error: pass

    def draw(self, force=False):
        now = self.startup.clock()
        if not force and now-self.last_draw < 0.1: return
        self.last_draw = now
        self.screen.erase()
        self.line(0, 'PHYSICAL ARM STARTUP | sequential zero + hold', curses.A_BOLD)
        self.line(1, f"Torque limit: {self.startup.profile['torque_limit']/10:g}% | "
                     f"Capture window: +/-{math.degrees(self.startup.max_start):g} deg, inside CAD bounds")
        self.line(3, ' ID  Joint              Current    Target    To move    Temp   Torque  State')
        for row, (servo_id, name) in enumerate(JOINTS.items(), 4):
            state = self.states.get(servo_id)
            label, style = 'WAITING', 0
            if servo_id in self.startup.held:
                label, style = 'LOCKED', self.colors.get('locked', curses.A_BOLD)
            elif servo_id == self.active:
                if self.locking:
                    label, style = 'LOCKING', self.colors.get('locking', curses.A_BOLD)
                else:
                    label = f'CAPTURE {self.progress*100:3.0f}%' if self.armed else 'POSITION / PAUSED'
                    style = self.colors.get('active', curses.A_BOLD)
            if state is None:
                text = f' {servo_id:2}  {name:<18} Reading feedback...'
            else:
                angle = math.degrees(self.startup.angle(servo_id, state['position']))
                text = (f" {servo_id:2}  {name:<18} {angle:+8.2f}   {0:7.2f}   {-angle:+8.2f}   "
                        f"{state['temperature_raw']:3}C   {'ON' if state['torque'] else 'OFF':<4}  {label}")
            self.line(row, text, style)
        self.line(11, 's = start/resume automatic capture | p = pause capture | q = abort and release')
        self.line(12, f'Bring the highlighted joint into the capture window and hold it still for {CAPTURE_DWELL_SECONDS:g} second.')
        self.line(13, 'It then moves to exact zero automatically. Earlier joints remain powered and held.')
        self.line(14, 'During LOCKING: q aborts; pause applies while positioning, before movement.')
        self.line(16, self.message)
        self.screen.refresh()

    def run(self):
        startup = self.startup
        startup.preflight()
        gate = CaptureWindow()
        for servo_id, name in JOINTS.items():
            self.active = servo_id
            gate.reset()
            self.progress = 0.0
            self.message = f'Position ID {servo_id} ({name}). Press s once when ready for automatic locking.' if not self.armed else f'Position ID {servo_id} ({name}); capture is active.'
            while True:
                self.key()
                startup.monitor()
                for current_id in JOINTS: startup.state(current_id)
                state = self.states[servo_id]
                if state['torque'] != 0:
                    raise RuntimeError(f'ID {servo_id}: torque unexpectedly enabled before capture')
                angle = startup.angle(servo_id, state['position'])
                limit = startup.model.find(f"joint[@name='{name}']/limit")
                eligible = (self.armed and abs(angle) <= startup.max_start and
                            float(limit.get('lower')) <= angle <= float(limit.get('upper')))
                self.progress = gate.update(startup.clock(), state['position'], eligible)
                self.draw(force=True)
                if self.progress >= 1.0: break
                startup.sleep(0.1)
            self.locking = True
            self.message = f'ID {servo_id}: moving to calibrated zero and holding.'
            self.draw(force=True)
            startup.lock_motor(servo_id)
            self.locking = False
            self.draw(force=True)
        startup.monitor()
        self.message = 'All six joints locked. Starting ROS, RViz and sliders with holds maintained.'
        self.draw(force=True)


def run_dashboard(startup):
    def run(screen):
        ui = Dashboard(screen, startup)
        startup.observer = ui.observe
        try:
            with redirect_stdout(ui): ui.run()
        finally:
            startup.observer = lambda servo_id, state: None
    curses.wrapper(run)
