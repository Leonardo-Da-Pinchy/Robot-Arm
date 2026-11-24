"""Thread-safe, read-only planning telemetry. Percent/ETA describe one stage."""
import threading
import time


class PlanningProgress:
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.lock=threading.Lock();self.active=False;self.data={}

    def begin(self,request_id,operation):
        with self.lock:
            self.started=self.phase_started=self.clock();self.active=True
            self.data=dict(request_id=request_id,operation=operation,phase='Checking request',completed=None,total=None,unit='')

    def update(self,phase,completed=None,total=None,unit='',**context):
        with self.lock:
            if not self.active:return
            old=self.data
            if phase!=old['phase'] or total!=old['total'] or unit!=old['unit'] or (completed is not None and old['completed'] is not None and completed<old['completed']):
                self.phase_started=self.clock()
            self.data.update(phase=phase,completed=completed,total=total,unit=unit)
            self.data.update(context)

    def end(self):
        with self.lock:self.active=False

    def snapshot(self):
        with self.lock:
            if not self.active:return dict(active=False)
            now=self.clock();result=dict(self.data,active=True,elapsed_s=max(0.,now-self.started),stage_elapsed_s=max(0.,now-self.phase_started),stage_remaining_s=None)
            done,total=result['completed'],result['total']
            if total and done is not None and 0<done<=total and result['stage_elapsed_s']>=2. and done>=8:
                result['stage_remaining_s']=result['stage_elapsed_s']*(total-done)/done
            return result


def duration_text(seconds):
    value=max(0,int(seconds));minutes,seconds=divmod(value,60)
    return f'{minutes}m {seconds:02d}s' if minutes else f'{seconds}s'


def progress_text(progress):
    phase=progress.get('phase','Working')
    phase={'starting IK':'Finding stroke starting configuration','continuous IK':'Solving continuous stroke IK',
        'bidirectional bottleneck search':'Trying alternate stroke configurations',
        'controller polynomial timing':'Timing complete stroke','stroke collision samples':'Checking stroke collisions',
        'pen-up approach collision samples':'Checking approach collisions',
        'complete trajectory and approach validation':'Checking whole-stroke candidate',
        'optimized trajectory and approach validation':'Checking optimized stroke',
        'whole-stroke optimization':'Optimizing stroke configurations'}.get(phase,phase)
    text=f"Planning · {duration_text(progress.get('elapsed_s',0.))} elapsed · overall ETA unknown\n{phase}"
    done,total,unit=progress.get('completed'),progress.get('total'),progress.get('unit','checks')
    if done is not None and total:
        text+=f' — {done:,}/{total:,} {unit} ({int(100*done/total)}% of this stage)'
    elif done is not None:text+=f' — {done:,} {unit}'
    detail=[]
    if progress.get('waypoint_index'):detail.append(f"Pose {progress['waypoint_index']}/{progress['waypoint_total']}")
    if progress.get('leg_index'):detail.append(f"Trace section {progress['leg_index']}/{progress['leg_total']}")
    if progress.get('configuration_attempt'):detail.append(f"Configuration attempt {progress['configuration_attempt']}/{progress.get('configuration_limit',8)} maximum")
    if progress.get('stage_remaining_s') is not None:
        detail.append(f"about {duration_text(progress['stage_remaining_s'])} left in this stage")
    if detail:text+='\n'+' · '.join(detail)
    return text
