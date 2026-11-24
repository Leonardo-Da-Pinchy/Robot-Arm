#!/usr/bin/python3
"""Read-only localization of low encoder progress in an existing motion log.

No ROS node, serial port, service calls or motor commands. Missing/stale layer
observations remain unknown; a downstream classification is not a mechanical
root-cause diagnosis. FK uses the shared robot model.
"""
import argparse,gzip,json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from pinchy_motion.geometry import forward_kinematics

ROOT=Path(__file__).resolve().parents[1]


def boundary(a,b,expected,minimum_ratio=.2):
    """Locate the first observed loss of progress; never fill missing layers."""
    if b.get('scaling',1.)<.99 or b.get('safety','READY').startswith('HOLD:'):return 'intentional_pause_or_safety_hold'
    for name,label in [('adapter_applied','adapter_targets_not_advancing'),('servo_goals','servo_goal_readback_not_advancing')]:
        first,last=a.get(name),b.get(name)
        if first is None or last is None:return name+'_unobserved_or_stale'
        if last<expected*minimum_ratio:return label
    return 'feedback_not_following_advancing_servo_goals'


def analyze(rows,model,window=.2,minimum_speed=.003):
    model=ET.fromstring(model) if isinstance(model,str) else model
    rows=sorted({r['stamp_ns']:r for r in rows}.values(),key=lambda r:r['stamp_ns'])
    if not rows:return []
    names=rows[0]['joints']
    def xyz(q):return forward_kinematics(model,dict(zip(names,q)))[:3,3]
    positions={field:[] for field in ('commanded','measured','adapter_applied','servo_goals')}
    for row in rows:
        for field in positions:
            v=row.get(field)
            if field in ('adapter_applied','servo_goals'):
                # Source and receipt ages both matter; the publisher can drop
                # samples under pressure rather than block hardware control.
                if v is None or v['age_s']>.1 or abs(row['stamp_ns']-v['stamp_ns'])>150_000_000:v=None
                else:v=v['positions']
            positions[field].append(None if v is None else xyz(v))
    stamps=np.array([r['stamp_ns']/1e9 for r in rows]);result=[];j=0;last_end=-1e20
    for i in range(len(rows)):
        while j<i and stamps[i]-stamps[j]>window:j+=1
        duration=stamps[i]-stamps[j]
        if duration<window*.8 or stamps[j]<last_end:continue
        if rows[i]['joints']!=names or rows[j]['joints']!=names:continue
        delta={field:None if vals[i] is None or vals[j] is None else float(np.linalg.norm(vals[i]-vals[j]))/duration for field,vals in positions.items()}
        expected=delta['commanded'];actual=delta['measured']
        if expected is None or actual is None or expected<minimum_speed or actual>=.2*expected:continue
        if min(r.get('scaling',1.) for r in rows[j:i+1])<.99:continue
        start={k:(None if v is None else 1.) for k,v in delta.items()};end=dict(delta)
        end.update(scaling=rows[i].get('scaling',1.),safety=rows[i].get('safety','READY'))
        result.append(dict(start_stamp_ns=rows[j]['stamp_ns'],end_stamp_ns=rows[i]['stamp_ns'],duration_s=duration,
            speed_mm_s={k:None if v is None else v*1000 for k,v in delta.items()},boundary=boundary(start,end,expected)))
        last_end=stamps[i]
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('log',type=Path)
    parser.add_argument('--model',type=Path,default=ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf')
    parser.add_argument('--output',type=Path);args=parser.parse_args()
    opener=gzip.open if args.log.suffix=='.gz' else open
    with opener(args.log,'rt') as stream:rows=[json.loads(line) for line in stream if line.strip()]
    result=dict(log=str(args.log),events=analyze(rows,args.model.read_text()),interpretation='Boundary evidence only; physical/firmware root cause requires further verification.')
    text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text)
    print(text,end='')

if __name__=='__main__':main()
