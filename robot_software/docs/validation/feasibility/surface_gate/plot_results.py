#!/usr/bin/python3
"""Regenerate archived scientific plots; no ROS nodes, plans or execution."""
import gzip,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pinchy_motion.ik import BoundedIK
from pinchy_motion.geometry import model_limits

folder=Path(__file__).resolve().parent;root=Path(__file__).resolve().parents[4]
report=json.loads((folder/'pedestal_endpoints.json').read_text());routes=json.loads((folder/'boundary_routes.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(12,4.5))
for ax,case,surface in zip(axes,report['cases'],routes['surfaces']):
    kind=case['surface'];scale=180/(np.pi*(.04 if kind=='cylinder' else .1)) if kind!='plane' else 1000
    for good,color,label in ((True,'#24955b','Endpoint witness'),(False,'#c45642','Unresolved endpoint')):
        values=[p for p in case['sampled_grid'] if p['ik_collision_valid']==good]
        ax.scatter([p['u_m']*scale for p in values],[p['v_m']*1000 for p in values],s=26,color=color,label=label,zorder=3)
    for i,path in enumerate(surface['cases']):
        data=path['point_set'];p=np.array([p['xyz'] for p in data['waypoints']]);center=np.array(data['source']['center_m'])
        u=(np.arctan2(p[:,1]-center[1],center[0]-p[:,0])*.04 if kind=='cylinder' else
           np.arctan2(p[:,1]-center[1],p[:,0]-center[0])*.1 if kind=='quarter_pipe' else p[:,1]-center[1])
        ax.plot(u*scale,(p[:,2]-center[2])*1000,color='tab:blue',linewidth=2,label='Complete boundary/height path' if i==0 else None,zorder=2)
    ax.set(title=f'{kind.replace("_"," ")}: {case["sampled_valid_count"]}/{len(case["sampled_grid"])} endpoints',xlabel='Surface angle (deg)' if kind!='plane' else 'Width coordinate (mm)',ylabel='Height from center (mm)',ylim=(-52,52));ax.grid(alpha=.2);ax.legend(fontsize=6.5,loc='center left')
fig.suptitle('Proposed supports: discrete IK witnesses and nine complete paths; not arbitrary or physical coverage');fig.tight_layout();fig.savefig(folder/'boundary_checks.png',dpi=160);plt.close(fig)
model=(root/'src/pinchy_description/urdf/pinchy_arm.urdf').read_text();solver=BoundedIK(model,model_limits(model))
fig,axes=plt.subplots(1,3,figsize=(12,4))
for ax,kind in zip(axes,('cylinder','plane','quarter_pipe')):
    for backend in ('mock','pty'):
        result=json.loads((folder/f'{kind}_{backend}.json').read_text());start,end=result['offline_metrics']['trace_interval_s']
        with gzip.open(folder/f'{kind}_{backend}_controller.jsonl.gz','rt') as stream:rows=[json.loads(s) for s in stream]
        rows=[r for r in rows if start<=r['t']<=end]
        ax.plot([r['t']-start for r in rows],[np.linalg.norm(solver.evaluate(r['reference'])[1]@r['velocity'])*1000 for r in rows],label=backend+' controller reference',alpha=.8,linewidth=1.)
    ax.axhspan(2.85,3.15,color='green',alpha=.1);ax.axhline(3,color='black',linestyle='--',linewidth=.7)
    ax.set(title=kind.replace('_',' '),xlabel='Time in stroke (s)',ylabel='Commanded TCP speed (mm/s)',ylim=(-.1,3.4));ax.legend(fontsize=7)
fig.suptitle('Six full-system executions + separate retracts — requested 3 mm/s, ±5% cruise band; no physical motors');fig.tight_layout();fig.savefig(folder/'execution_speed.png',dpi=160);plt.close(fig)
