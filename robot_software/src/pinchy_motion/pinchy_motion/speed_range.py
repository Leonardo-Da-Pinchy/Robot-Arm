"""Smooth bounded TCP timing on an unchanged complete geometric joint path.

The LP selects squared path speed, not IK. Its result remains a candidate:
shared JTC polynomial, geometry, speed and collision checks are authoritative.
"""
import math,time
import numpy as np
from scipy.interpolate import BSpline,PchipInterpolator
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import linprog


class BoundedSpeedLaw:
    def __init__(self,length,minimum,maximum,acceleration,joint_spline,velocity_limits,acceleration_limits,stop=None,joint_names=None):
        began=time.monotonic()
        if stop and stop():raise RuntimeError('Stopped during bounded TCP timing')
        if not all(math.isfinite(v) and v>0 for v in (length,minimum,maximum,acceleration)) or minimum>=maximum:
            raise ValueError('TCP speed range requires finite 0 < minimum < maximum and positive acceleration')
        names=joint_names or [f'joint {i+1}' for i in range(len(velocity_limits))]
        if length/maximum>110:raise ValueError('Even the TCP range maximum exceeds the 110-second path budget')
        self.length=length;self.requested=maximum;self.minimum=minimum
        slack=min(.02*minimum,.2*(maximum-minimum));entry=minimum+slack;ceiling=maximum-slack
        samples=np.linspace(0,length,min(2049,max(101,int(math.ceil(length/.0005))+1)))
        samples=np.unique(np.r_[samples,joint_spline.x])
        qs=joint_spline(samples,1);qss=joint_spline(samples,2)
        vl=np.asarray(velocity_limits);al=np.asarray(acceleration_limits)
        # Conservative endpoint ramps to the band minimum; their geometric
        # windows are fixed before solving, never expanded to hide slow spots.
        available=.86*al-np.max(abs(qss),axis=0)*entry**2
        allowance=np.divide(available,np.maximum(np.max(abs(qs),axis=0),1e-12))
        ramp_acceleration=min(acceleration*.85,float(min(allowance)))
        if ramp_acceleration<=0:
            raise ValueError('TCP range minimum cannot be supported by the candidate curvature with smooth endpoint ramps; choose another configuration or lower the explicit minimum')
        self.entry=entry;self.ramp_time=math.pi*entry/(2*ramp_acceleration);self.ramp_distance=entry*self.ramp_time/2
        start=self.ramp_distance;end=length-self.ramp_distance
        if end-start<.001:
            raise ValueError('Stroke too short for the requested TCP range and endpoint ramps; no positive-speed interior retained')
        interior=np.linspace(start,end,min(2049,max(101,int(math.ceil((end-start)/.0005))+1)))
        interior=np.unique(np.r_[interior,joint_spline.x[(joint_spline.x>=start)&(joint_spline.x<=end)]])
        qs=joint_spline(interior,1);qss=joint_spline(interior,2)
        if len(interior)>2049:
            # Bound LP memory on long strokes. Retain a uniform spread and all
            # per-joint global derivative/curvature peaks. The unchanged dense
            # JTC validator remains authoritative before a preview can exist.
            keep=np.unique(np.r_[np.linspace(0,len(interior)-1,2000).astype(int),np.argmax(abs(qs),axis=0),np.argmax(abs(qss),axis=0)])
            interior=interior[keep];qs=qs[keep];qss=qss[keep]
        knots=np.r_[[start]*4,np.linspace(start,end,min(126,max(3,int(math.ceil((end-start)/.004))+1)))[1:-1],[end]*4]
        count=len(knots)-4;basis=BSpline(knots,np.eye(count),3);B=basis(interior);D=basis.derivative()(interior)
        scale=maximum**2;floor=entry**2/scale;upper=ceiling**2/scale
        caps=np.minimum(upper,np.min((.93*vl/np.maximum(abs(qs),1e-12))**2/scale,axis=1))
        if min(caps)<floor:
            index=int(np.argmin(caps));joint=int(np.argmin(vl/np.maximum(abs(qs[index]),1e-12)))
            raise ValueError(f'TCP range minimum exceeds {names[joint]} velocity near {100*interior[index]/length:.1f}% of the stroke; candidate upper bound {math.sqrt(max(0,caps[index])*scale)*1000:.3f} mm/s')
        matrices=[B];bounds=[caps]
        for j in range(len(vl)):
            M=(qss[:,j,None]*B+.5*qs[:,j,None]*D)*scale
            matrices.extend((M,-M));bounds.extend((np.full(len(interior),.86*al[j]),np.full(len(interior),.86*al[j])))
        # Derivative control coefficients bound tangential acceleration over
        # entire spline spans, not merely at the sampled constraint locations.
        derivative=np.zeros((count-1,count))
        for i in range(count-1):
            f=3/(knots[i+4]-knots[i+1]);derivative[i,i]=-f;derivative[i,i+1]=f
        matrices.extend((.5*scale*derivative,-.5*scale*derivative));bounds.extend((np.full(count-1,.9*acceleration),np.full(count-1,.9*acceleration)))
        equal=np.zeros((4,count));equal[0,0]=1;equal[1,1]=1;equal[2,-2]=1;equal[3,-1]=1
        if stop and stop():raise RuntimeError('Stopped during bounded TCP timing')
        result=linprog(-basis.integrate(start,end)/(end-start),A_ub=np.vstack(matrices),b_ub=np.concatenate(bounds),A_eq=equal,b_eq=np.full(4,floor),bounds=[(floor,upper)]*count,method='highs',options={'time_limit':5.})
        if stop and stop():raise RuntimeError('Stopped during bounded TCP timing')
        if not result.success:
            if result.status==1:raise ValueError('TCP range timing solver reached its 5-second budget; no plan retained')
            ratio=abs(qss)*entry**2/(.86*al)
            index,joint=np.unravel_index(int(np.argmax(ratio)),ratio.shape)
            raise ValueError(f'No bounded TCP timing profile passed minimum and acceleration constraints for this candidate; {names[joint]} curvature constraint near {100*interior[index]/length:.1f}%. The minimum was not relaxed')
        self.squared=BSpline(knots,result.x*scale,3);self.squared_derivative=self.squared.derivative()
        grid=np.linspace(start,end,min(20001,max(301,int(math.ceil((end-start)/.0001))+1)))
        elapsed=cumulative_trapezoid(1/np.sqrt(self.squared(grid)),x=grid,initial=0.)
        self.distance_at=PchipInterpolator(elapsed,grid);self.elapsed_at=PchipInterpolator(grid,elapsed)
        self.plateau_time=float(elapsed[-1]);self.duration=2*self.ramp_time+self.plateau_time
        self.solver_seconds=time.monotonic()-began
        speeds=np.sqrt(self.squared(grid));slow=speeds<maximum*.95
        self.planned_min=float(min(speeds));self.planned_max=float(max(speeds))
        self.slowdown_distance_fraction=float(np.mean(slow));self.slowdown_intervals=[]
        edges=np.diff(np.r_[False,slow,False].astype(int));starts=np.where(edges==1)[0];ends=np.where(edges==-1)[0]-1
        for a,b in zip(starts,ends):self.slowdown_intervals.append([float(grid[a]),float(grid[b])])

    def at(self,t):
        t=float(np.clip(t,0,self.duration));T=self.ramp_time;v=self.entry
        if t<T:
            z=math.pi*t/T
            return v/2*(t-T/math.pi*math.sin(z)),v/2*(1-math.cos(z)),v*math.pi/(2*T)*math.sin(z)
        if t<=T+self.plateau_time:
            s=float(self.distance_at(t-T));return s,math.sqrt(max(0,float(self.squared(s)))),.5*float(self.squared_derivative(s))
        rem=self.duration-t;z=math.pi*rem/T
        return self.length-v/2*(rem-T/math.pi*math.sin(z)),v/2*(1-math.cos(z)),-v*math.pi/(2*T)*math.sin(z)

    def time_at(self,s):
        from scipy.optimize import brentq
        if s<=0:return 0.
        if s>=self.length:return self.duration
        if self.ramp_distance<=s<=self.length-self.ramp_distance:return self.ramp_time+float(self.elapsed_at(s))
        return brentq(lambda t:self.at(t)[0]-s,0.,self.duration,xtol=1e-11)
