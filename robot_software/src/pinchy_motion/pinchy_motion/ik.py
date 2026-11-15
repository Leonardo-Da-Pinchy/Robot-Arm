"""Bounded, multi-start IK supplement to MoveIt's KDL plugin.

This module generates candidates only. MoveIt owns collision/path validation;
the existing controller and hardware adapter remain the only motor writer.
"""
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
from pinchy_bringup.control_contract import JOINTS

MODES = ('full', 'tool_axis', 'position')
POSITION_TOLERANCE = 0.0001  # model convergence, not physical accuracy
ANGLE_TOLERANCE = 0.001


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def free_twist_frames(target, reference_rotation, samples=24):
    """Full-pose candidates covering one revolution about the requested tool +z.

    Construct the tangent from the starting tool frame, not target roll. Thus
    arbitrary input rotation about the pen cannot bias free-twist semantics.
    These are numerical IK queries, never commanded waypoints.
    """
    direction=target[:3,2]
    tangent=reference_rotation[:,0]-direction*(reference_rotation[:,0]@direction)
    if np.linalg.norm(tangent)<1e-6:
        tangent=reference_rotation[:,1]-direction*(reference_rotation[:,1]@direction)
    tangent=tangent/np.linalg.norm(tangent)
    aligned=np.column_stack([tangent,np.cross(direction,tangent),direction])
    # Nearby twists first, then alternate both directions through the full circle.
    indices=[0]
    for index in range(1,samples//2+1):
        indices.extend([index,-index] if index*2!=samples else [index])
    for index in indices[:samples]:
        result=target.copy()
        result[:3,:3]=aligned@Rotation.from_rotvec([0.,0.,index*2*np.pi/samples]).as_matrix()
        yield result


class SearchStopped(RuntimeError):
    pass


class BoundedIK:
    def __init__(self, model, limits):
        root=ET.fromstring(model) if isinstance(model,str) else model
        self.lower=np.array([limits[n]['lower'] for n in JOINTS])
        self.upper=np.array([limits[n]['upper'] for n in JOINTS])
        chain=[];link='pointer_tip'
        while link!='base_link':
            joint=next(j for j in root.findall('joint') if j.find('child').get('link')==link)
            origin=joint.find('origin');matrix=np.eye(4)
            if origin is not None:
                matrix[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
                matrix[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            index=JOINTS.index(joint.get('name')) if joint.get('type') in ('revolute','continuous') else None
            axis=np.fromstring(joint.find('axis').get('xyz','1 0 0'),sep=' ') if index is not None else None
            chain.append((index,matrix,axis));link=joint.find('parent').get('link')
        self.chain=list(reversed(chain))
        self.global_seeds=qmc.Halton(d=6,scramble=True,seed=27).random(64)
        self.cache=[];self.last_report={}

    def evaluate(self,q):
        transform=np.eye(4);axes=np.zeros((6,3));origins=np.zeros((6,3))
        for index,origin,axis in self.chain:
            transform=transform@origin
            if index is not None:
                axes[index]=transform[:3,:3]@axis;origins[index]=transform[:3,3]
                rotation=np.eye(4);rotation[:3,:3]=Rotation.from_rotvec(axis*q[index]).as_matrix()
                transform=transform@rotation
        position_jac=np.cross(axes,transform[:3,3]-origins).T
        return transform,position_jac,axes.T

    def errors(self,q,target,mode):
        actual,_,_=self.evaluate(q)
        position=float(np.linalg.norm(actual[:3,3]-target[:3,3]))
        full=float(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude())
        axis=float(np.arccos(np.clip(actual[:3,2]@target[:3,2],-1,1)))
        return position, full if mode=='full' else axis if mode=='tool_axis' else 0.

    def system(self,q,target,mode):
        actual,jp,axes=self.evaluate(q)
        residual=(actual[:3,3]-target[:3,3])*10.;jacobian=jp*10.
        if mode=='full':
            residual=np.r_[residual,(actual[:3,:3]-target[:3,:3]).ravel()]
            jacobian=np.vstack([jacobian,np.column_stack([(skew(axes[:,i])@actual[:3,:3]).ravel() for i in range(6)])])
        elif mode=='tool_axis':
            residual=np.r_[residual,actual[:3,2]-target[:3,2]]
            jacobian=np.vstack([jacobian,np.column_stack([np.cross(axes[:,i],actual[:3,2]) for i in range(6)])])
        return residual,jacobian

    def remember(self,q):
        q=np.array(q)
        if all(np.linalg.norm(q-old)>.05 for old in self.cache):self.cache=([q]+self.cache)[:24]

    def search(self,target,start,mode='full',valid=lambda q:True,stop=lambda:False,budget=6.,attempts=64,anchor_weight=0.,position_tolerance=POSITION_TOLERANCE,angle_tolerance=ANGLE_TOLERANCE):
        if mode not in MODES:raise ValueError('Unknown orientation mode: '+mode)
        started=time.monotonic();deadline=started+budget;best=None;rejected=0;tries=0;reason='attempt limit'
        mid=(self.lower+self.upper)/2;span=self.upper-self.lower
        seeds=[np.array(start),*sorted(self.cache,key=lambda q:np.linalg.norm(q-start)),mid]
        seeds.extend(self.lower+row*span for row in self.global_seeds)
        def check():
            if stop():raise SearchStopped('IK search stopped')
            if time.monotonic()>deadline:raise TimeoutError
        def consider(q):
            nonlocal best,rejected,reason
            error=self.errors(q,target,mode)
            if best is None or error[0]*10+error[1]<best[0]*10+best[1]:best=(*error,q.copy())
            if error[0]<=position_tolerance and error[1]<=angle_tolerance:
                check()
                if valid(q.tolist()):self.remember(q);reason='solved';return q.tolist()
                rejected+=1
            return None
        try:
            for seed in seeds[:attempts]:
                check();tries+=1;seed=np.clip(seed,self.lower,self.upper)
                candidate=consider(seed)
                if candidate is not None:return candidate
                def fun(q):
                    check();residual=self.system(q,target,mode)[0]
                    return np.r_[residual,anchor_weight*(q-np.array(start))] if anchor_weight else residual
                def jac(q):
                    check();jacobian=self.system(q,target,mode)[1]
                    return np.vstack([jacobian,anchor_weight*np.eye(6)]) if anchor_weight else jacobian
                fit=least_squares(fun,np.clip(seed,self.lower+1e-9,self.upper-1e-9),jac=jac,
                    bounds=(self.lower,self.upper),max_nfev=120,ftol=1e-10,xtol=1e-10,gtol=1e-10)
                candidate=consider(fit.x)
                if candidate is not None:return candidate
        except TimeoutError:reason='time budget'
        finally:
            self.last_report=dict(mode=mode,attempts=tries,elapsed_s=time.monotonic()-started,
                rejected_candidates=rejected,reason=reason)
            if best is not None:
                self.last_report.update(best_position_error_mm=best[0]*1000,best_orientation_error_deg=float(np.degrees(best[1])),
                    limiting_joints=[n for n,v,l,h in zip(JOINTS,best[2],self.lower,self.upper) if min(v-l,h-v)<np.radians(1.)])
        return None

    def conditioning(self,q,mode='full'):
        transform,jp,axes=self.evaluate(q)
        # Nondimensionalize translation using a representative 0.4 m arm length.
        dimension=6
        if mode=='tool_axis':
            axes=np.cross(axes.T,transform[:3,2]).T;dimension=5
        matrix=jp/.4 if mode=='position' else np.vstack([jp/.4,axes])
        if mode=='position':dimension=3
        values=np.linalg.svd(matrix,compute_uv=False)[:dimension]
        return dict(mode=mode,task_dimensions=dimension,smallest_singular_value=float(values[-1]),condition_number=float(values[0]/max(values[-1],1e-12)),
                    near_singular=bool(values[-1]<.02),translation_scale_m=.4)
