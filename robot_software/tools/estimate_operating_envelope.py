#!/usr/bin/python3
"""Read-only nominal joint-envelope estimate from CAD and the 12V servo curve.

No ROS node, motor connection or automatic operating-limit update. Bounds are
conditional sampled performance estimates, not verified safe operating limits.
"""
import argparse,json,math,time,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,default=ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf')
    parser.add_argument('--profile',type=Path,default=ROOT/'calibration/arm.json')
    parser.add_argument('--branch',type=Path,help='Optional previously selected geometric q(s) NPZ witness')
    parser.add_argument('--samples',type=int,default=1000)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not 100<=args.samples<=20000:parser.error('Samples must be 100..20000; finite sampling does not prove global safety')
    root=ET.parse(args.model).getroot();profile=json.loads(args.profile.read_text())
    names=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','pen_pitch'];joints=[root.find(f"joint[@name='{n}']") for n in names]
    def transform(element):
     t=np.eye(4);t[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ');t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix();return t
    origins=[transform(j.find('origin')) for j in joints];axes=[np.fromstring(j.find('axis').get('xyz'),sep=' ') for j in joints]
    bodies=[]
    for j in joints:
     link=root.find(f"link[@name='{j.find('child').get('link')}']");e=link.find('inertial');I=e.find('inertia');mat=np.array([[float(I.get('ixx')),float(I.get('ixy')),float(I.get('ixz'))],[float(I.get('ixy')),float(I.get('iyy')),float(I.get('iyz'))],[float(I.get('ixz')),float(I.get('iyz')),float(I.get('izz'))]])
     bodies.append((float(e.find('mass').get('value')),transform(e.find('origin')),mat))
    # base_link to first joint's parent includes fixed base geometry: first joint's
    # parent is base_link in this exported model; all six children form the chain.
    assert joints[0].find('parent').get('link')=='base_link'
    low=np.array([profile['joints'][n]['min_radians'] for n in names]);high=np.array([profile['joints'][n]['max_radians'] for n in names])

    started=time.monotonic()
    def dynamics(q):
     T=np.eye(4);jp=[];a=[];M=np.zeros((6,6));g=np.zeros(6)
     for k in range(6):
      pre=T@origins[k];jp.append(pre[:3,3]);a.append(pre[:3,:3]@axes[k]);rot=np.eye(4);rot[:3,:3]=Rotation.from_rotvec(axes[k]*q[k]).as_matrix();T=pre@rot
      frame=T@bodies[k][1];c=frame[:3,3];I=frame[:3,:3]@bodies[k][2]@frame[:3,:3].T
      Jp=np.zeros((3,6));Jr=np.zeros((3,6))
      for i in range(k+1):Jp[:,i]=np.cross(a[i],c-jp[i]);Jr[:,i]=a[i]
      M+=bodies[k][0]*Jp.T@Jp+Jr.T@I@Jr;g+=Jp.T@np.array([0,0,-9.80665*bodies[k][0]])
     return M,g
    states=np.vstack([np.zeros(6),low,high,np.random.default_rng(123).uniform(low,high,(args.samples,6))])
    # Include every input sample from the previously checked large circle branch.
    if args.branch is not None:states=np.vstack([states,np.load(args.branch)['q']])
    speed_bounds=np.full(6,np.inf);acc_bounds=np.full(6,np.inf);g_max=np.zeros(6);coupled_acc=np.zeros(6);cmax=np.zeros(6)
    # Datasheet electrical specification +/-10%: use the lower values for torque
    # and no-load speed. This remains a nominal CAD calculation, not certification.
    stall=.9*30*.0980665;omega0=np.deg2rad(.9*270);present_omega=np.deg2rad(90);present_alpha=np.deg2rad(360)
    for n,q in enumerate(states):
     M,g=dynamics(q);der=np.empty((6,6,6));h=1e-5
     for k in range(6):
      d=np.eye(6)[k]*h;der[:,:,k]=(dynamics(q+d)[0]-dynamics(q-d)[0])/(2*h)
     gamma=np.empty((6,6,6))
     for i in range(6):
      for j in range(6):
       for k in range(6):gamma[i,j,k]=.5*(der[i,j,k]+der[i,k,j]-der[j,k,i])
     G=np.abs(g);A=np.sum(abs(M),axis=1);C=np.sum(abs(gamma),axis=(1,2))
     g_max=np.maximum(g_max,G);coupled_acc=np.maximum(coupled_acc,A);cmax=np.maximum(cmax,C)
     for i in range(6):
      constant=G[i]+A[i]*present_alpha-stall
      if constant>=0:w=0.
      elif C[i]<1e-12:w=-constant/(stall/omega0)
      else:w=2*(-constant)/(stall/omega0+math.sqrt((stall/omega0)**2-4*C[i]*constant))
      speed_bounds[i]=min(speed_bounds[i],w)
      available=stall*(1-present_omega/omega0)-G[i]-C[i]*present_omega**2
      acc_bounds[i]=min(acc_bounds[i],max(0,available)/max(A[i],1e-12))
    rows=[dict(joint=name,conservative_sampled_simultaneous_speed_deg_s=float(np.rad2deg(speed_bounds[i])),conservative_sampled_simultaneous_acceleration_deg_s2=float(np.rad2deg(acc_bounds[i])),max_gravity_nm=float(g_max[i]),max_mass_matrix_abs_row_sum=float(coupled_acc[i]),max_coriolis_absolute_sum=float(cmax[i])) for i,name in enumerate(names)]
    result=dict(samples=len(states),elapsed_seconds=time.monotonic()-started,assumptions='Nominal CAD masses/inertias, no added pen or payload; sampled poses, absolute-value coupled dynamics bounds for simultaneous motion; 10% reduced stall torque and no-load speed; omits friction, motor rotor inertia and thermal duty. Performance estimate only, not measured safe limits.',joints=rows,global_speed_estimate_deg_s=float(np.rad2deg(min(speed_bounds))),global_acceleration_estimate_deg_s2=float(np.rad2deg(min(acc_bounds))))

    scales=[]
    for row in rows:
     G=row['max_gravity_nm'];A=row['max_mass_matrix_abs_row_sum'];C=row['max_coriolis_absolute_sum']
     b=A*present_alpha+stall*present_omega/omega0;c=G-stall;aa=C*present_omega**2
     factor=0. if c>=0 else (-c/b if aa<1e-12 else 2*(-c)/(b+math.sqrt(b*b-4*aa*c)))
     scales.append(factor);row['joint_compatible_scale']=factor
    scale=min(scales);estimated_v=90*scale;estimated_a=360*scale
    selected_v=max(90.,math.floor((estimated_v-10)*100)/100);selected_a=max(360.,math.floor((estimated_a-10)*10)/10)
    result['selected_joint_compatible_pair']=dict(common_relative_scale=scale,estimated_speed_deg_s=estimated_v,estimated_acceleration_deg_s2=estimated_a,speed_margin_deg_s=10.,acceleration_margin_deg_s2=10.,pre_task_floor_speed_deg_s=90.,pre_task_floor_acceleration_deg_s2=360.,software_speed_deg_s=selected_v,software_acceleration_deg_s2=selected_a,raw_profile_unchanged=True)
    result['method']='Upper bounds on absolute gravity, mass-matrix row sums and Coriolis coefficients across sampled poses. Common relative speed/acceleration scaling; independent extrema are NOT combined.'
    result['limitations']='Finite nominal CAD envelope at 12 V, no extra payload, friction, rotor inertia or thermal model. Family suffix/real duty not verified. Not a certified safe limit; floor rule can retain an unqualified existing ceiling.'
    result['sources']=['https://cdn.robotshop.com/media/F/Fit/RB-Fit-155/pdf/feetech_12v_30kg_cm_magnetic_encoding_servo_sts321_specification_pdf.pdf','https://files.seeedstudio.com/products/Feetech/108090003_FEETECH_ST-3215-C047-Datasheet.pdf']
    result['input_hashes']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.model,args.profile,args.branch) if p is not None}
    result['nominal_moving_mass_kg']=sum(b[0] for b in bodies)
    text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text)
    print(text,end='')

if __name__=='__main__':main()
