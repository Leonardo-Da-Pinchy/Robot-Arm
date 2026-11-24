"""Validated nominal collision geometry; no scene mutation or motor access."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point,Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh,MeshTriangle,SolidPrimitive


def finite_vector(value,label):
    if not isinstance(value,(list,tuple)) or len(value)!=3:raise ValueError(label+' needs three values')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value):raise ValueError('Invalid '+label)
    return list(map(float,value))


def positive(value,label):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:raise ValueError('Invalid '+label)
    return float(value)


def quarter_pipe_mesh(radius,height,thickness,angle_deg=90.,segments=48):
    """Closed annular sector about +X, axis Z, open concavity toward -X.

    Inner chords slightly shrink the cavity; circumscribed outer chords enclose
    the outer radius. This conservative faceting is for nominal dry-run scenes.
    """
    radius=positive(radius,'inner radius');height=positive(height,'height');thickness=positive(thickness,'wall thickness')
    angle_deg=positive(angle_deg,'arc angle')
    if angle_deg>180 or isinstance(segments,bool) or not isinstance(segments,int) or not 4<=segments<=256:raise ValueError('Invalid quarter-pipe discretization')
    half=math.radians(angle_deg)/2;outer=(radius+thickness)/math.cos(half/segments)
    mesh=Mesh()
    for angle in np.linspace(-half,half,segments+1):
        for r,z in ((radius,-height/2),(outer,-height/2),(radius,height/2),(outer,height/2)):
            mesh.vertices.append(Point(x=float(r*math.cos(angle)),y=float(r*math.sin(angle)),z=z))
    def quad(a,b,c,d):
        mesh.triangles.extend([MeshTriangle(vertex_indices=[a,b,c]),MeshTriangle(vertex_indices=[a,c,d])])
    for index in range(segments):
        a=index*4;b=a+4
        quad(a,a+2,b+2,b);quad(a+1,b+1,b+3,a+3)
        quad(a+2,a+3,b+3,b+2);quad(a,b,b+1,a+1)
    quad(0,1,3,2);last=segments*4;quad(last,last+2,last+3,last+1)
    return mesh


def collision_object(item):
    if not isinstance(item,dict) or not isinstance(item.get('id'),str) or not item['id']:raise ValueError('Scene object needs an ID')
    if not isinstance(item.get('frame'),str) or not item['frame']:raise ValueError('Scene object needs a frame')
    obj=CollisionObject();obj.id=item['id'];obj.header.frame_id=item['frame'];obj.operation=CollisionObject.ADD
    pose=Pose();pose.position.x,pose.position.y,pose.position.z=finite_vector(item.get('center'),'scene center')
    rotation=Rotation.from_euler('xyz',finite_vector(item.get('rpy_deg',[0.,0.,0.]),'scene rotation'),degrees=True).as_quat()
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,rotation)
    kind=item.get('type','box')
    if kind=='box':
        size=finite_vector(item.get('size'),'scene box size')
        if any(v<=0 for v in size):raise ValueError('Invalid scene box dimensions')
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=size)];obj.primitive_poses=[pose]
    elif kind=='cylinder':
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.CYLINDER,dimensions=[positive(item.get('height'),'cylinder height'),positive(item.get('radius'),'cylinder radius')])];obj.primitive_poses=[pose]
    elif kind=='quarter_pipe':
        obj.meshes=[quarter_pipe_mesh(item.get('inner_radius'),item.get('height'),item.get('wall_thickness'),item.get('angle_deg',90.),item.get('segments',48))];obj.mesh_poses=[pose]
    else:raise ValueError('Unknown scene object type: '+str(kind))
    return obj


def scene_objects(definition):
    if not isinstance(definition,dict) or not isinstance(definition.get('objects',[]),list):raise ValueError('Scene needs an object list')
    objects=[collision_object(item) for item in definition.get('objects',[])]
    if len({obj.id for obj in objects})!=len(objects):raise ValueError('Duplicate scene object ID')
    return objects
