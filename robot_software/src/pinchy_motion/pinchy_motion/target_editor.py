"""Live 6-DOF goal editing. Every edit is visualization only; never plans/moves."""
import copy
import math
import threading
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped,Point,Quaternion
from visualization_msgs.msg import Marker,MarkerArray,InteractiveMarker,InteractiveMarkerControl,InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.qos import QoSProfile,DurabilityPolicy
from .workspace import configuration,inside


class TargetEditor:
    def __init__(self,node,changed,can_edit):
        self.node,self.changed,self.can_edit=node,changed,can_edit
        self.workspace=configuration()
        self.revision=0;self.target=None;self.marker_created=False;self.lock=threading.RLock()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub=node.create_publisher(PoseStamped,'/pinchy_motion/target_pose',qos)
        self.axes_pub=node.create_publisher(MarkerArray,'/pinchy_motion/markers',qos)
        node.create_subscription(PoseStamped,'/pinchy_motion/target_input',self.input,10)
        self.server=InteractiveMarkerServer(node,'/pinchy_motion/goal')

    def input(self,message):
        try:self.set_target(message)
        except ValueError as error:
            self.restore();self.node.get_logger().warn('Target edit rejected: '+str(error))

    def restore(self):
        with self.lock:
            if self.target is not None:
                self.server.setPose('pose_goal',self.target.pose);self.server.applyChanges()
                self.pose_pub.publish(self.target)

    def initialize(self,pose):
        if self.target is None:self.set_target(pose)

    def set_target(self,pose,force=False):
        with self.lock:
            if pose.header.frame_id!='base_link':raise ValueError('Target editor uses base_link')
            p=pose.pose.position;q=pose.pose.orientation
            values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            if not all(math.isfinite(v) for v in values) or abs(np.linalg.norm(values[3:])-1)>1e-3:
                raise ValueError('Expected a finite pose with normalized orientation')
            if not inside(values[:3],self.workspace):
                raise ValueError('Target outside numerical XYZ input bounds')
            if self.target is not None:
                old=self.target.pose;oq=old.orientation
                same=np.linalg.norm(np.array(values[:3])-[old.position.x,old.position.y,old.position.z])<1e-10
                same=same and abs(np.dot(values[3:],[oq.x,oq.y,oq.z,oq.w]))>1-1e-12
                if same:return self.revision
                if not force and not self.can_edit():
                    self.restore();return self.revision
            self.target=copy.deepcopy(pose);self.target.header.stamp=self.node.get_clock().now().to_msg()
            self.revision+=1
            self.render();self.pose_pub.publish(self.target)
            self.changed(self.revision)
            return self.revision

    def render(self):
        marker=InteractiveMarker();marker.header.frame_id='base_link';marker.name='pose_goal'
        marker.description='Goal only — move/rotate, then Plan';marker.scale=.15;marker.pose=self.target.pose
        center=InteractiveMarkerControl();center.name='target';center.always_visible=True
        dot=Marker();dot.type=Marker.SPHERE;dot.pose.orientation.w=1.
        dot.scale.x=dot.scale.y=dot.scale.z=.012;dot.color.r=1.;dot.color.g=.55;dot.color.a=.9
        center.markers=[dot];marker.controls.append(center)
        orientations=[Quaternion(w=1.),Quaternion(z=math.sqrt(.5),w=math.sqrt(.5)),Quaternion(y=-math.sqrt(.5),w=math.sqrt(.5))]
        for axis,orientation in zip('xyz',orientations):
            for mode,name in [(InteractiveMarkerControl.MOVE_AXIS,'move'),(InteractiveMarkerControl.ROTATE_AXIS,'rotate')]:
                control=InteractiveMarkerControl();control.name=name+'_'+axis
                control.orientation=orientation;control.orientation_mode=InteractiveMarkerControl.FIXED
                control.interaction_mode=mode;marker.controls.append(control)
        if not self.marker_created:
            self.server.insert(marker,feedback_callback=self.feedback);self.marker_created=True
        else:self.server.setPose('pose_goal',self.target.pose)
        self.server.applyChanges()
        q=self.target.pose.orientation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]);p=self.target.pose.position
        origin=np.array([p.x,p.y,p.z]);axes=[]
        for index,color in enumerate([(1.,0.,0.),(0.,1.,0.),(0.,0.,1.)]):
            arrow=Marker();arrow.header=self.target.header;arrow.ns='pose_target';arrow.id=index
            arrow.type=Marker.ARROW;arrow.action=Marker.ADD
            end=origin+rotation.as_matrix()[:,index]*.04
            arrow.points=[Point(x=float(origin[0]),y=float(origin[1]),z=float(origin[2])),Point(x=float(end[0]),y=float(end[1]),z=float(end[2]))]
            arrow.scale.x=.003;arrow.scale.y=.006;arrow.scale.z=.01
            arrow.color.r,arrow.color.g,arrow.color.b=color;arrow.color.a=1.;axes.append(arrow)
        self.axes_pub.publish(MarkerArray(markers=axes))

    def feedback(self,message):
        if message.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE,InteractiveMarkerFeedback.MOUSE_UP):return
        pose=PoseStamped();pose.header.frame_id='base_link';pose.pose=message.pose
        try:self.set_target(pose)
        except ValueError as error:
            self.restore();self.node.get_logger().warn(str(error))
