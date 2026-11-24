"""XYZ/RPY pose panel: preview first, explicit execution, shared powered Stop."""
import json
import math
import signal
import sys
import time
from pathlib import Path
from python_qt_binding.QtCore import QTimer,Qt
from python_qt_binding.QtWidgets import QApplication,QWidget,QGridLayout,QLabel,QDoubleSpinBox,QPushButton,QComboBox,QCheckBox,QTableWidget,QTableWidgetItem,QHeaderView,QAbstractItemView,QVBoxLayout,QHBoxLayout,QFileDialog,QProgressBar
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped,Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger
from pinchy_motion_interfaces.srv import PlanPose,PlanPath
from pinchy_motion_interfaces.action import ExecutePlan
from pinchy_bringup.control_contract import MAX_SPEED,MAX_ACCELERATION,MAX_TCP_SPEED,MAX_TCP_ACCELERATION
from .geometry import quaternion_from_rpy
from .workspace import configuration as workspace_configuration
from . import point_sets
from .planning_progress import progress_text,duration_text


class PosePanel(QWidget):
    def __init__(self,node):
        super().__init__(); self.node=node
        self.hardware=node.declare_parameter('hardware',False).value
        self.executor=SingleThreadedExecutor(context=node.context); self.executor.add_node(node)
        self.workspace=workspace_configuration();self.syncing_target=False;self.limits_window=None
        self.target_pub=node.create_publisher(PoseStamped,'/pinchy_motion/target_input',10)
        self.queue_pub=node.create_publisher(String,'/pinchy_motion/queued_points',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.queue_timer=QTimer(self);self.queue_timer.setSingleShot(True);self.queue_timer.timeout.connect(self.publish_queue_snapshot)
        self.point_set_metadata={};self.point_set_path=None
        self.target_timer=QTimer(self);self.target_timer.setSingleShot(True);self.target_timer.timeout.connect(self.publish_target)
        self.planning_started=0.;self.planning_previous_id=None;self.planning_interrupted=False
        self.status_data={}; self.plan_id=''; self.busy=False; self.executing=False; self.goal=None; self.initialized=False;self.has_plan_result=False
        self.setWindowTitle('Pinchy REAL Cartesian poses' if self.hardware else 'Pinchy MOCK Cartesian poses')
        grid=QGridLayout(self)
        title=QLabel(('REAL hardware' if self.hardware else 'MOCK motors')+' | base_link → pointer_tip\n'
            'Plan previews only. Execute moves. Temporary RViz TCP; physical tip unverified.')
        grid.addWidget(title,0,0,1,4)
        self.inputs=[]
        for row,(name,low,high,decimals,suffix) in enumerate([
            ('X',-1.,1.,6,' m'),('Y',-1.,1.,6,' m'),('Z',-.2,1.2,6,' m'),
            ('Roll',-180.,180.,3,' deg'),('Pitch',-180.,180.,3,' deg'),('Yaw',-180.,180.,3,' deg')],1):
            box=QDoubleSpinBox();box.setRange(low,high);box.setDecimals(decimals);box.setSuffix(suffix)
            box.setSingleStep(.005 if row<=3 else 1.)
            box.valueChanged.connect(self.target_edited)
            self.inputs.append(box);grid.addWidget(QLabel(name),row,0);grid.addWidget(box,row,1)
        for i,box in enumerate(self.inputs[:3]):box.setRange(self.workspace['minimum_m'][i],self.workspace['maximum_m'][i])
        self.speed=QDoubleSpinBox();self.speed.setRange(.1,math.degrees(MAX_SPEED));self.speed.setValue(30.);self.speed.setSuffix(' deg/s')
        self.acceleration=QDoubleSpinBox();self.acceleration.setRange(.1,math.degrees(MAX_ACCELERATION));self.acceleration.setValue(180.);self.acceleration.setSuffix(' deg/s²')
        self.speed.valueChanged.connect(self.invalidate);self.acceleration.valueChanged.connect(self.invalidate)
        grid.addWidget(QLabel('Max joint speed'),1,2);grid.addWidget(self.speed,1,3)
        grid.addWidget(QLabel('Max joint acceleration'),2,2);grid.addWidget(self.acceleration,2,3)
        self.current=QLabel('Current TCP: waiting');self.current.setMinimumWidth(430);self.current.setWordWrap(True)
        grid.addWidget(self.current,3,2,2,2)
        copy_controls=QWidget();copy_layout=QHBoxLayout(copy_controls);copy_layout.setContentsMargins(0,0,0,0)
        copy_button=QPushButton('Copy current pose');copy_button.clicked.connect(self.copy_current);copy_layout.addWidget(copy_button)
        direction_button=QPushButton('Copy current direction');direction_button.clicked.connect(self.copy_current_direction)
        direction_button.setToolTip('Keep goal XYZ and replace RPY with current measured/model orientation. Does not plan or move.')
        copy_layout.addWidget(direction_button);grid.addWidget(copy_controls,5,2,1,2)
        limits_button=QPushButton('Joint limits / bounds');limits_button.clicked.connect(self.show_limits);grid.addWidget(limits_button,6,2,1,2)
        self.plan_button=QPushButton('Plan / preview');self.plan_button.setEnabled(False);self.plan_button.clicked.connect(self.plan)
        self.execute_button=QPushButton('Execute previewed plan');self.execute_button.setEnabled(False);self.execute_button.clicked.connect(self.execute)
        self.stop_button=QPushButton('Stop / hold');self.stop_button.clicked.connect(self.stop)
        self.recover_button=QPushButton('Recover / hold');self.recover_button.clicked.connect(lambda:self.stop(True))
        for column,button in enumerate((self.plan_button,self.execute_button,self.stop_button,self.recover_button)):grid.addWidget(button,8,column)
        self.orientation_mode=QComboBox()
        for label,code in [('Exact RPY','full'),('Pen direction / free twist','tool_axis'),('Position only / free orientation','position')]:self.orientation_mode.addItem(label,code)
        self.orientation_mode.setCurrentIndex(1)
        self.orientation_mode.setToolTip('Exact RPY never relaxes orientation. Pen direction constrains tool +z only. Position only ignores RPY.')
        self.orientation_mode.currentIndexChanged.connect(self.invalidate)
        self.orientation_mode.currentIndexChanged.connect(self.update_current_display)
        grid.addWidget(QLabel('Orientation constraint'),7,0);grid.addWidget(self.orientation_mode,7,1)
        self.straight_path=QCheckBox('Straight TCP path (LERP + SLERP)')
        self.straight_path.setToolTip('Unchecked: OMPL may curve around obstacles. Checked: straight tip line; Exact RPY uses quaternion SLERP, free twist interpolates only pen direction. Incomplete paths are rejected. Not contact drawing.')
        self.straight_path.toggled.connect(self.invalidate);grid.addWidget(self.straight_path,7,2,1,2)
        self.notice=QLabel('Waiting for model, MoveIt and controller');self.notice.setWordWrap(True)
        self.notice.setMinimumWidth(650)
        planning_box=QWidget();planning_layout=QVBoxLayout(planning_box);planning_layout.setContentsMargins(0,0,0,0)
        planning_layout.addWidget(self.notice)
        self.progress_bar=QProgressBar();self.progress_bar.setToolTip('Progress for the current stage only. Configuration search can retry; total remaining time is unknown.')
        self.progress_bar.hide();planning_layout.addWidget(self.progress_bar);grid.addWidget(planning_box,9,0,1,4)
        legend=QLabel('Edit fields or drag the goal arrows/rings in RViz (Interact tool, I); then Plan. Editing never moves the arm.\n'
            f"Blue working region: {self.workspace['design_working_region']['minimum_m']} to {self.workspace['design_working_region']['maximum_m']} m; reach/collisions still require Plan.\n"
            'Robot geometry and the green example floor are collision-checked. The floor is NOT a measured tabletop.')
        legend.setWordWrap(True);grid.addWidget(legend,10,0,1,4)
        self.queue=QTableWidget(0,7);self.queue.setHorizontalHeaderLabels(['X m','Y m','Z m','Roll deg','Pitch deg','Yaw deg','Contact'])
        self.queue.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);self.queue.setMinimumHeight(130);self.queue.setMaximumHeight(180)
        self.queue.itemChanged.connect(self.invalidate);grid.addWidget(self.queue,11,0,1,4)
        self.queue.itemSelectionChanged.connect(self.select_waypoint)
        bar=QWidget();buttons=QHBoxLayout(bar);buttons.setContentsMargins(0,0,0,0)
        self.queue_edit_buttons=[]
        for title,callback in [('Add goal',self.add_waypoint),('Update selected',self.update_waypoint),('Remove',self.remove_waypoint),('Up',lambda:self.move_waypoint(-1)),('Down',lambda:self.move_waypoint(1)),('Clear',self.clear_waypoints)]:
            button=QPushButton(title);button.clicked.connect(callback);buttons.addWidget(button);self.queue_edit_buttons.append(button)
        self.path_button=QPushButton('Plan queued path');self.path_button.clicked.connect(self.plan_queued);buttons.addWidget(self.path_button)
        grid.addWidget(bar,12,0,1,4)
        self.path_style=QComboBox()
        for title,code in [('Move between poses / pen-up routing','visits'),('Trace straight segments','polyline'),('Smooth through points','spline'),('Smooth points and pen direction','spline_axis'),('Fixed TCP rotations + approach','fixed_tcp')]:self.path_style.addItem(title,code)
        self.path_style.setToolTip('Default: collision-planned pen-up routes between destinations. Trace modes additionally constrain the complete TCP path. Smooth explicitly chooses another curve; contact corners require a lift or approved smooth curve.')
        self.path_style.currentIndexChanged.connect(self.invalidate);grid.addWidget(self.path_style,13,0,1,2)
        self.path_style.currentIndexChanged.connect(self.path_style_changed)
        self.tcp_speed=QDoubleSpinBox();self.tcp_speed.setRange(1.,MAX_TCP_SPEED*1000);self.tcp_speed.setValue(20.);self.tcp_speed.setSuffix(' mm/s TCP')
        self.tcp_acceleration=QDoubleSpinBox();self.tcp_acceleration.setRange(1.,MAX_TCP_ACCELERATION*1000);self.tcp_acceleration.setValue(50.);self.tcp_acceleration.setSuffix(' mm/s² TCP')
        for column,box in [(2,self.tcp_speed),(3,self.tcp_acceleration)]:box.valueChanged.connect(self.invalidate);grid.addWidget(box,13,column)
        self.stroke_approach=QCheckBox('Choose stroke start + pen-up route')
        self.stroke_approach.setToolTip('First queued point is the stroke start; its incoming segment must be pen-up. Search starting configurations for the complete remaining trace. Preview only until Execute.')
        self.contact_ramps=QCheckBox('Allow contact endpoint speed ramps')
        self.contact_ramps.setToolTip('Permit smooth acceleration/braking at stroke ends on the surface. Interior contact speed remains checked. Short strokes can be ramp-only and are reported.')
        for column,control in ((0,self.stroke_approach),(2,self.contact_ramps)):
            control.stateChanged.connect(self.invalidate);grid.addWidget(control,14,column,1,2)
        self.require_speed=QCheckBox('Require requested TCP cruise speed (continuous dry run)')
        self.require_speed.setToolTip('Check cruise speed within 5%, without automatically slowing the path. Endpoint ramps remain smooth; intermediate points do not stop.')
        self.require_speed.stateChanged.connect(self.invalidate);grid.addWidget(self.require_speed,15,0,1,4)
        self.range_summary=QLabel('Interior speed stays within the selected range; start/end ramps are exceptions.')
        self.range_summary.setWordWrap(True);grid.addWidget(self.range_summary,15,0,1,4);self.range_summary.hide()
        self.speed_range=QCheckBox('Use TCP speed range')
        self.speed_range.setToolTip('Explicitly vary interior speed between minimum and maximum, preserving geometry and normals. Endpoint ramps are exceptions. An infeasible minimum rejects the path.')
        self.tcp_min_speed=QDoubleSpinBox();self.tcp_min_speed.setRange(1.,MAX_TCP_SPEED*1000);self.tcp_min_speed.setValue(3.);self.tcp_min_speed.setSuffix(' mm/s min TCP')
        self.speed_range.stateChanged.connect(self.speed_range_changed);self.tcp_min_speed.valueChanged.connect(self.invalidate)
        grid.addWidget(self.speed_range,16,0,1,2);grid.addWidget(self.tcp_min_speed,16,2,1,2)
        explanation=QLabel('Queue: Add each goal, choose trace/curve/visits, then Plan queued path. Nothing moves until Execute.\nPrefer pen-up speed ramps; contact endpoint ramps require the explicit option. Physical contact remains disabled until qualified.')
        explanation.setWordWrap(True);grid.addWidget(explanation,17,0,1,4)
        files=QWidget();file_layout=QHBoxLayout(files);file_layout.setContentsMargins(0,0,0,0)
        self.file_buttons=[]
        for title,callback in [('Capture current TCP',self.capture_current_waypoint),('Save points…',self.save_points),('Load points…',self.load_points)]:
            button=QPushButton(title);button.clicked.connect(callback);file_layout.addWidget(button);self.file_buttons.append(button)
        self.example_sets=QComboBox();self.example_sets.addItem('Example point sets…','')
        self.example_dir=Path(get_package_share_directory('pinchy_motion'))/'examples/point_sets'
        for path in sorted(self.example_dir.glob('*.json')):
            self.example_sets.addItem(json.loads(path.read_text()).get('name',path.stem),str(path))
        file_layout.addWidget(self.example_sets)
        example_button=QPushButton('Load example');example_button.clicked.connect(self.load_example);file_layout.addWidget(example_button);self.file_buttons.append(example_button)
        grid.addWidget(files,18,0,1,4)
        self.point_set_label=QLabel('Unsaved point set — Plan queued path, inspect preview, then Execute previewed plan.');self.point_set_label.setWordWrap(True)
        grid.addWidget(self.point_set_label,19,0,1,4)
        self.path_planner=node.create_client(PlanPath,'/pinchy_motion/plan_path')
        self.planner=node.create_client(PlanPose,'/pinchy_motion/plan_pose')
        self.execution=ActionClient(node,ExecutePlan,'/pinchy_motion/execute_plan')
        self.stop_client=node.create_client(Trigger,'/pinchy_motion/stop')
        self.recovery=node.create_client(Trigger,'/pinchy_motion/recover')
        node.create_subscription(String,'/pinchy_motion/status',self.on_status,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(PoseStamped,'/pinchy_motion/target_pose',self.on_target,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_status=0.
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(20)
        self.queue_timer.start(0)

    def add_waypoint(self):
        if self.busy:return
        if self.queue.rowCount()>=80:self.notice.setText('Point sets support at most 80 points');return
        row=self.queue.rowCount();self.queue.insertRow(row)
        for column,box in enumerate(self.inputs):self.queue.setItem(row,column,QTableWidgetItem(str(box.value())))
        contact=QTableWidgetItem();contact.setFlags(Qt.ItemIsEnabled|Qt.ItemIsUserCheckable if not self.hardware else Qt.ItemIsEnabled)
        contact.setCheckState(Qt.Unchecked);contact.setToolTip('Contact on the segment approaching this waypoint. Physical contact execution is not qualified.')
        self.queue.setItem(row,6,contact);self.queue.selectRow(row);self.invalidate()

    def update_waypoint(self):
        if self.busy:return
        row=self.queue.currentRow()
        if row<0:return
        for column,box in enumerate(self.inputs):self.queue.item(row,column).setText(str(box.value()))
        self.invalidate()

    def select_waypoint(self):
        if self.busy:return
        row=self.queue.currentRow()
        if row<0:return
        try:values=[float(self.queue.item(row,column).text()) for column in range(6)]
        except (ValueError,AttributeError):return
        for box,value in zip(self.inputs,values):
            previous=box.blockSignals(True);box.setValue(value);box.blockSignals(previous)
        self.target_edited()

    def capture_current_waypoint(self):
        if self.busy:return
        if self.queue.rowCount()>=80:self.notice.setText('Point sets support at most 80 points');return
        pose=self.status_data.get('current_pose_xyz_rpy_deg')
        if pose is None or time.monotonic()-self.last_status>1.5:
            self.notice.setText('Waiting for fresh current TCP feedback');return
        self.append_waypoint(pose,False)
        self.notice.setText('Captured measured/model TCP as a queued point; no motion sent.')

    def path_style_changed(self):
        if self.path_style.currentData()=='fixed_tcp':
            self.orientation_mode.setCurrentIndex(self.orientation_mode.findData('full'))
            self.stroke_approach.setChecked(False);self.contact_ramps.setChecked(False)
        if self.path_style.currentData() in ('visits','fixed_tcp'):
            self.require_speed.setChecked(False)
            if hasattr(self,'speed_range'):self.speed_range.setChecked(False)

    def append_waypoint(self,values,contact):
        row=self.queue.rowCount();self.queue.insertRow(row)
        for column,value in enumerate(values):self.queue.setItem(row,column,QTableWidgetItem(str(value)))
        item=QTableWidgetItem();item.setFlags(Qt.ItemIsEnabled|Qt.ItemIsUserCheckable if not self.hardware else Qt.ItemIsEnabled)
        item.setCheckState(Qt.Checked if contact else Qt.Unchecked);self.queue.setItem(row,6,item)
        self.queue.selectRow(row);self.invalidate()

    def queue_document(self,allow_empty=False):
        data=dict(self.point_set_metadata)
        data.update(schema_version=1,frame='base_link',tcp_frame='pointer_tip',units={'position':'m','orientation':'deg'},
            orientation_mode=self.orientation_mode.currentData(),style=self.path_style.currentData(),
            joint_speed_deg_s=self.speed.value(),joint_acceleration_deg_s2=self.acceleration.value(),
            tcp_min_speed_m_s=self.tcp_min_speed.value()/1000. if self.speed_range.isChecked() else 0.,tcp_speed_m_s=self.tcp_speed.value()/1000.,tcp_acceleration_m_s2=self.tcp_acceleration.value()/1000.,
            allow_contact_ramps=self.contact_ramps.isChecked(),plan_pen_up_approach=self.stroke_approach.isChecked(),enforce_tcp_speed=self.require_speed.isChecked(),waypoints=[])
        for row in range(self.queue.rowCount()):
            values=[float(self.queue.item(row,column).text()) for column in range(6)]
            data['waypoints'].append(dict(xyz=values[:3],rpy_deg=values[3:],contact=self.queue.item(row,6).checkState()==Qt.Checked))
        return point_sets.normalize(data,self.workspace,allow_empty)

    def publish_queue_snapshot(self):
        try:data=dict(self.queue_document(allow_empty=True),queue_valid=True,snapshot_unix_s=time.time())
        except (ValueError,AttributeError,TypeError) as error:data=dict(queue_valid=False,error=str(error),snapshot_unix_s=time.time())
        self.queue_pub.publish(String(data=json.dumps(data,allow_nan=False)))

    def apply_point_set(self,data,path=None):
        if self.busy:raise ValueError('Wait for the active planning/execution request before replacing points')
        data=point_sets.normalize(data,self.workspace)
        if data['joint_speed_deg_s']>self.speed.maximum() or data['joint_acceleration_deg_s2']>self.acceleration.maximum():
            raise ValueError('Saved joint settings exceed this session’s caps')
        if self.hardware and any(p['contact'] for p in data['waypoints']):raise ValueError('Contact-tagged point sets remain mock-only')
        # Validate every field before replacing the existing queue. Loading
        # changes only staging data and goal markers, never an execution token.
        self.queue_timer.stop();self.queue.setRowCount(0)
        self.point_set_metadata={key:data[key] for key in ('name','notes','source') if key in data}
        self.orientation_mode.setCurrentIndex(self.orientation_mode.findData(data['orientation_mode']))
        self.path_style.setCurrentIndex(self.path_style.findData(data['style']))
        self.speed.setValue(data['joint_speed_deg_s']);self.acceleration.setValue(data['joint_acceleration_deg_s2'])
        self.tcp_speed.setValue(data['tcp_speed_m_s']*1000);self.tcp_acceleration.setValue(data['tcp_acceleration_m_s2']*1000)
        self.contact_ramps.setChecked(data['allow_contact_ramps']);self.stroke_approach.setChecked(data['plan_pen_up_approach'])
        self.require_speed.setChecked(data['enforce_tcp_speed'])
        self.speed_range.setChecked(data['tcp_min_speed_m_s']>0)
        if data['tcp_min_speed_m_s']>0:self.tcp_min_speed.setValue(data['tcp_min_speed_m_s']*1000)
        for point in data['waypoints']:self.append_waypoint([*point['xyz'],*point['rpy_deg']],point['contact'])
        self.queue.selectRow(0);self.point_set_path=Path(path) if path else None
        self.point_set_label.setText(data['name']+(' — '+str(path) if path else ''))
        self.invalidate();self.notice.setText(f'Loaded {self.queue.rowCount()} points. Plan queued path to preview; nothing moved.')

    def save_points(self):
        if self.busy:return
        try:
            data=self.queue_document()
            default=self.point_set_path or Path.home()/'pinchy-points.json'
            if default.resolve().is_relative_to(self.example_dir.resolve()):default=Path.home()/default.name
            path,_=QFileDialog.getSaveFileName(self,'Save point set',str(default),'Point sets (*.json)')
            if not path:return
            data['name']=Path(path).stem;point_sets.write(path,data,self.workspace)
            self.point_set_path=Path(path);self.point_set_metadata['name']=data['name'];self.point_set_label.setText(str(path))
            self.invalidate()
            self.notice.setText(f'Saved {self.queue.rowCount()} points; execution still requires a new checked preview.')
        except (ValueError,OSError,AttributeError,TypeError) as error:self.notice.setText(str(error))

    def load_points(self):
        if self.busy:return
        path,_=QFileDialog.getOpenFileName(self,'Load point set',str(self.point_set_path or Path.home()),'Point sets (*.json)')
        if path:
            try:self.apply_point_set(point_sets.read(path,self.workspace),path)
            except (ValueError,OSError) as error:self.notice.setText(str(error))

    def load_example(self):
        if self.busy:return
        path=self.example_sets.currentData()
        if path:
            try:self.apply_point_set(point_sets.read(path,self.workspace),path)
            except (ValueError,OSError) as error:self.notice.setText(str(error))

    def remove_waypoint(self):
        if self.busy:return
        if self.queue.currentRow()>=0:self.queue.removeRow(self.queue.currentRow());self.invalidate()

    def clear_waypoints(self):
        if self.busy:return
        self.queue.setRowCount(0);self.invalidate()

    def move_waypoint(self,offset):
        if self.busy:return
        row=self.queue.currentRow();destination=row+offset
        if row<0 or not 0<=destination<self.queue.rowCount():return
        for column in range(7):
            a=self.queue.takeItem(row,column);b=self.queue.takeItem(destination,column)
            self.queue.setItem(row,column,b);self.queue.setItem(destination,column,a)
        self.queue.selectRow(destination);self.invalidate()

    def plan_queued(self):
        if self.busy or not self.status_data.get('ready'):return
        try:
            request=PlanPath.Request(orientation_mode=self.orientation_mode.currentData(),style=self.path_style.currentData(),
                plan_pen_up_approach=self.stroke_approach.isChecked(),allow_contact_ramps=self.contact_ramps.isChecked(),
                enforce_tcp_speed=self.require_speed.isChecked(),
                tcp_min_speed=self.tcp_min_speed.value()/1000. if self.speed_range.isChecked() else 0.,tcp_speed=self.tcp_speed.value()/1000.,tcp_acceleration=self.tcp_acceleration.value()/1000.,
                velocity_scaling=min(1.,self.speed.value()/self.status_data.get('max_speed_deg_s',30.)),
                acceleration_scaling=min(1.,self.acceleration.value()/self.status_data.get('max_acceleration_deg_s2',180.)))
            for row in range(self.queue.rowCount()):
                values=[float(self.queue.item(row,column).text()) for column in range(6)]
                pose=PoseStamped();pose.header.frame_id='base_link';pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=values[:3]
                q=quaternion_from_rpy(values[3:]);pose.pose.orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
                request.waypoints.append(pose);request.contact_segments.append(self.queue.item(row,6).checkState()==Qt.Checked)
            self.invalidate();self.busy=True;self.target_timer.stop();self.begin_planning_display()
            self.path_planner.call_async(request).add_done_callback(self.planned)
        except Exception as error:self.busy=False;self.progress_bar.hide();self.notice.setText(str(error))

    def make_target(self):
        pose=PoseStamped();pose.header.frame_id='base_link'
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=[box.value() for box in self.inputs[:3]]
        q=quaternion_from_rpy([box.value() for box in self.inputs[3:]])
        pose.pose.orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3]);return pose

    def target_edited(self,*args):
        self.invalidate()
        self.update_current_display()
        if not self.syncing_target:self.target_timer.start(40)

    def publish_target(self):
        if not self.executing:self.target_pub.publish(self.make_target())

    def on_target(self,message):
        from scipy.spatial.transform import Rotation
        q=message.pose.orientation;p=message.pose.position
        values=[p.x,p.y,p.z,*Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz',degrees=True)]
        self.syncing_target=True;self.target_timer.stop()
        for box,value in zip(self.inputs,values):
            old=box.blockSignals(True);box.setValue(float(value));box.blockSignals(old)
        self.syncing_target=False;self.initialized=True;self.invalidate();self.update_current_display()

    def show_limits(self):
        if self.limits_window is None:
            self.limits_window=QWidget(self,Qt.Window);self.limits_window.setWindowTitle('Calibrated limits and planning bounds')
            layout=QVBoxLayout(self.limits_window)
            note=QLabel('These are saved usable joint bounds, not proof of absolute mechanical travel.\n'
                'Full XYZ + RPY reach also depends on configuration and collisions. Bounds are not changed here.')
            note.setWordWrap(True);layout.addWidget(note)
            self.limit_table=QTableWidget(6,5);self.limit_table.setHorizontalHeaderLabels(['Joint','Current deg','Minimum deg','Maximum deg','Reserve deg'])
            self.limit_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
            self.limit_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            layout.addWidget(self.limit_table)
            region=self.workspace['design_working_region']
            text=QLabel('Working region: '+str(region['minimum_m'])+' to '+str(region['maximum_m'])+' m (guidance, not guaranteed reachable).\n'
                'Numerical XYZ input box: '+str(self.workspace['minimum_m'])+' to '+str(self.workspace['maximum_m'])+' m.\n'
                'This box is not the reachable workspace. Cyan FK samples vary in orientation and are not collision-checked.\n'
                'RViz: enable Robot collision geometry (toggle) to inspect the robot collision proxies.')
            text.setWordWrap(True);layout.addWidget(text);self.limits_window.resize(760,400)
        self.update_limits();self.limits_window.show();self.limits_window.raise_()

    def update_limits(self):
        if self.limits_window is None:return
        for row,(name,values) in enumerate(self.status_data.get('joint_limits_deg',{}).items()):
            reserve=min(values['current']-values['minimum'],values['maximum']-values['current']) if values['current'] is not None else None
            for column,value in enumerate([name,values['current'],values['minimum'],values['maximum'],reserve]):
                item=self.limit_table.item(row,column)
                if item is None:item=QTableWidgetItem();self.limit_table.setItem(row,column,item)
                item.setText(value if isinstance(value,str) else ('waiting' if value is None else f'{value:+.2f}'))

    def invalidate(self,*args):
        self.plan_id='';self.execute_button.setEnabled(False) if hasattr(self,'execute_button') else None
        if self.has_plan_result and not self.busy:
            self.has_plan_result=False
            self.notice.setText('Inputs changed — plan again for the new goal or queue.')
        if hasattr(self,'queue_timer'):self.queue_timer.start(40)

    def on_status(self,message):
        self.last_status=time.monotonic();self.status_data=json.loads(message.data)
        self.update_limits()
        if self.status_data.get('hardware')!=self.hardware:
            self.status_data['ready']=False;self.notice.setText('Panel mode does not match running server');return
        pose=self.status_data.get('current_pose_xyz_rpy_deg')
        self.update_current_display()
        if pose:
            if not self.initialized:self.copy_current();self.initialized=True
        self.speed.setMaximum(self.status_data.get('max_speed_deg_s',30.))
        self.acceleration.setMaximum(self.status_data.get('max_acceleration_deg_s2',180.))
        if self.status_data.get('safety','').startswith('HOLD:'):
            self.invalidate();self.notice.setText(self.status_data['safety']+'; correct cause, then Recover / hold')
        elif not self.status_data.get('ready') and not self.busy:
            self.notice.setText(self.status_data.get('reason','Waiting'))

    def update_current_display(self,*args):
        if not hasattr(self,'current'):return
        pose=self.status_data.get('current_pose_xyz_rpy_deg')
        if not pose:
            self.current.setText('Current TCP (encoder/model): unavailable\nTCP speed (encoder/model): unavailable; commanded: unavailable')
            return
        from .geometry import goal_difference
        distance,angle=goal_difference(pose,[box.value() for box in self.inputs])
        text='Current TCP (encoder/model):\nXYZ '+', '.join(f'{v:+.4f}' for v in pose[:3])+' m\nRPY '+', '.join(f'{v:+.2f}' for v in pose[3:])+' deg'
        actual_speed=self.status_data.get('current_tcp_speed_m_s');commanded_speed=self.status_data.get('commanded_tcp_speed_m_s')
        def speed_text(value):
            return f'{value*1000:.1f} mm/s' if isinstance(value,(int,float)) and math.isfinite(value) and value>=0 else 'unavailable'
        text+='\nTCP speed (encoder/model): '+speed_text(actual_speed)+'; commanded: '+speed_text(commanded_speed)
        text+=f'\nGoal: {distance*1000:.1f} mm away'
        if self.orientation_mode.currentData()!='position':text+=f'; pen direction differs {math.degrees(angle):.1f}°'
        if self.status_data.get('current_conditioning_modes',{}).get(self.orientation_mode.currentData(),self.status_data.get('current_conditioning',{})).get('near_singular'):
            text+='\nNear-singularity warning: small tip moves may need large joint changes.'
        reserves=[(min(v['current']-v['minimum'],v['maximum']-v['current']),name) for name,v in self.status_data.get('joint_limits_deg',{}).items() if v['current'] is not None]
        if reserves:
            margin,name=min(reserves)
            if margin<5.:text+=f'\nLow joint travel reserve: {name} {margin:.1f}° to nearest saved limit'
        self.current.setText(text)

    def copy_current_direction(self):
        pose=self.status_data.get('current_pose_xyz_rpy_deg')
        if not pose or self.busy:return
        # Preserve XYZ; this is a deliberate edit, never automatic relaxation.
        for control,value in zip(self.inputs[3:],pose[3:]):
            previous=control.blockSignals(True);control.setValue(value);control.blockSignals(previous)
        self.target_edited()

    def copy_current(self):
        pose=self.status_data.get('current_pose_xyz_rpy_deg')
        if pose:
            for control,value in zip(self.inputs,pose):control.setValue(value)
            self.invalidate()

    def speed_range_changed(self,*args):
        ranged=self.speed_range.isChecked()
        self.tcp_speed.setSuffix(' mm/s max TCP' if ranged else ' mm/s TCP')
        self.require_speed.setVisible(not ranged);self.range_summary.setVisible(ranged)
        self.invalidate()

    def begin_planning_display(self):
        self.planning_started=time.monotonic();self.planning_interrupted=False
        self.planning_previous_id=self.status_data.get('planning_progress',{}).get('request_id')
        self.progress_bar.setRange(0,0);self.progress_bar.show();self.update_planning_display()

    def update_planning_display(self):
        if not self.busy or self.executing:return
        if self.planning_interrupted or self.status_data.get('safety','').startswith('HOLD:') or self.status_data.get('hardware',self.hardware)!=self.hardware:
            self.progress_bar.hide();return
        state=self.status_data.get('planning_progress',{})
        fresh=time.monotonic()-self.last_status<1.5
        if fresh and state.get('active') and state.get('request_id')!=self.planning_previous_id:
            self.notice.setText(progress_text(state))
            done,total=state.get('completed'),state.get('total')
            if total and done is not None:
                self.progress_bar.setRange(0,total);self.progress_bar.setValue(done);self.progress_bar.setFormat('%p% of this stage')
            else:self.progress_bar.setRange(0,0)
        else:
            self.progress_bar.setRange(0,0)
            self.notice.setText(f'Planning · {duration_text(time.monotonic()-self.planning_started)} elapsed\n'+
                ('Waiting for the planner’s next stage update…' if fresh else 'Planning status is stale; waiting for fresh feedback…'))
        self.progress_bar.show()

    def poll(self):
        self.executor.spin_once(timeout_sec=0)
        self.update_planning_display()
        ready=self.status_data.get('ready',False) and time.monotonic()-self.last_status<1.5
        self.plan_button.setEnabled(ready and not self.busy)
        self.path_button.setEnabled(ready and not self.busy and self.queue.rowCount()>0)
        for control in [self.queue,self.path_style,self.tcp_speed,self.tcp_acceleration,self.stroke_approach,self.contact_ramps]:control.setEnabled(not self.busy)
        for button in self.queue_edit_buttons:button.setEnabled(not self.busy)
        for button in self.file_buttons:button.setEnabled(not self.busy)
        self.example_sets.setEnabled(not self.busy)
        self.execute_button.setEnabled(ready and bool(self.plan_id) and not self.busy)
        for box in [*self.inputs,self.speed,self.acceleration,self.orientation_mode,self.straight_path]:box.setEnabled(not self.busy)
        for box in self.inputs[3:]:box.setEnabled(not self.busy and self.orientation_mode.currentData()!='position')
        self.orientation_mode.setEnabled(not self.busy and self.path_style.currentData()!='fixed_tcp')
        self.require_speed.setEnabled(not self.busy and self.path_style.currentData() in ('polyline','spline','spline_axis') and not self.speed_range.isChecked())
        continuous=self.path_style.currentData() in ('polyline','spline','spline_axis')
        self.speed_range.setEnabled(not self.busy and continuous)
        self.tcp_min_speed.setEnabled(not self.busy and continuous and self.speed_range.isChecked())
        if self.path_style.currentData()=='fixed_tcp':
            for control in (self.tcp_speed,self.tcp_acceleration,self.stroke_approach,self.contact_ramps):control.setEnabled(False)

    def plan(self):
        if self.busy or not self.status_data.get('ready'):return
        self.invalidate();self.busy=True;self.begin_planning_display()
        self.target_timer.stop();request=PlanPose.Request(target=self.make_target(),orientation_mode=self.orientation_mode.currentData(),
            path_mode='cartesian' if self.straight_path.isChecked() else 'joint')
        request.velocity_scaling=min(1.,self.speed.value()/self.status_data.get('max_speed_deg_s',30.))
        request.acceleration_scaling=min(1.,self.acceleration.value()/self.status_data.get('max_acceleration_deg_s2',180.))
        self.planner.call_async(request).add_done_callback(self.planned)

    def planned(self,future):
        self.busy=False;self.progress_bar.hide()
        try:
            result=future.result();self.plan_id=result.plan_id if result.success else ''
            self.has_plan_result=True
            self.notice.setText(f'{result.duration:.2f} s — {result.message} Preview in RViz, then Execute' if result.success else result.message)
        except Exception as error:self.notice.setText(str(error))

    def execute(self):
        if self.busy or not self.plan_id:return
        self.busy=True;self.executing=True;token=self.plan_id;self.invalidate()
        self.notice.setText('Checking current scene and start before execution…')
        self.execution.send_goal_async(ExecutePlan.Goal(plan_id=token,hardware_confirmed=self.hardware),
            feedback_callback=lambda msg:self.notice.setText(f'{msg.feedback.stage}: {msg.feedback.progress*100:.0f}%')).add_done_callback(self.accepted)

    def accepted(self,future):
        try:
            self.goal=future.result()
            if not self.goal.accepted:
                self.busy=self.executing=False;self.notice.setText('Plan rejected/expired; plan again');return
            self.goal.get_result_async().add_done_callback(self.completed)
        except Exception as error:self.busy=self.executing=False;self.notice.setText(str(error))

    def completed(self,future):
        self.busy=self.executing=False;self.goal=None
        try:self.notice.setText(future.result().result.message)
        except Exception as error:self.notice.setText(str(error))

    def stop(self,recover=False):
        self.invalidate();client=self.recovery if recover else self.stop_client
        if not client.service_is_ready():self.notice.setText('Motion gateway unavailable');return
        self.planning_interrupted=True;self.progress_bar.hide()
        self.notice.setText('Requesting powered hold…')
        def completed(future):
            try:self.notice.setText(future.result().message)
            except Exception as error:self.notice.setText(str(error))
        client.call_async(Trigger.Request()).add_done_callback(completed)

    def closeEvent(self,event):
        if self.executing:self.stop();event.ignore()
        else:
            self.timer.stop();self.queue_timer.stop();self.target_timer.stop()
            if self.limits_window is not None:self.limits_window.close()
            event.accept()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app=QApplication(sys.argv[:1]);node=rclpy.create_node('pinchy_pose_panel');window=PosePanel(node);window.show()
    def stop(signum,frame):
        if window.executing:
            window.stop()
            def wait():
                if window.executing:QTimer.singleShot(50,wait)
                else:app.quit()
            wait()
        else:app.quit()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:return app.exec_()
    finally:
        window.timer.stop();window.queue_timer.stop();window.target_timer.stop();window.executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
