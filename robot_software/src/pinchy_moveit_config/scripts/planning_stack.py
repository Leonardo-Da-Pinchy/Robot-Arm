#!/usr/bin/python3
"""Configure MoveIt from the existing live robot description, never a second model."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


def parameters(model):
    share = Path(get_package_share_directory('pinchy_moveit_config'))
    root = ET.fromstring(model)
    semantic = ET.parse(share/'config/arm.srdf').getroot()
    semantic.set('name', root.get('name'))
    from pinchy_motion.geometry import model_limits
    limits = model_limits(root)
    joints = {}
    for joint in root.findall("joint[@type='revolute']"):
        limit = joint.find('limit')
        joints[joint.get('name')] = dict(has_velocity_limits=True,
            max_velocity=limits[joint.get('name')]['velocity'],
            has_acceleration_limits=True, max_acceleration=limits[joint.get('name')]['acceleration'])
    load = lambda name: yaml.safe_load((share/'config'/name).read_text())
    result = dict(robot_description=model,
        robot_description_semantic=ET.tostring(semantic, encoding='unicode'),
        robot_description_kinematics=load('kinematics.yaml'),
        robot_description_planning=dict(joint_limits=joints),
        planning_pipelines=['ompl'], default_planning_pipeline='ompl',
        ompl=load('ompl_planning.yaml'), allow_trajectory_execution=False,
        publish_robot_description=False, publish_robot_description_semantic=True,
        disable_capabilities='move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupExecuteService move_group/MoveGroupMoveAction',
        planning_scene_monitor_options=dict(name='pinchy_planning_scene_monitor',
            robot_description='robot_description', joint_state_topic='/joint_states',
            attached_collision_object_topic='/attached_collision_object',
            publish_planning_scene_topic='/monitored_planning_scene',
            monitored_planning_scene_topic='/planning_scene', wait_for_initial_state_timeout=10.0),
        publish_planning_scene=True, publish_geometry_updates=True,
        publish_state_updates=True, publish_transforms_updates=True,
        octomap_resolution=0.02)
    result.update(load('moveit_controllers.yaml'))
    return result


def visualization_config():
    bringup = Path(get_package_share_directory('pinchy_bringup'))
    view = yaml.safe_load((bringup/'config/arm.rviz').read_text())
    view['Visualization Manager']['Tools'].insert(0,{'Class':'rviz_default_plugins/Interact'})
    view['Visualization Manager']['Displays'].extend([
        {'Class':'rviz_default_plugins/InteractiveMarkers','Name':'Draggable goal','Enabled':True,
         'Interactive Markers Namespace':'/pinchy_motion/goal','Show Descriptions':True,'Show Axes':False},
        {'Class':'rviz_default_plugins/MarkerArray','Name':'Working region (400 mm cube)','Enabled':True,
         'Topic':{'Value':'/pinchy_motion/working_region','Depth':1,'Durability Policy':'Transient Local',
                  'History Policy':'Keep Last','Reliability Policy':'Reliable'}},
        {'Class':'rviz_default_plugins/MarkerArray','Name':'Input bounds and kinematic samples (debug)','Enabled':False,
         'Topic':{'Value':'/pinchy_motion/workspace_markers','Depth':1,'Durability Policy':'Transient Local',
                  'History Policy':'Keep Last','Reliability Policy':'Reliable'}},
        {'Class':'rviz_default_plugins/RobotModel','Name':'Robot collision geometry (toggle)','Enabled':False,
         'Alpha':0.3,'Visual Enabled':False,'Collision Enabled':True,'Description Source':'Topic',
         'Description Topic':{'Value':'/robot_description','Depth':1,'Durability Policy':'Transient Local',
                              'History Policy':'Keep Last','Reliability Policy':'Reliable'}},
        {'Class':'moveit_rviz_plugin/PlanningScene','Name':'Collision world','Enabled':True,
         'Robot Description':'robot_description','Planning Scene Topic':'/monitored_planning_scene',
         'Scene Geometry':{'Show Scene Geometry':True,'Scene Alpha':0.6},
         'Scene Robot':{'Show Robot Visual':False,'Show Robot Collision':False}},
        {'Class':'moveit_rviz_plugin/Trajectory','Name':'Planned motion','Enabled':True,
         'Robot Description':'robot_description','Planned Path':{'Topic':'/display_planned_path'},
         'Loop Animation':True,'State Display Time':'0.05 s','Trajectory Trail':False},
        {'Class':'rviz_default_plugins/MarkerArray','Name':'Ordered path preview','Enabled':True,
         'Topic':{'Value':'/pinchy_motion/path_preview','Depth':1,'Durability Policy':'Transient Local',
                  'History Policy':'Keep Last','Reliability Policy':'Reliable'}},
        {'Class':'rviz_default_plugins/MarkerArray','Name':'Pose target','Enabled':True,
         'Topic':{'Value':'/pinchy_motion/markers','Depth':1,'Durability Policy':'Transient Local',
                  'History Policy':'Keep Last','Reliability Policy':'Reliable'}}])
    return view


def main():
    rclpy.init(); node = rclpy.create_node('pinchy_planning_stack')
    node.declare_parameter('hardware', False)
    node.declare_parameter('rviz', True)
    model = []
    node.create_subscription(String, '/robot_description', lambda msg: model.append(msg.data) if not model else None,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    status = node.create_publisher(String, '/pinchy_motion/planning_status',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    process = None; rviz = None
    try:
        deadline = time.monotonic()+30
        while not model and rclpy.ok() and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=0.1)
        if not model: raise RuntimeError('No live robot description; start bringup first')
        plugin = ET.fromstring(model[0]).find('./ros2_control/hardware/plugin')
        expected = 'pinchy_hardware/FeetechSystem' if node.get_parameter('hardware').value else 'mock_components/GenericSystem'
        if plugin is None or plugin.text != expected: raise RuntimeError('Live model does not match selected mock/hardware mode')
        with tempfile.TemporaryDirectory(prefix='pinchy-moveit-') as folder:
            config = Path(folder)/'parameters.yaml'
            values = parameters(model[0])
            config.write_text(yaml.safe_dump({'/**': {'ros__parameters': values}}))
            process = subprocess.Popen(['ros2','run','moveit_ros_move_group','move_group','--ros-args','--params-file',str(config)])
            if node.get_parameter('rviz').value:
                view = visualization_config()
                rviz_config = Path(folder)/'planning.rviz'; rviz_config.write_text(yaml.safe_dump(view, sort_keys=False))
                # RViz consumes geometry; it must not inherit move_group's
                # scene-publisher/controller settings and create a second world source.
                rviz_parameters = Path(folder)/'rviz-parameters.yaml'
                rviz_parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
                    key: value for key, value in values.items() if key.startswith('robot_description')}}}))
                rviz = subprocess.Popen(['ros2','run','rviz2','rviz2','-d',str(rviz_config),
                    '--ros-args','--params-file',str(rviz_parameters)])
            status.publish(String(data='STARTED'))
            node.get_logger().info('MoveIt uses the live calibrated model; execution stays in the powered-hold gateway')
            while rclpy.ok() and process.poll() is None: rclpy.spin_once(node, timeout_sec=0.1)
            if rclpy.ok(): raise RuntimeError(f'MoveIt exited {process.returncode}; hardware bringup remains independent')
    except KeyboardInterrupt: pass
    except Exception as error:
        node.get_logger().error(str(error)); status.publish(String(data='ERROR: '+str(error)))
        return 1
    finally:
        for child in (rviz, process):
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try: child.wait(timeout=8)
                except subprocess.TimeoutExpired: child.terminate(); child.wait(timeout=4)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0

if __name__ == '__main__': raise SystemExit(main())
