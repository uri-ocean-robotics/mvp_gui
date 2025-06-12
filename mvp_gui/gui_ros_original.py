import numpy as np
import rclpy
from rclpy.node import Node
import message_filters
from datetime import datetime
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from mvp_msgs.msg import Power, Waypoint, ControlProcess
from std_msgs.msg import Float64, Float32MultiArray, Int16
from tf_transformations import euler_from_quaternion
from mvp_gui import *
import yaml
from mvp_msgs.srv import GetState, ChangeState, GetWaypoints, SendWaypoints
from roslaunch_manager_interfaces.srv import GetLaunch, SetLaunch
from std_srvs.srv import Empty, Trigger, SetBool
import time
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sqlalchemy import func
import os
import subprocess

class GuiRos(Node):
    def __init__(self):
        super().__init__('mvp_gui_node')
        self.helm_state = 'start'
        self.helm_connected_states = []
        
        # ROS parameters from YAML
        self.get_params()
        
        # ROS subscribers, publishers, and services
        self.setup_ros()

        self.get_logger().info('mvp_gui_node has been started.')

    def get_params(self):
        # # The logic to read from a YAML file is not ROS-specific and remains the same.
        dataset_config = yaml.safe_load(open(global_file_name, 'r'))

        # # parse yaml file
        # self.name_space = '/' + dataset_config['name_space']
        self.ros_source = dataset_config['ros_source_base']
        topic_command = self.ros_source + 'ros2 topic list | grep remote'
        topic_result = subprocess.run(
            ['bash', '-c', topic_command],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
            timeout=SUBPROCESS_TIMEOUT
        )
        
        topic_result_split =  topic_result.stdout.splitlines()[0].split('/')
        self.name_space = '/' + topic_result_split[1]
        self.remote_id = topic_result_split[3]
        self.topic_ns = self.name_space + '/' + topic_result_split[2] + '/' + self.remote_id + '/'

        self.poses_source = self.topic_ns + dataset_config['poses_source']
        self.geo_pose_source = self.topic_ns + dataset_config['geo_pose_source']

        self.get_helm_state = self.topic_ns + dataset_config['helm_state']
        self.get_controller_state = self.topic_ns + dataset_config['controller_state']


        # self.poses_source = self.name_space + '/' + dataset_config['poses_source']
        # self.pose_setpoint_source = self.name_space + '/' + dataset_config['setpoint_source']
        # self.pose_value_source = self.name_space + '/' + dataset_config['value_source']

        # self.geo_pose_source = self.name_space + '/' + dataset_config['geo_pose_source']
        # self.vitals_source = self.name_space + '/' + dataset_config['vitals_source']

        # self.get_state_srv_name  = self.name_space + '/' + dataset_config['get_state_service']
        # self.change_state_srv_name  = self.name_space + '/' + dataset_config['change_state_service']

        # self.controller_srv_name  = self.name_space + '/' + dataset_config['controller_service']
        # self.controller_state_srv_name  = self.name_space + '/' + dataset_config['controller_state_service']

        # self.get_waypoint_srv_name  = self.name_space + '/' + dataset_config['get_waypoints_service']
        # self.pub_waypoint_srv_name = self.name_space + '/' + dataset_config['pub_waypoints_service']
        
        # self.get_power_port_srv_name = self.name_space + '/' + dataset_config['get_power_port_srv']

        # self.set_roslaunch_srv_name = self.name_space + '/' + dataset_config['set_launch']
        # self.get_roslaunch_srv_name = self.name_space + '/' + dataset_config['get_launch']

        # self.lumen_control_topic = self.name_space + '/' + dataset_config['lumen_control_topic']
        # self.geo_pose_secondary_topic = self.name_space + '/' + dataset_config['geo_pose_secondary_source']

        # self.pose_decay_time = dataset_config['pose_decay_time']
        # self.current_lumen = 1.0

    def vital_callback(self, msg):
        with app.app_context():
            vital = Vitals.query.first()
            if vital is None: # Create if not exists
                vital = Vitals(id=1, name=self.name_space, voltage=0.0, current=0.0)
                db.session.add(vital)
            vital.name = self.name_space
            vital.voltage = msg.data[0]
            vital.current = msg.data[1]
            db.session.commit() 
    
    def setpoint_callback(self, msg):
        with app.app_context():
            setpoint_pose = PoseSetpoint.query.first()
            if setpoint_pose is None: # Create if not exists
                setpoint_pose = PoseSetpoint(id=1)
                db.session.add(setpoint_pose)
            setpoint_pose.frame_id = msg.header.frame_id
            setpoint_pose.roll = msg.orientation.x * 180 / np.pi
            setpoint_pose.pitch = msg.orientation.y * 180 / np.pi 
            setpoint_pose.yaw = msg.orientation.z * 180 / np.pi
            setpoint_pose.x = msg.position.x
            setpoint_pose.y = msg.position.y
            setpoint_pose.z = msg.position.z
            setpoint_pose.u = msg.velocity.x
            setpoint_pose.v = msg.velocity.y
            setpoint_pose.w = msg.velocity.z
            setpoint_pose.p = msg.angular_rate.x
            setpoint_pose.q = msg.angular_rate.y
            setpoint_pose.r = msg.angular_rate.z
            db.session.commit() 

    def value_callback(self, msg):
        with app.app_context():
            value_pose = PoseValue.query.first()
            if value_pose is None: # Create if not exists
                value_pose = PoseValue(id=1)
                db.session.add(value_pose)
            value_pose.frame_id = msg.header.frame_id
            value_pose.roll = msg.orientation.x * 180 / np.pi
            value_pose.pitch = msg.orientation.y * 180 / np.pi 
            value_pose.yaw = msg.orientation.z * 180 / np.pi
            value_pose.x = msg.position.x
            value_pose.y = msg.position.y
            value_pose.z = msg.position.z
            value_pose.u = msg.velocity.x
            value_pose.v = msg.velocity.y
            value_pose.w = msg.velocity.z
            value_pose.p = msg.angular_rate.x
            value_pose.q = msg.angular_rate.y
            value_pose.r = msg.angular_rate.z
            db.session.commit() 

    def pose_callback(self, poses_sub, geo_pose_sub):
        quad = [geo_pose_sub.pose.orientation.x, 
                geo_pose_sub.pose.orientation.y, 
                geo_pose_sub.pose.orientation.z, 
                geo_pose_sub.pose.orientation.w]
        euler_angles = euler_from_quaternion(quad)
        
        with app.app_context():
            poses = Poses.query.first()
            if poses is None: # Create if not exists
                poses = Poses(id=1)
                db.session.add(poses)
            poses.frame_id = poses_sub.header.frame_id
            poses.child_frame_id = poses_sub.child_frame_id
            poses.roll = euler_angles[0] * 180 / np.pi
            poses.pitch = euler_angles[1] * 180 / np.pi 
            poses.yaw = euler_angles[2] * 180 / np.pi
            poses.x = poses_sub.pose.pose.position.x
            poses.y = poses_sub.pose.pose.position.y
            poses.z = geo_pose_sub.pose.position.altitude
            poses.u = poses_sub.twist.twist.linear.x
            poses.v = poses_sub.twist.twist.linear.y
            poses.w = poses_sub.twist.twist.linear.z
            poses.p = poses_sub.twist.twist.angular.x
            poses.q = poses_sub.twist.twist.angular.y
            poses.r = poses_sub.twist.twist.angular.z
            if geo_pose_sub.pose.position.latitude is not None and geo_pose_sub.pose.position.longitude is not None:
                poses.lat = geo_pose_sub.pose.position.latitude
                poses.lon = geo_pose_sub.pose.position.longitude
            else:
                poses.lat = 0.0
                poses.lon = 0.0
            db.session.commit()

    def setup_ros(self):
        # Use a reentrant callback group to allow service calls from within callbacks/timers
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.poses_sub = message_filters.Subscriber(self, Odometry, self.poses_source)
        self.geo_pose_sub = message_filters.Subscriber(self, GeoPoseStamped, self.geo_pose_source)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.poses_sub, self.geo_pose_sub], 10, 0.1)
        self.ts.registerCallback(self.pose_callback)

        self.create_subscription(ControlProcess, self.pose_setpoint_source, self.setpoint_callback, 10, callback_group=self.callback_group)
        self.create_subscription(ControlProcess, self.pose_value_source, self.value_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Float32MultiArray, self.vitals_source, self.vital_callback, 10, callback_group=self.callback_group)
        self.create_subscription(GeoPoseStamped, self.geo_pose_secondary_topic, self.geo_pose_secondary_callback, 10, callback_group=self.callback_group)

        # Publisher
        self.lumen_pub = self.create_publisher(Float64, self.lumen_control_topic, 10)

        self.action_list = ['change_state', 'controller_state', 'publish_waypoints', 'get_topics', 'rosnode_cleanup', 'set_power']
        
        with app.app_context():
            for action_item in self.action_list:
                action = RosActions.query.filter_by(action=action_item).first()
                if action:
                    action.pending = 0
                    db.session.commit()

            lumen_item = LedItems.query.first()
            if lumen_item:
                lumen_item.status = 1.0
                self.current_lumen = float(lumen_item.status)
                db.session.commit()
    
    # This main loop will be run in a separate thread
    def run(self):
        while rclpy.ok():
            try:
                self.roslaunch_file()
                self.activate_roslaunch_list()
                self.get_power_port()
                self.set_power_port()
                self.set_lumen()
                self.get_state()
                self.change_state()
                self.get_controller_state()
                self.change_controller_state()
                self.get_waypoints()
                self.publish_waypoints()
                self.log_poses()
            except Exception as e:
                self.get_logger().warn(f"Error in main loop: {e}", throttle_duration_sec=5)
            
            time.sleep(0.5)

    def call_service_safely(self, service_name, service_type, request=None, timeout_sec=2.0):
        client = self.create_client(service_type, service_name, callback_group=self.callback_group)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f"Service '{service_name}' not available after waiting.")
            return None
        
        if request is None:
            # Create a default request if none is provided
            request = service_type.Request()

        future = client.call_async(request)
        
        # This is a simplified way to wait for the future in a non-async function
        # running in a separate thread from the executor.
        start_time = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"Service call to '{service_name}' timed out.")
                return None
            time.sleep(0.01)
            
        if future.done():
            try:
                response = future.result()
                return response
            except Exception as e:
                self.get_logger().error(f"Service call to '{service_name}' failed: {e}")
                return None
        return None

    # Ros Services
    def get_state(self):
        with app.app_context():
            req = GetState.Request()
            # req.state.name = "" # Proactively fixed this from the original conversion
            response = self.call_service_safely(self.get_state_srv_name, GetState, req)
            
            if response:
                db.session.query(HelmStates).delete()
                db.session.commit()
                
                state = HelmStates(id=0, name=str(response.state.name))
                self.helm_state = response.state.name
                db.session.add(state)
                db.session.commit()
                
                for count, state_name in enumerate(response.state.transitions):
                    state = HelmStates(id=count+1, name=state_name)
                    db.session.add(state)
                db.session.commit()

    def change_state(self):
        with app.app_context():
            change_state_action = RosActions.query.filter_by(action='change_state', pending=1).first()
            if change_state_action:
                req = ChangeState.Request()
                req.state = change_state_action.value
                req.caller = self.get_name()
                response = self.call_service_safely(self.change_state_srv_name, ChangeState, req)
                if response:
                    change_state_action.pending = 0
                    db.session.commit()
                    self.get_logger().info(f"Helm State changed to {change_state_action.value}")

    def get_controller_state(self):
        with app.app_context():
            response = self.call_service_safely(self.controller_state_srv_name, Trigger)
            if response and response.success:
                controller_state = ControllerState.query.first()
                if controller_state is None:
                    controller_state = ControllerState(id=1)
                    db.session.add(controller_state)
                controller_state.state = response.message
                db.session.commit()

    def change_controller_state(self):
        with app.app_context():
            controller_state_action = RosActions.query.filter_by(action='controller_state', pending=1).first()
            if controller_state_action:
                service_name = self.controller_srv_name + '/' + controller_state_action.value
                response = self.call_service_safely(service_name, Empty)
                if response is not None:
                    controller_state_action.pending = 0
                    db.session.commit()
                    self.get_logger().info(f"Controller State changed to {controller_state_action.value}")

    def get_power_port(self):
        with app.app_context():
            response = self.call_service_safely(self.get_power_port_srv_name, Trigger)
            if response and response.success:
                db.session.query(PowerItems).delete()
                power_list = response.message.splitlines()
                for count, line in enumerate(power_list):
                    parts = line.split('=')
                    if len(parts) == 2:
                        name, status = parts
                        power_item = PowerItems(id=count, name=name, status=status)
                        db.session.add(power_item)
                db.session.commit()

    def set_power_port(self):
        with app.app_context():
            set_power_action = RosActions.query.filter_by(action='set_power', pending=1).first()
            if set_power_action:
                try:
                    parts = set_power_action.value.split('=')
                    if len(parts) != 2:
                        self.get_logger().error(f"Invalid power command format: {set_power_action.value}")
                        set_power_action.pending = 0
                        db.session.commit()
                        return
                        
                    srv_name, set_status_str = parts
                    service_path = self.name_space + '/' + srv_name
                    
                    req = SetBool.Request()
                    req.data = set_status_str.lower() != "false"
                    
                    response = self.call_service_safely(service_path, SetBool, req)
                    
                    if response:
                        if response.success:
                            self.get_logger().info(f"Set power port {srv_name} to {set_status_str}: {response.message}")
                        else:
                            self.get_logger().warn(f"Service {srv_name} reported failure: {response.message}")
                            
                    set_power_action.pending = 0
                    db.session.commit()
                except Exception as e:
                    self.get_logger().error(f"Error processing power command: {e}")
                    set_power_action.pending = 0
                    db.session.commit()

    def set_lumen(self):
        with app.app_context():
            lumen_item = LedItems.query.first()
            if lumen_item and float(lumen_item.status) != self.current_lumen:
                self.current_lumen = float(lumen_item.status)
                lumen_msg = Float64()
                lumen_msg.data = self.current_lumen
                for _ in range(3):
                    self.lumen_pub.publish(lumen_msg)

    def get_waypoints(self):
        with app.app_context():
            req = GetWaypoints.Request()
            # CORRECTED LINE: The field is likely named 'data', not 'source.data'.
            # req.data = 0
            response = self.call_service_safely(self.get_waypoint_srv_name, GetWaypoints, req)
            if response:
                db.session.query(CurrentWaypoints).delete()
                db.session.commit()
                for count, wpt in enumerate(response.wpt):
                    p = CurrentWaypoints(id=count, 
                                        lat = wpt.ll_wpt.latitude, 
                                        lon = wpt.ll_wpt.longitude, 
                                        alt = wpt.ll_wpt.altitude)
                    db.session.add(p)
                db.session.commit()

    def publish_waypoints(self):
        with app.app_context():
            pub_wpt_action = RosActions.query.filter_by(action='publish_waypoints', pending=1).first()
            if pub_wpt_action:
                waypoints = Waypoints.query.order_by(Waypoints.id).all()
                req = SendWaypoints.Request()
                req.type = 'geopath'
                for count, entry in enumerate(waypoints):
                    wpt = Waypoint()
                    wpt.header.seq = count
                    wpt.ll_wpt.latitude = float(entry.lat)
                    wpt.ll_wpt.longitude = float(entry.lon)
                    wpt.ll_wpt.altitude = float(entry.alt)
                    req.wpt.append(wpt)

                response = self.call_service_safely(self.pub_waypoint_srv_name, SendWaypoints, req)
                if response:
                    pub_wpt_action.pending = 0
                    db.session.commit()
                    self.get_logger().info(f"Published {len(req.wpt)} waypoints.")

    def log_poses(self):
        with app.app_context():
            new_pose = db.session.query(Poses).first()
            if not new_pose: return

            db.session.query(PoseHistory).update({PoseHistory.id: PoseHistory.id + 1})
            db.session.query(PoseHistory).filter(PoseHistory.id > self.pose_decay_time).delete()

            new_pose_history = PoseHistory(
                id=1, frame_id=new_pose.frame_id, child_frame_id=new_pose.child_frame_id,
                roll=new_pose.roll, pitch=new_pose.pitch, yaw=new_pose.yaw,
                x=new_pose.x, y=new_pose.y, z=new_pose.z,
                u=new_pose.u, v=new_pose.v, w=new_pose.w,
                p=new_pose.p, q=new_pose.q, r=new_pose.r,
                lat=new_pose.lat, lon=new_pose.lon
            )
            db.session.add(new_pose_history)
            db.session.commit()

    def geo_pose_secondary_callback(self, msg):
        with app.app_context():
            geo_pose_secondary = PoseSecondary.query.first()
            if geo_pose_secondary is None:
                geo_pose_secondary = PoseSecondary(id=1, lat=0.0, lon=0.0, z=0.0)
                db.session.add(geo_pose_secondary)

            geo_pose_secondary.lat = msg.pose.position.latitude
            geo_pose_secondary.lon = msg.pose.position.longitude
            geo_pose_secondary.z = msg.pose.position.altitude
            
            db.session.query(PoseHistorySecondary).update({PoseHistorySecondary.id: PoseHistorySecondary.id + 1})
            db.session.query(PoseHistorySecondary).filter(PoseHistorySecondary.id > self.pose_decay_time).delete()

            geo_pose_secondary_history = PoseHistorySecondary(
                id=1, lat=msg.pose.position.latitude, 
                lon=msg.pose.position.longitude, z=msg.pose.position.altitude
            )
            db.session.add(geo_pose_secondary_history)
            db.session.commit()
    
    def roslaunch_file(self):
        with app.app_context():
            launch_files_list = RosLaunchList.query.filter_by(pending=1).all()
            for launch_file in launch_files_list:
                launch_file_dir = launch_file.folder_dir.replace('~', os.path.expanduser('~'), 1)
                launch_file_full_path = os.path.join(launch_file_dir, launch_file.name)

                req = SetLaunch.Request()
                req.launch_file = launch_file_full_path
                req.start = True
                response = self.call_service_safely(self.set_roslaunch_srv_name, SetLaunch, req)
                
                if response and response.success:
                    self.get_logger().info(f"Successfully launched: {launch_file.name}")
                    launch_file.pending = 0
                else:
                    self.get_logger().warn(f"Failed to launch: {launch_file.name}")
                    launch_file.pending = 0 # Also set to 0 to avoid retries
                db.session.commit()
                time.sleep(1.0)
    
    def activate_roslaunch_list(self):
        with app.app_context():
            response = self.call_service_safely(self.get_roslaunch_srv_name, GetLaunch)
            
            if response:
                active_launches_db_map = {launch.full_path: launch for launch in RosActiveLaunchList.query.all()}
                service_launch_set = set(response.list)
                
                # Remove launches from DB that are no longer active according to the service
                for full_path, launch_obj in list(active_launches_db_map.items()):
                    if full_path not in service_launch_set:
                        db.session.delete(launch_obj)
                        self.get_logger().info(f"Removed inactive launch file from DB: {full_path}")
                
                # Add new active launches from the service to the DB
                current_db_paths = set(active_launches_db_map.keys())
                for full_path in service_launch_set:
                    if full_path not in current_db_paths:
                        new_id = (db.session.query(func.max(RosActiveLaunchList.id)).scalar() or 0) + 1
                        new_record = RosActiveLaunchList(id=new_id, full_path=full_path, pending=0)
                        db.session.add(new_record)
                        self.get_logger().info(f"Added new active launch file to DB: {full_path}")
                db.session.commit()

                # Process stop requests for active launches
                pending_launches = RosActiveLaunchList.query.filter_by(pending=1).all()
                for launch in pending_launches:
                    req = SetLaunch.Request()
                    req.launch_file = launch.full_path
                    req.start = False
                    self.call_service_safely(self.set_roslaunch_srv_name, SetLaunch, req)
                    self.get_logger().info(f"Requested stop for launch file: {launch.full_path}")
                    launch.pending = 0
                    db.session.commit()
            else:
                if RosActiveLaunchList.query.count() > 0:
                    db.session.query(RosActiveLaunchList).delete()
                    db.session.commit()
                    self.get_logger().info("Cleared all active launches from DB as service returned empty list or failed.")


def main(args=None):
    rclpy.init(args=args)
    gui_ros_node = None
    executor = None
    main_loop_thread = None
    try:
        gui_ros_node = GuiRos()
        executor = MultiThreadedExecutor()
        executor.add_node(gui_ros_node)

        main_loop_thread = threading.Thread(target=gui_ros_node.run)
        main_loop_thread.daemon = True
        main_loop_thread.start()

        executor.spin()
    except KeyboardInterrupt:
        if gui_ros_node:
            gui_ros_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if executor and gui_ros_node:
            executor.shutdown()
            gui_ros_node.destroy_node()
        rclpy.shutdown()
        if main_loop_thread:
            main_loop_thread.join()

if __name__ == "__main__":
    main()