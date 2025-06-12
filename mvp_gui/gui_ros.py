import numpy as np
import rclpy
from rclpy.node import Node
import message_filters
from datetime import datetime
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from mvp_msgs.msg import Power, Waypoint, ControlProcess, HelmState
from std_msgs.msg import Float64, Float32MultiArray, Int16, Bool
from tf_transformations import euler_from_quaternion
from mvp_gui import *
import yaml
from mvp_msgs.srv import SetString, SendWaypoints
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
        self.ros_source = dataset_config['ros_source_base']
        self.pose_decay_time = dataset_config['pose_decay_time']
        #----------------------------
        # Topics
        #----------------------------
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

        self.get_helm_state = self.topic_ns + dataset_config['helm_state_get']
        self.get_controller_state = self.topic_ns + dataset_config['controller_state_get']

        #----------------------------
        # Services
        #----------------------------
        service_command = self.ros_source + 'ros2 service list | grep remote'
        service_result = subprocess.run(
            ['bash', '-c', topic_command],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
            timeout=SUBPROCESS_TIMEOUT
        )
        service_result_split =  service_result.stdout.splitlines()[0].split('/')
        self.service_ns = self.name_space + '/' + service_result_split[2] + '/' + self.remote_id + '/'

        self.set_controller_state = self.service_ns + dataset_config['controller_state_set']
        self.set_helm_state = self.service_ns + dataset_config['helm_state_set']

        self.pub_waypoints_service = self.service_ns + dataset_config['pub_waypoints_service']

    def pose_callback(self, poses_sub, geo_pose_sub):
        quad = [geo_pose_sub.pose.orientation.x, 
                geo_pose_sub.pose.orientation.y, 
                geo_pose_sub.pose.orientation.z, 
                geo_pose_sub.pose.orientation.w]
        euler_angles = euler_from_quaternion(quad)
        with app.app_context():
            # pose
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

    def helm_state_callback(self, msg):
        with app.app_context():
            db.session.query(HelmStates).delete()                
            helm_state = HelmStates(id=0, name=str(msg.name))
            db.session.add(helm_state)                
            for count, state_name in enumerate(msg.transitions):
                helm_state = HelmStates(id=count+1, name=state_name)
                db.session.add(helm_state)
            db.session.commit()

    def controller_state_callback(self, msg):
        with app.app_context():
            controller_state = ControllerState.query.first()
            if controller_state is None:
                controller_state = ControllerState(id=1)
                db.session.add(controller_state)
            controller_state.state = msg.data
            db.session.commit()

    def setup_ros(self):
        # Synchronized Subscribers for pose data (they have headers)
        self.poses_sub = message_filters.Subscriber(self, Odometry, self.poses_source)
        self.geo_pose_sub = message_filters.Subscriber(self, GeoPoseStamped, self.geo_pose_source)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.poses_sub, self.geo_pose_sub], 10, 0.1)
        self.ts.registerCallback(self.pose_callback)

        # Individual subscribers for state data (no headers)
        self.helm_state_sub = self.create_subscription(
            HelmState, 
            self.get_helm_state, 
            self.helm_state_callback, 
            10)
        self.controller_state_sub = self.create_subscription(
            Bool, 
            self.get_controller_state, 
            self.controller_state_callback, 
            10)

        # Use a reentrant callback group to allow service calls from within callbacks/timers
        self.callback_group = ReentrantCallbackGroup()

        self.action_list = ['change_state', 'controller_state', 'publish_waypoints']
        
        with app.app_context():
            for action_item in self.action_list:
                action = RosActions.query.filter_by(action=action_item).first()
                if action:
                    action.pending = 0
                    db.session.commit()


    # This main loop will be run in a separate thread
    def run(self):
        while rclpy.ok():
            try:
                self.change_helm_state()
                self.change_controller_state()
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
    
    def change_helm_state(self):
        with app.app_context():
            change_state_action = RosActions.query.filter_by(action='change_state', pending=1).first()
            if change_state_action:
                req = SetString.Request()
                req.data = change_state_action.value
                
                # The service name is self.set_helm_state, the type is SetString
                response = self.call_service_safely(self.set_helm_state, SetString, req)

                if response is not None:
                    if response.success:
                        self.get_logger().info(f"Helm State change to '{req.data}' successful. Message: '{response.message}'")
                    else:
                        self.get_logger().warn(f"Helm State change to '{req.data}' failed. Message: '{response.message}'")
                    
                    # Mark action as processed to avoid retrying
                    change_state_action.pending = 0
                    db.session.commit()

    def change_controller_state(self):
        with app.app_context():
            controller_state_action = RosActions.query.filter_by(action='controller_state', pending=1).first()
            if controller_state_action:
                service_name = self.set_controller_state

                # The service type is std_srvs.srv.SetBool, not std_msgs.msg.Bool.
                req = SetBool.Request()

                # Convert the value from the database to a boolean
                try:
                    # Handles string 'true'/'false' or '1'/'0'
                    val_str = str(controller_state_action.value).lower()
                    if val_str in ['true', '1']:
                        req.data = True
                    elif val_str in ['false', '0']:
                        req.data = False
                    else:
                        # If the value is invalid, log an error and mark as processed to avoid retrying
                        self.get_logger().error(f"Invalid boolean value for controller state: '{controller_state_action.value}'.")
                        controller_state_action.pending = 0
                        db.session.commit()
                        return
                except Exception as e:
                    self.get_logger().error(f"Could not convert '{controller_state_action.value}' to bool: {e}. Action marked as processed.")
                    controller_state_action.pending = 0
                    db.session.commit()
                    return

                # Call the service with the correct service type (SetBool) and the populated request
                response = self.call_service_safely(service_name, SetBool, req)
                
                if response is not None:
                    # The SetBool service response has 'success' and 'message' fields.
                    if response.success:
                        self.get_logger().info(f"Controller state change to {req.data} successful. Message: '{response.message}'")
                    else:
                        self.get_logger().warn(f"Controller state change to {req.data} failed. Message: '{response.message}'")
                    
                    # Mark action as processed to avoid retrying, even if it failed.
                    controller_state_action.pending = 0
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
                    wpt.ll_wpt.latitude = float(entry.lat)
                    wpt.ll_wpt.longitude = float(entry.lon)
                    wpt.ll_wpt.altitude = float(entry.alt)
                    req.wpt.append(wpt)

                response = self.call_service_safely(self.pub_waypoints_service, SendWaypoints, req)
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