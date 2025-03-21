import numpy as np
import rospy
import message_filters
from datetime import datetime
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from mvp_msgs.msg import Power, Waypoint, ControlProcess
from std_msgs.msg import Float64, Float32MultiArray
from tf.transformations import euler_from_quaternion
from mvp_gui import *
import yaml
from mvp_msgs.srv import GetStateRequest, GetState, ChangeStateRequest, ChangeState, GetWaypoints, GetWaypointsRequest, SendWaypoints, SendWaypointsRequest
from std_srvs.srv import Empty, Trigger, SetBool, SetBoolRequest
from std_msgs.msg import Int16
import threading
import time

class callback_functions():
    def vital_callback(self, msg):
        with app.app_context():
            with self.db_lock:
                vital = Vitals.query.first()
                vital.id = 1
                vital.name = self.name_space
                vital.voltage = msg.data[0]
                vital.current = msg.data[1]
                db.session.commit() 
    
    def setpoint_callback(self, msg):
        with app.app_context():
            with self.db_lock:
                setpoint_pose = PoseSetpoint.query.first()
                setpoint_pose.id = 1
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
            with self.db_lock:
                value_pose = PoseValue.query.first()
                value_pose.id = 1
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
            with self.db_lock:
                poses = Poses.query.first()
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
                if geo_pose_sub.pose.position.latitude != None and geo_pose_sub.pose.position.longitude != None:
                    poses.lat = geo_pose_sub.pose.position.latitude
                    poses.lon = geo_pose_sub.pose.position.longitude
                else:
                    poses.lat = 0.0
                    poses.lon = 0.0
                db.session.commit() 

class gui_ros():
    def __init__(self):
        self.helm_state = 'start'
        self.helm_connected_states = []
        self.rate = rospy.Rate(1.0)
        self.sleep_t1
        # Thread control
        self.running = True
        self.threads = []
        # ros parameters
        self.get_params()
        # ros subscribers and publishers
        self.setup_ros()
        # Start threads instead of main loop
        self.start_threads()
    
    def get_params(self):
        dataset_config = yaml.safe_load(open(global_file_name, 'r'))

        # make lookup table for mapping
        self.name_space = '/' + dataset_config['name_space'] + '/'
        self.poses_source = self.name_space + dataset_config['poses_source']
        self.pose_setpoint_source = self.name_space + dataset_config['setpoint_source']
        self.pose_value_source = self.name_space + dataset_config['value_source']

        self.geo_pose_source = self.name_space + dataset_config['geo_pose_source']
        self.vitals_source = self.name_space + dataset_config['vitals_source']

        self.get_state_srv  = self.name_space + dataset_config['get_state_service']
        self.change_state_srv  = self.name_space + dataset_config['change_state_service']

        self.controller_srv  = self.name_space + dataset_config['controller_service']
        self.controller_state_srv  = self.name_space + dataset_config['controller_state_service']

        self.get_waypoint_srv  = self.name_space + dataset_config['get_waypoints_service']
        self.pub_waypoint_srv = self.name_space + dataset_config['pub_waypoints_service']
        
        self.get_power_port_srv = self.name_space + dataset_config['get_power_port_srv']
       
        self.lumen_control_topic = self.name_space + dataset_config['lumen_control_topic']
        self.geo_pose_secondary_topic = self.name_space + dataset_config['geo_pose_secondary_source']

        self.pose_decay_time = dataset_config['pose_decay_time']
        # Add a thread lock for database operations
        self.db_lock = threading.Lock()

    def setup_ros(self):
        self.poses_sub = message_filters.Subscriber(self.poses_source, Odometry)
        self.geo_pose_sub = message_filters.Subscriber(self.geo_pose_source, GeoPoseStamped)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.poses_sub, self.geo_pose_sub], 10, 0.1)

        self.setpoint_sub = rospy.Subscriber(self.pose_setpoint_source, ControlProcess, callback_functions.setpoint_callback)
        self.value_sub = rospy.Subscriber(self.pose_value_source, ControlProcess, callback_functions.value_callback)

        self.vitals_sub = rospy.Subscriber(self.vitals_source, Float32MultiArray, callback_functions.vital_callback)
        self.lumen_pub = rospy.Publisher(self.lumen_control_topic, Float64, queue_size=10)

        self.ts.registerCallback(callback_functions.pose_callback)
    
        self.geo_pose_secondary_sub = rospy.Subscriber(self.geo_pose_secondary_topic, GeoPoseStamped, self.geo_pose_secondary_callback)

        self.action_list = ['change_state', 'controller_state', 'publish_waypoints', 'get_topics', 'rosnode_cleanup', 'set_power']
        
        with app.app_context():
            with self.db_lock:  # Use a lock for thread safety
                for action_item in self.action_list:
                    action_item = action_item
                    action = RosActions.query.filter_by(action=action_item).first()
                    action.pending = 0
                    db.session.commit()

                lumen_item = LedItems.query.first()
                lumen_item.status = 1.0
                self.current_lumen = float(lumen_item.status) ##used for publishing when there is a change
                db.session.commit()

    def start_threads(self):
        """Start separate threads for each function"""
        functions = [
            self.power_port_monitor,
            self.lumen_monitor,
            self.state_monitor,
            self.controller_state_monitor,
            self.waypoints_monitor,
            self.pose_logger
        ]
        
        # Create and start threads
        for func in functions:
            thread = threading.Thread(target=func)
            thread.daemon = True  # Daemon threads exit when main thread exits
            self.threads.append(thread)
            thread.start()
        
        # Main thread can now do other work or just wait
        try:
            while not rospy.is_shutdown() and self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            # Wait for threads to finish
            for thread in self.threads:
                thread.join(timeout=1.0)

    # Thread functions with continuous loops
    def power_port_monitor(self):
        """Thread for monitoring and controlling power ports"""
        while not rospy.is_shutdown() and self.running:
            try:
                self.get_power_port()
                self.set_power_port()
                time.sleep(2)  # Adjust timing as needed
            except Exception as e:
                rospy.logwarn(f"Error in power port thread: {e}")
                time.sleep(2)  # Continue despite errors
    
    def lumen_monitor(self):
        """Thread for monitoring and setting lumen values"""
        while not rospy.is_shutdown() and self.running:
            try:
                self.set_lumen()
                time.sleep(0.5)  # Adjust timing as needed
            except Exception as e:
                rospy.logwarn(f"Error in lumen thread: {e}")
                time.sleep(2)
    
    def state_monitor(self):
        """Thread for state monitoring and changes"""
        while not rospy.is_shutdown() and self.running:
            try:
                self.get_state()
                self.change_state()
                time.sleep(1)  # Adjust timing as needed
            except Exception as e:
                rospy.logwarn(f"Error in state thread: {e}")
                time.sleep(2)
    
    def controller_state_monitor(self):
        """Thread for controller state monitoring and changes"""
        while not rospy.is_shutdown() and self.running:
            try:
                self.change_controller_state()
                self.get_controller_state()
                time.sleep(1)  # Adjust timing as needed
            except Exception as e:
                rospy.logwarn(f"Error in controller state thread: {e}")
                time.sleep(2)
    
    def waypoints_monitor(self):
        """Thread for waypoints monitoring and publishing"""
        while not rospy.is_shutdown() and self.running:
            try:
                self.get_waypoints()
                self.publish_waypoints()
                time.sleep(2)  # Adjust timing as needed
            except Exception as e:
                rospy.logwarn(f"Error in waypoints thread: {e}")
                time.sleep(2)
    
    def pose_logger(self):
        """Thread for logging poses"""
        while not rospy.is_shutdown() and self.running:
            try:
                self.log_poses()
                time.sleep(2)  # Adjust timing as needed
            except Exception as e:
                rospy.logwarn(f"Error in pose logger thread: {e}")
                time.sleep(2)


    def call_service_safely(self, service_name, service_type, request=None, timeout=0.1):
        """
        A helper method to safely call ROS services with proper error handling.
        
        Args:
            service_name (str): The name of the service to call
            service_type: The service type/class
            request: The request object to send (optional)
            timeout (float): Timeout in seconds (optional)
            
        Returns:
            Response object if successful, None if failed
        """
        try:
            # Non-blocking check if the service exists
            if not rospy.service_exists(service_name):
                rospy.logwarn(f"Service {service_name} does not exist")
                return None
                
            # Wait for service with timeout
            service_available = rospy.wait_for_service(service_name, timeout=timeout)
            if not service_available:
                rospy.logwarn(f"Service {service_name} timed out")
                return None
                
            # Create proxy and call service
            service_proxy = rospy.ServiceProxy(service_name, service_type)
            
            # Call with or without request
            if request is not None:
                return service_proxy(request)
            else:
                return service_proxy()
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:
            rospy.logwarn(f"ROS exception calling {service_name}: {e}")
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS interrupt received")
            self.running = False
        except Exception as e:
            rospy.logerr(f"Unexpected error calling {service_name}: {e}")
        
        return None
    
    # Ros Services
    def get_state(self):
        with app.app_context():
            with self.db_lock:
                # Use the helper method for service call
                response = self.call_service_safely(
                    self.get_state_srv, 
                    GetState,
                    GetStateRequest(""),
                )
                
                if response:
                    # Process successful response
                    db.session.query(HelmStates).delete()
                    db.session.commit()
                    
                    state = HelmStates(id=0, name=str(response.state.name))
                    self.helm_state = response.state.name
                    db.session.add(state)
                    db.session.commit()
                    
                    # Process transitions
                    count = 1
                    for state_name in response.state.transitions:
                        state = HelmStates(id=count, name=state_name)
                        db.session.add(state)
                        count += 1
                    db.session.commit()
                else:
                    rospy.logwarn("Failed to get state information")

    # Example of how to simplify the change_state method
    def change_state(self):
        with app.app_context():
            with self.db_lock:
                change_state_action = RosActions.query.filter_by(action='change_state').first()
                if change_state_action.pending == 1:
                    response = self.call_service_safely(
                        self.change_state_srv,
                        ChangeState,
                        ChangeStateRequest(change_state_action.value, rospy.get_name())
                    )
                    if response:
                        # Service call successful
                        change_state_action.pending = 0
                        db.session.commit()
                        rospy.loginfo(f"State changed to {change_state_action.value}")
                    else:
                        rospy.logwarn(f"Failed to change state to {change_state_action.value}")

    def get_controller_state(self):
        with app.app_context():
            with self.db_lock:
                # Use the helper method for service call
                response = self.call_service_safely(
                    self.controller_srv, 
                    Trigger
                )
                if response:
                    controller_state = ControllerState.query.first()
                    controller_state.state = response.message
                    db.session.commit()
                else:
                    rospy.logwarn("Failed to get controller state information")

    def change_controller_state(self):
        with app.app_context():
            with self.db_lock:
                controller_state = RosActions.query.filter_by(action='controller_state').first()
                if controller_state.pending == 1:
                    response = self.call_service_safely(
                        self.controller_srv + '/' + controller_state.value,
                        Empty
                    )
                    if response:
                        # Service call successful
                        controller_state.pending = 0
                        db.session.commit()
                        rospy.loginfo(f"State changed to {controller_state.value}")
                    else:
                        rospy.logwarn(f"Failed to change state to {controller_state.value}")


    def get_power_port(self):
        with app.app_context():
            with self.db_lock:
                response = self.call_service_safely(
                    self.get_power_port_srv, 
                    Trigger
                )
                if response:
                    db.session.query(PowerItems).delete()
                    power_list = response.message.splitlines()
                    for count, line in enumerate(power_list):
                        parts = line.split('=')
                        if len(parts) == 2:  # Ensure there are exactly two parts
                            name, status = parts
                            power_item = PowerItems(id=count, name=name, status=status)
                            db.session.add(power_item)
                    db.session.commit()
                else:
                    rospy.logwarn("Failed to get power port information")

    # Example of how to handle service calls with a conditional result
    def set_power_port(self):
        with app.app_context():
            with self.db_lock:
                set_power_action = RosActions.query.filter_by(action='set_power').first()
                
                if set_power_action.pending == 1:
                    try:
                        parts = set_power_action.value.split('=')
                        if len(parts) != 2:
                            rospy.logerr(f"Invalid power command format: {set_power_action.value}")
                            set_power_action.pending = 0
                            db.session.commit()
                            return
                            
                        srv_name, set_status = parts
                        service_path = self.name_space + srv_name
                        
                        # Create request based on boolean value
                        request = SetBoolRequest(set_status.lower() != "false")
                        
                        # Call service
                        response = self.call_service_safely(service_path, SetBool, request)
                        
                        if response:
                            if response.success:
                                rospy.loginfo(f"Set power port {srv_name} to {set_status}: {response.message}")
                            else:
                                rospy.logwarn(f"Service {srv_name} reported failure: {response.message}")
                                
                        # Mark as processed either way
                        set_power_action.pending = 0
                        db.session.commit()
                        
                    except Exception as e:
                        rospy.logerr(f"Error processing power command: {e}")
                        set_power_action.pending = 0
                        db.session.commit()

    def set_lumen(self):
        with app.app_context():
            with self.db_lock:
                lumen_item = LedItems.query.first()
                if float(lumen_item.status) != self.current_lumen:
                    self.current_lumen = float(lumen_item.status)
                    for i in range(3):
                        lumen_ms = Float64()
                        lumen_ms.data = float(lumen_item.status)
                        self.lumen_pub.publish(lumen_ms)

    def get_waypoints(self):
        with app.app_context():
            with self.db_lock:
                response = self.call_service_safely(
                    self.get_waypoint_srv, 
                    GetWaypoints
                )
                if response:
                    db.session.query(CurrentWaypoints).delete()
                    db.session.commit()
                    count = 0
                    for wpt in response.wpt:
                        p = CurrentWaypoints(id=count, 
                                                lat = wpt.ll_wpt.latitude, 
                                                lon = wpt.ll_wpt.longitude, 
                                                alt = wpt.ll_wpt.altitude)
                        db.session.add(p)
                        count = count +1
                    db.session.commit()
                else:
                    rospy.logwarn("Failed to get waypoint information")

    def publish_waypoints(self):
        with app.app_context():
            with self.db_lock:
                pub_wpt_action = RosActions.query.filter_by(action='publish_waypoints').first()
                if pub_wpt_action.pending == 1:
                    response = self.call_service_safely(
                        self.pub_waypoint_srv,
                        SendWaypoints
                    )
                    if response:
                        waypoints = Waypoints.query.order_by(Waypoints.id).all()
                        geo_wpt =  SendWaypointsRequest()
                        geo_wpt.type = 'geopath'
                        for count, entry in enumerate(waypoints):
                            wpt = Waypoint()
                            wpt.header.seq = count
                            wpt.ll_wpt.latitude = entry.lat
                            wpt.ll_wpt.longitude = entry.lon
                            wpt.ll_wpt.altitude =  entry.alt
                            geo_wpt.wpt.append(wpt)
                        pub_wpt_action.pending = 0
                        db.session.commit()
                    else:
                        rospy.logwarn(f"Failed to publish waypoints")
