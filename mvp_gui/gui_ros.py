import numpy as np
import rospy
import rosservice
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
from mvp_roslaunch_manager.srv import GetLaunch, SetLaunch, GetLaunchRequest, SetLaunchRequest
from std_srvs.srv import Empty, Trigger, SetBool, SetBoolRequest
from std_msgs.msg import Int16
import time
import threading

class gui_ros():
    def __init__(self):
        self.helm_state = 'start'
        self.helm_connected_states = []
        # ros parameters
        self.get_params()
        # ros subscribers and publishers
        self.setup_ros()
    
    def get_params(self):
        dataset_config = yaml.safe_load(open(global_file_name, 'r'))

        # parse yaml file
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

        self.set_roslaunch_srv = self.name_space + dataset_config['set_launch']
        self.get_roslaunch_srv = self.name_space + dataset_config['get_launch']

        self.lumen_control_topic = self.name_space + dataset_config['lumen_control_topic']
        self.geo_pose_secondary_topic = self.name_space + dataset_config['geo_pose_secondary_source']

        self.pose_decay_time = dataset_config['pose_decay_time']

        self.current_lumen = 1.0

    def vital_callback(self, msg):
        with app.app_context():
            vital = Vitals.query.first()
            vital.id = 1
            vital.name = self.name_space
            vital.voltage = msg.data[0]
            vital.current = msg.data[1]
            db.session.commit() 
    
    def setpoint_callback(self, msg):
        with app.app_context():
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

    def setup_ros(self):
        self.poses_sub = message_filters.Subscriber(self.poses_source, Odometry)
        self.geo_pose_sub = message_filters.Subscriber(self.geo_pose_source, GeoPoseStamped)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.poses_sub, self.geo_pose_sub], 10, 0.1)

        self.setpoint_sub = rospy.Subscriber(self.pose_setpoint_source, ControlProcess, self.setpoint_callback)
        self.value_sub = rospy.Subscriber(self.pose_value_source, ControlProcess, self.value_callback)

        self.vitals_sub = rospy.Subscriber(self.vitals_source, Float32MultiArray, self.vital_callback)
        self.lumen_pub = rospy.Publisher(self.lumen_control_topic, Float64, queue_size=10)

        self.ts.registerCallback(self.pose_callback)
    
        self.geo_pose_secondary_sub = rospy.Subscriber(self.geo_pose_secondary_topic, GeoPoseStamped, self.geo_pose_secondary_callback)

        self.action_list = ['change_state', 'controller_state', 'publish_waypoints', 'get_topics', 'rosnode_cleanup', 'set_power']
        
        with app.app_context():
            for action_item in self.action_list:
                action_item = action_item
                action = RosActions.query.filter_by(action=action_item).first()
                action.pending = 0
                db.session.commit()

            lumen_item = LedItems.query.first()
            lumen_item.status = 1.0
            self.current_lumen = float(lumen_item.status) # used for publishing when there is a change
            db.session.commit()

    def run(self):
        try:
            # Set the loop rate
            while not rospy.is_shutdown():
                # Execute all the monitoring functions in sequence
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
                    rospy.logwarn(f"Error in main loop: {e}")
                
                # Sleep at the specified rate
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            rospy.loginfo("Received keyboard interrupt, shutting down...")
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS interrupt received")

    def service_exists(self, service_name):
        """
        Check if a ROS service exists using the rosservice library.
        
        Args:
            service_name (str): The name of the service to check
            
        Returns:
            bool: True if the service exists, False otherwise
        """
        try:
            # Get list of all available services
            service_list = rosservice.get_service_list()
            # Check if the requested service is in the list
            return service_name in service_list
        except Exception as e:
            rospy.logerr(f"Error checking if service {service_name} exists: {e}")
            return False

    # def call_service_safely(self, service_name, service_type, request=None, timeout=3.0):
    #     """
    #     A helper method to safely call ROS services with proper error handling.
    #     Args:
    #         service_name (str): The name of the service to call
    #         service_type: The service type/class
    #         request: The request object to send (optional)
    #         timeout (float): Timeout in seconds (optional)
    #     Returns:
    #         Response object if successful, None if failed
    #     """
    #     try:
    #         # Check if the service exists
    #         if not self.service_exists(service_name):
    #             rospy.logwarn(f"Service {service_name} does not exist")
    #             return None
                
    #         # Wait for service with timeout
    #         try:
    #             rospy.wait_for_service(service_name, timeout=timeout)
    #         except rospy.ROSException as e:
    #             rospy.logwarn(f"Service {service_name} timed out: {e}")
    #             return None
                
    #         # Create proxy and call service
    #         service_proxy = rospy.ServiceProxy(service_name, service_type)
        
    #         # Call with or without request
    #         if request is not None:
    #             return service_proxy(request)
    #         else:
    #             return service_proxy()

    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call to {service_name} failed: {e} !!!")
    #     except rospy.ROSInterruptException:
    #         rospy.loginfo("ROS interrupt received !!!")
    #     except Exception as e:
    #         rospy.logerr(f"Unexpected error calling {service_name}: {e} !!!")
    #     return None

    def call_service_safely(self, service_name, service_type, request=None, timeout=2.0):
        """
        A helper method to safely call ROS services with proper error handling using threading.

        Args:
            service_name (str): The name of the service to call
            service_type: The service type/class
            request: The request object to send (optional)
            timeout (float): Timeout in seconds (optional)

        Returns:
            Response object if successful, None if failed or timed out.
        """
        # Check if the service exists
        if not self.service_exists(service_name):
            rospy.logwarn(f"Service {service_name} does not exist")
            return None

        # Try waiting for the service with a timeout
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
        except rospy.ROSException as e:
            rospy.logwarn(f"Service {service_name} timed out: {e}")
            return None

        response_container = {'response': None}  # Shared variable for response
        done_event = threading.Event()  # Event to track completion

        def service_call():
            """ Calls the ROS service and stores the response. """
            try:
                service_proxy = rospy.ServiceProxy(service_name, service_type)
                if request is not None:
                    response_container['response'] = service_proxy(request)
                else:
                    response_container['response'] = service_proxy()
                done_event.set()  # Mark as done
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call to {service_name} failed: {e}")
                done_event.set()  # Ensure thread doesn't hang
            except Exception as e:
                rospy.logerr(f"Unexpected error calling {service_name}: {e}")
                done_event.set()  # Ensure thread doesn't hang

        # Create and start a thread for the service call
        service_thread = threading.Thread(target=service_call)
        service_thread.start()

        # Wait for completion or timeout
        service_thread.join(timeout=timeout)

        if not done_event.is_set():
            rospy.logerr(f"Service {service_name} timed out after {timeout} seconds")
            return None  # Timeout occurred

        return response_container['response']  # Return actual response


    # Ros Services
    def get_state(self):
        with app.app_context():
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
                for count, state_name in enumerate(response.state.transitions):
                    state = HelmStates(id=count+1, name=state_name)
                    db.session.add(state)
                db.session.commit()
            else:
                rospy.logwarn("Failed to get state information")

    def change_state(self):
        with app.app_context():
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
                    rospy.loginfo(f"Helm State changed to {change_state_action.value}")
                else:
                    rospy.logwarn(f"Failed to change Helm State to {change_state_action.value}")

    def get_controller_state(self):
        with app.app_context():
            # Use the helper method for service call
            response = self.call_service_safely(
                self.controller_state_srv, 
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
                    rospy.loginfo(f"Controller State changed to {controller_state.value}")
                else:
                    rospy.logwarn(f"Failed to change Controller State to {controller_state.value}")


    def get_power_port(self):
        with app.app_context():
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

    def set_power_port(self):
        with app.app_context():
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
            lumen_item = LedItems.query.first()
            if float(lumen_item.status) != self.current_lumen:
                self.current_lumen = float(lumen_item.status)
                for i in range(3):
                    lumen_ms = Float64()
                    lumen_ms.data = float(lumen_item.status)
                    self.lumen_pub.publish(lumen_ms)

    def get_waypoints(self):
        with app.app_context():
            response = self.call_service_safely(
                self.get_waypoint_srv, 
                GetWaypoints,
                GetWaypointsRequest(Int16(0))
            )
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
            else:
                rospy.logwarn("Failed to get waypoint information")

    def publish_waypoints(self):
        with app.app_context():
            pub_wpt_action = RosActions.query.filter_by(action='publish_waypoints').first()
            if pub_wpt_action.pending == 1:
                waypoints = Waypoints.query.order_by(Waypoints.id).all()
                geo_wpt = SendWaypointsRequest()
                geo_wpt.type = 'geopath'
                for count, entry in enumerate(waypoints):
                    wpt = Waypoint()
                    wpt.header.seq = count
                    wpt.ll_wpt.latitude = entry.lat
                    wpt.ll_wpt.longitude = entry.lon
                    wpt.ll_wpt.altitude = entry.alt
                    geo_wpt.wpt.append(wpt)

                response = self.call_service_safely(
                    self.pub_waypoint_srv,
                    SendWaypoints,
                    geo_wpt
                )
                if response:
                    # Log waypoint details
                    waypoint_details = [
                        f"{wpt.header.seq}: [{wpt.ll_wpt.latitude}, {wpt.ll_wpt.longitude}, {wpt.ll_wpt.altitude}]"
                        for wpt in geo_wpt.wpt
                    ]
                    rospy.loginfo(f"Published Waypoints: {', '.join(waypoint_details)}")
                    # rospy.loginfo(f"Published Waypoints")
                    pub_wpt_action.pending = 0
                    db.session.commit()
                else:
                    rospy.logwarn(f"Failed to publish waypoints")

    # def publish_waypoints(self):
    #     with app.app_context():
    #         pub_wpt_action = RosActions.query.filter_by(action='publish_waypoints').first()
    #         if pub_wpt_action.pending == 1:
    #             waypoints = Waypoints.query.order_by(Waypoints.id).all()
    #             geo_wpt =  SendWaypointsRequest()
    #             geo_wpt.type = 'geopath'
    #             count = 0
    #             for entry in waypoints:
    #                 wpt = Waypoint()
    #                 wpt.header.seq = count
    #                 wpt.ll_wpt.latitude = entry.lat
    #                 wpt.ll_wpt.longitude = entry.lon
    #                 wpt.ll_wpt.altitude =  entry.alt
    #                 count = count +1
    #                 geo_wpt.wpt.append(wpt)
    #             try:
    #                 service_client_pub_waypoint_srv= rospy.ServiceProxy(self.pub_waypoint_srv, SendWaypoints)
    #                 response = service_client_pub_waypoint_srv(geo_wpt)
    #                 pub_wpt_action.pending = 0
    #                 db.session.commit()
    #             except:
    #                 print("Publish Waypoints Service Timeout")

    def log_poses(self):
        with app.app_context():
            new_pose = db.session.query(Poses).first()
            #increase id by 1
            db.session.query(PoseHistory).update({PoseHistory.id: PoseHistory.id + 1})
            db.session.query(PoseHistory).filter(PoseHistory.id > self.pose_decay_time).delete()

            # Create a new instance of PoseHistory with the values from new_pose
            new_pose_history = PoseHistory()
            new_pose_history.id = 1
            new_pose_history.frame_id = new_pose.frame_id
            new_pose_history.child_frame_id = new_pose.child_frame_id
            new_pose_history.roll = new_pose.roll
            new_pose_history.pitch = new_pose.pitch
            new_pose_history.yaw = new_pose.yaw
            new_pose_history.x = new_pose.x
            new_pose_history.y = new_pose.y
            new_pose_history.z = new_pose.z
            new_pose_history.u = new_pose.u
            new_pose_history.v = new_pose.v
            new_pose_history.w = new_pose.w
            new_pose_history.p = new_pose.p
            new_pose_history.q = new_pose.q
            new_pose_history.r = new_pose.r
            new_pose_history.lat = new_pose.lat
            new_pose_history.lon = new_pose.lon

            # Add the new entry to the session and commit
            db.session.add(new_pose_history)
            db.session.commit()

    def geo_pose_secondary_callback(self, msg):
        with app.app_context():
            geo_pose_secondary = PoseSecondary.query.first()
            if geo_pose_secondary == None:
                geo_pose_secondary = PoseSecondary()
                geo_pose_secondary.id = 1
                geo_pose_secondary.lat = 0.0
                geo_pose_secondary.lon = 0.0
                geo_pose_secondary.z = 0.0
            geo_pose_secondary.id = 1
            geo_pose_secondary.lat = msg.pose.position.latitude
            geo_pose_secondary.lon = msg.pose.position.longitude
            geo_pose_secondary.z = msg.pose.position.altitude
            db.session.add(geo_pose_secondary)

            #increase id by 1
            db.session.query(PoseHistorySecondary).update({PoseHistorySecondary.id: PoseHistorySecondary.id + 1})
            db.session.query(PoseHistorySecondary).filter(PoseHistorySecondary.id > self.pose_decay_time).delete()

            geo_pose_secondary_history = PoseHistorySecondary()
            geo_pose_secondary_history.id = 1
            geo_pose_secondary_history.lat = msg.pose.position.latitude
            geo_pose_secondary_history.lon = msg.pose.position.longitude
            geo_pose_secondary_history.z = msg.pose.position.altitude
            db.session.add(geo_pose_secondary_history)
            db.session.commit()
    
    def roslaunch_file(self):
        with app.app_context():
            launch_files_list = RosLaunchList.query.all()
            for launch_file in launch_files_list:
                if launch_file.pending == 1:
                    launch_file_dir = launch_file.folder_dir.replace('~', '..', 1)
                    launch_file_full_path = os.path.join(launch_file_dir, launch_file.name)

                    # call rosservice
                    response = self.call_service_safely(
                        self.set_roslaunch_srv,
                        SetLaunch,
                        SetLaunchRequest(launch_file_full_path, True)
                    )
                    if response:
                        print("sucessfully launched: {}".format(launch_file.name))
                        launch_file.pending = 0
                        db.session.commit()
                    else:
                        print("failed to launch: {}".format(launch_file.name))
                    time.sleep(1.0)
    
    def activate_roslaunch_list(self):
        with app.app_context():
            # Call the ROS service to get the current launch list
            response = self.call_service_safely(
                self.get_roslaunch_srv,
                GetLaunch,
                GetLaunchRequest()
            )
            if len(response.list) != 0:
                active_launches_db_list = RosActiveLaunchList.query.all()
                active_launches_db_list_full_path = {launch.full_path: launch for launch in RosActiveLaunchList.query.all()}
                # response.list now contains only full_path values, so we create a set for easy lookup
                new_launch_set = set(response.list)
                current_launches_len = len(active_launches_db_list_full_path)
                
                # remove records that are no longer in the new list
                if current_launches_len != 0:
                    for active_launches_full_path in active_launches_db_list_full_path:
                        if active_launches_full_path not in new_launch_set:
                            db.session.query(RosActiveLaunchList).filter(RosActiveLaunchList.full_path == active_launches_full_path).delete()
                            # Reindex remaining entries
                            remaining_entries = db.session.query(RosActiveLaunchList).order_by(RosActiveLaunchList.id).all()
                            for index, entry in enumerate(remaining_entries):
                                entry.id = index
                            print(f"Deleted launch file: {active_launches_full_path}")
        
                current_launches_len = len(RosActiveLaunchList.query.all())
                # Add new launch files that don't exist in the current records
                count = 0
                for full_path in new_launch_set:
                    if full_path not in active_launches_db_list_full_path:
                        new_record = RosActiveLaunchList(id=count + current_launches_len, full_path=full_path, pending=0)
                        db.session.add(new_record)
                        count += 1
                        print(f"Added new launch file: {full_path}")
                
                # Commit all changes once done
                db.session.commit()

                # Now process any records that have pending set to 1.
                pending_launches = RosActiveLaunchList.query.filter_by(pending=1).all()
                for launch in pending_launches:
                    # Call the ROS service for the pending launch
                    response = self.call_service_safely(
                        self.set_roslaunch_srv,
                        SetLaunch,
                        SetLaunchRequest(launch.full_path, False)
                    )
                    print(f"SetLaunch for: {launch.full_path} to False")
            else:
                db.session.query(RosActiveLaunchList).delete()
                db.session.commit()
            

if __name__ == "__main__":
    try:
        rospy.init_node('mvp_gui_node')
        gui_ros_node = gui_ros()
        gui_ros_node.run()
    except rospy.exceptions.ROSException as e:
        print("Exiting mvp gui node!")
