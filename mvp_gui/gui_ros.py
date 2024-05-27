import numpy as np
import rospy
import message_filters
import signal
from datetime import datetime
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from mvp_msgs.msg import Power, Waypoint
from tf.transformations import euler_from_quaternion
from mvp_gui import *
import yaml
from mvp_msgs.srv import GetStateRequest, GetState, ChangeStateRequest, ChangeState, GetWaypoints, GetWaypointsRequest, SendWaypoints, SendWaypointsRequest
from std_srvs.srv import Empty, Trigger
from std_msgs.msg import Int16


class gui_ros():
    def __init__(self):
        self.helm_state = 'start'
        self.helm_connected_states = []
        # ros parameters
        self.get_params()
        # ros subscribers and publishers
        self.setup_ros()
        # Main while loop.
        self.main_loop()

    def main_loop(self):
        while not rospy.is_shutdown():
            self.log_poses()
            rospy.sleep(0.1)
            self.get_state()
            rospy.sleep(0.1)
            self.change_state()
            rospy.sleep(0.1)
            self.change_controller_state()
            rospy.sleep(0.1)
            self.get_controller_state()
            rospy.sleep(0.1)
            self.get_waypoints()
            rospy.sleep(0.1)
            self.publish_wpt()
            rospy.sleep(0.5)
    

    def get_params(self):
        
        dataset_config = yaml.safe_load(open(global_file_name, 'r'))

        # make lookup table for mapping
        self.name_space = '/' + dataset_config['name_space'] + '/'
        self.poses_source = self.name_space + dataset_config['poses_source']

        self.geo_pose_source = self.name_space + dataset_config['geo_pose_source']
        self.vitals_source = self.name_space + dataset_config['vitals_source']

        self.get_state_srv  = self.name_space + dataset_config['get_state_service']
        self.change_state_srv  = self.name_space + dataset_config['change_state_service']

        self.controller_srv  = self.name_space + dataset_config['controller_service']
        self.controller_state_srv  = self.name_space + dataset_config['controller_state_service']

        self.get_waypoint_srv  = self.name_space + dataset_config['get_waypoints_service']
        self.pub_waypoint_srv = self.name_space + dataset_config['pub_waypoints_service']

        # self.power_items_source= rospy.get_param('power_items', ['power_manager/jetson',
        #                                                          'power_manager/24v',
        #                                                          'power_manager/lumen'])

    
    def setup_ros(self):
        
        self.poses_sub = message_filters.Subscriber(self.poses_source, Odometry)
        self.geo_pose_sub = message_filters.Subscriber(self.geo_pose_source, GeoPoseStamped)

        self.vitals_sub = message_filters.Subscriber(self.vitals_source, Power)


        self.ts = message_filters.ApproximateTimeSynchronizer([self.poses_sub, self.geo_pose_sub], 10, 0.1)

        self.ts.registerCallback(self.callback)
        with app.app_context():
            action = RosActions.query.filter_by(action='change_state').first()
            action.pending = 0
            db.session.commit()

            action = RosActions.query.filter_by(action='controller_state').first()
            action.pending = 0
            db.session.commit()
        
            action = RosActions.query.filter_by(action='publish_waypoints').first()
            action.pending = 0
            db.session.commit()

    #obtain pose and power information and store in the database 
    def callback(self, poses_sub, geo_pose_sub):
        quad = [poses_sub.pose.pose.orientation.x, 
                poses_sub.pose.pose.orientation.y, 
                poses_sub.pose.pose.orientation.z, 
                poses_sub.pose.pose.orientation.w]
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
            poses.z = poses_sub.pose.pose.position.z
            poses.u = poses_sub.twist.twist.linear.x
            poses.v = poses_sub.twist.twist.linear.y
            poses.w = poses_sub.twist.twist.linear.z
            poses.p = poses_sub.twist.twist.angular.x
            poses.q = poses_sub.twist.twist.angular.y
            poses.r = poses_sub.twist.twist.angular.z
            poses.lat = geo_pose_sub.pose.position.latitude
            poses.lon = geo_pose_sub.pose.position.longitude

            # print(poses.roll)

            # vitals = Vitals.query.first()
            # vitals.voltage = self.vitals_sub.voltage
            # vitals.current = self.vitals_sub.current

            db.session.commit() 
            
    def log_poses(self):
        decay = 20
        with app.app_context():
            new_pose = db.session.query(Poses).first()

            num_entries = db.session.query(PoseHistory).count()
            # print(num_entries)
            #increase id by 1
            db.session.query(PoseHistory).update({PoseHistory.id: PoseHistory.id + 1})
            db.session.query(PoseHistory).filter(PoseHistory.id > decay).delete()
            db.session.commit()

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

    ##state info
    def get_state(self):
        with app.app_context():
            rospy.wait_for_service(self.get_state_srv)
            try:
                service_client_get_state = rospy.ServiceProxy(self.get_state_srv, GetState)
                request = GetStateRequest("")
                response = service_client_get_state(request)
                db.session.query(HelmStates).delete()
                db.session.commit()
                state = HelmStates(id=0,name = str(response.state.name))
                self.helm_state = response.state.name
                db.session.add(state)
                db.session.commit()
                count = 1
                for state_name in response.state.transitions:
                    state = HelmStates(id=count,name = state_name)
                    db.session.add(state)
                    count = count +1
                db.session.commit()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

     ##state info
    def change_state(self):
        with app.app_context():
            change_state = RosActions.query.filter_by(action='change_state').first()
            if change_state.pending == 1:
                rospy.wait_for_service(self.get_state_srv)
                try:
                    service_client_change_state = rospy.ServiceProxy(self.change_state_srv, ChangeState)
                    print(rospy.get_name())
                    request = ChangeStateRequest(change_state.value, rospy.get_name())
                    response = service_client_change_state(request)
                    change_state.pending = 0
                    db.session.commit()
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
            
    ##change controller state action
    def get_controller_state(self):
        with app.app_context():
            rospy.wait_for_service(self.controller_state_srv)
            try:
                service_client_get_controller_state = rospy.ServiceProxy(self.controller_state_srv, Trigger)
                response = service_client_get_controller_state()

                controller_state = ControllerState.query.first()
                controller_state.state = response.message
                # print(response.message)
                db.session.commit()
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)


    def change_controller_state(self):
        with app.app_context():
            controller_state = RosActions.query.filter_by(action='controller_state').first()
            if controller_state.pending == 1:
                rospy.wait_for_service(self.controller_srv + '/' + controller_state.value)
                try:
                    service_client_change_controller_state = rospy.ServiceProxy(self.controller_srv + '/' + controller_state.value, Empty)
                    service_client_change_controller_state()
                    controller_state.pending = 0
                    db.session.commit()
                    print(self.controller_srv + '/' + controller_state.value)

                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
    
    ##get current waypoints
    def get_waypoints(self):
        with app.app_context():
            rospy.wait_for_service(self.get_waypoint_srv)
            try:
                service_client_get_waypoint_srv= rospy.ServiceProxy(self.get_waypoint_srv, GetWaypoints)
                request = GetWaypointsRequest(Int16(0)) ##get all waypoints
                response = service_client_get_waypoint_srv(request)
   
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
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    ##publishing waypoints 
    def publish_wpt(self):
        with app.app_context():
            # print("publishing waypoints")
            pub_wpt_action = RosActions.query.filter_by(action='publish_waypoints').first()

            if pub_wpt_action.pending == 1:
                waypoints = Waypoints.query.order_by(Waypoints.id).all()
                geo_wpt =  SendWaypointsRequest()
                geo_wpt.type = 'geopath'
                count = 0
                for entry in waypoints:
                    wpt = Waypoint()
                    wpt.header.seq = count
                    wpt.ll_wpt.latitude = entry.lat
                    wpt.ll_wpt.longitude = entry.lon
                    wpt.ll_wpt.altitude =  entry.alt
                    count = count +1
                    geo_wpt.wpt.append(wpt)
                # print(self.geo_wpt)
                # print("sending waypoints number = ", count)
                try:
                    service_client_pub_waypoint_srv= rospy.ServiceProxy(self.pub_waypoint_srv, SendWaypoints)
                    response = service_client_pub_waypoint_srv(geo_wpt)
                    # print("wpt_function:" , response)
                    pub_wpt_action.pending = 0
                    db.session.commit()
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

    
def gui_ros_start():  
    try:
        rospy.init_node('mvp_gui_node', disable_signals=True)
        gui_ros()
    except rospy.exceptions.ROSException as e:
        pass

if __name__ == "__main__":
    gui_ros_start()



