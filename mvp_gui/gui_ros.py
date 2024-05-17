import time
import numpy as np
import threading
import rospy
import message_filters
from datetime import datetime
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from mvp_msgs.msg import Power
from tf.transformations import euler_from_quaternion
from mvp_gui import *
import yaml
from mvp_msgs.srv import GetStateRequest, GetState, ChangeStateRequest, ChangeState, GetWaypoints, GetWaypointsRequest
from std_srvs.srv import Empty, Trigger
from std_msgs.msg import Int16


class gui_ros():
    def __init__(self):
        # self.node_name = rospy.get_name()

        self.helm_state = 'start'
        self.helm_connected_states = []
        # ros parameters
        self.get_params()

        # ros subscribers and publishers
        self.setup_ros()

        
        # Main while loop.
        while not rospy.is_shutdown():
            self.get_state()
            self.change_state()
            self.change_controller_state()
            self.get_controller_state()
            self.get_waypoints()
            rospy.sleep(1.0)
    

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


        # self.power_items_source= rospy.get_param('power_items', ['power_manager/jetson',
        #                                                          'power_manager/24v',
        #                                                          'power_manager/lumen'])

    
    def setup_ros(self):
        self.poses_sub = message_filters.Subscriber(self.poses_source, Odometry)
        self.geo_pose_sub = message_filters.Subscriber(self.geo_pose_source, GeoPoseStamped)

        self.vitals_sub = message_filters.Subscriber(self.vitals_source, Power)


        self.ts = message_filters.ApproximateTimeSynchronizer([self.poses_sub, self.geo_pose_sub], 10, 0.1)

        self.ts.registerCallback(self.callback)

        self.geo_wpt_pub = rospy.Publisher("helm/path3d/update_geo_points", GeoPath, queue_size=10)
        self.geo_wpt_msg = GeoPath()

    def shutdown_node():    
        rospy.loginfo("Shutting down subscriber!")
        rospy.signal_shutdown("Shutting down subscriber!")

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


    ##state info
    def get_state(self):
        with app.app_context():
            rospy.wait_for_service(self.get_state_srv)
            try:
                service_client_get_state = rospy.ServiceProxy(self.get_state_srv, GetState)
                request = GetStateRequest("")
                response = service_client_get_state(request)
                # state = HelmStates.query.all()
                db.session.query(HelmStates).delete()

                state = HelmStates(id=0,name = str(response.state.name))
                self.helm_state = response.state.name
            # print(helmstate)
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
                    request = ChangeStateRequest(change_state.value)
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
        waypoints = Waypoints.query.order_by(Waypoints.id).all()
        count = 0
        for entry in waypoints:
            gpose = GeoPoseStamped()
            gpose.header.seq = count
            gpose.pose.position.latitude = entry.lat
            gpose.pose.position.longitude = entry.lon
            gpose.pose.position.altitude =  entry.z
            self.geo_wpt_msg.poses.append(gpose)
            count = count +1

        self.geo_wpt_pub.publish(self.geo_wpt_msg)



def shutdown_node():    
    rospy.loginfo("Shutting down subscriber!")
    rospy.signal_shutdown("Shutting down subscriber!")


def gui_ros_start():  
    rospy.init_node('mvp_gui_node', disable_signals=True)
    node = gui_ros()
    return node

global_file_name = './config/gui_config.yaml'


pose_t = threading.Thread(target = gui_ros_start)
pose_t.daemon = True
pose_t.start()

# except rospy.ROSInterruptException: pass
rospy.on_shutdown(shutdown_node)

