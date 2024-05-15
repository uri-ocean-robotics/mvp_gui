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
        # while not rospy.is_shutdown():
            # rospy.sleep(0.1)
    

    def get_params(self):
        
        dataset_config = yaml.safe_load(open(global_file_name, 'r'))

        # make lookup table for mapping
        self.poses_source =dataset_config['poses_source']

        # self.poses_source = rospy.get_param('poses_source', 'odom')
        self.vitals_source = rospy.get_param('vitals_source', 'power')

        self.waypoints_topic = rospy.get_param('waypoint_topic', 'update_geo_wpt')

        self.get_states_srv = rospy.get_param('get_states_service', 'helm/get_states')
        self.get_state_srv  = rospy.get_param('get_state_service', 'helm/get_state')

        self.change_state_srv = rospy.get_param('change_state_service', 'helm/change_state')


        self.power_items_source= rospy.get_param('power_items', ['power_manager/jetson',
                                                                 'power_manager/24v',
                                                                 'power_manager/lumen'])

    
    def setup_ros(self):
        self.poses_sub = message_filters.Subscriber(self.poses_source, Odometry)
        self.vitals_sub = message_filters.Subscriber(self.vitals_source, Power)
    

        self.ts = message_filters.ApproximateTimeSynchronizer([self.poses_sub, self.vitals_sub], 10, 0.1)

        # self.ts.registerCallback(self.callback)

        self.geo_wpt_pub = rospy.Publisher("helm/path3d/update_geo_points", GeoPath, queue_size=10)
        self.geo_wpt_msg = GeoPath()

    def shutdown_node():    
        rospy.loginfo("Shutting down subscriber!")
        rospy.signal_shutdown("Shutting down subscriber!")

    #obtain pose and power information and store in the database 
    def callback(self):
        quad = [self.poses_sub.pose.pose.orientation.x, 
                self.poses_sub.pose.pose.orientation.y, 
                self.poses_sub.pose.pose.orientation.z, 
                self.poses_sub.pose.pose.orientation.w]
        euler_angles = euler_from_quaternion(quad)

        with app.app_context():
            poses = Poses.query.first()
            poses.roll = euler_angles[0] * 180 / np.pi
            poses.pitch = euler_angles[1] * 180 / np.pi 
            poses.yaw = euler_angles[2] * 180 / np.pi
            poses.x = self.poses_sub.pose.pose.position.x
            poses.y = self.poses_sub.pose.pose.position.y
            poses.z = self.poses_sub.pose.pose.position.z
            poses.u = self.poses_sub.twist.twist.linear.x
            poses.v = self.poses_sub.twist.twist.linear.y
            poses.w = self.poses_sub.twist.twist.linear.z
            poses.p = self.poses_sub.twist.twist.angular.x
            poses.q = self.poses_sub.twist.twist.angular.y
            poses.r = self.poses_sub.twist.twist.angular.z
            
            vitals = Vitals.query.first()
            vitals.voltage = self.vitals_sub.voltage
            vitals.current = self.vitals_sub.current

            db.session.commit() 

    ##publishing waypoints 
    def publish_wpt(self):
        waypoints = Waypoints.query.order_by(Waypoints.id).all()
        count = 0
        for entry in waypoints:
            gpose = GeoPoseStamped()
            gpose.header.seq = count
            gpose.pose.position.latitude = entry.lat
            gpose.pose.position.longitude = entry.lon
            gpose.pose.position.altitude = - entry.z
            self.geo_wpt_msg.poses.append(gpose)
            count = count +1

        self.geo_wpt_pub.publish(self.geo_wpt_msg)

def shutdown_node():    
    rospy.loginfo("Shutting down subscriber!")
    rospy.signal_shutdown("Shutting down subscriber!")


def gui_ros_start():
    rospy.init_node('mvp_gui_node', disable_signals=True)
    node = gui_ros()

global_file_name = './config/gui_config.yaml'

pose_t = threading.Thread(target = gui_ros_start)
pose_t.daemon = True
pose_t.start()
# except rospy.ROSInterruptException: pass
rospy.on_shutdown(shutdown_node)

