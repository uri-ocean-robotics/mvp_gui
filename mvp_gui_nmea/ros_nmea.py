#!/usr/bin/env python

import rospy
import message_filters

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoseStamped

from ros_nmea_config import *

class RosToNMEAConverter:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ros_to_nmea_converter', anonymous=True)
        
        self.mode = 'robot'
        if self.mode == 'robot':
            # Subscribe to the relevant topic(s)
            self.odom_sub = rospy.Subscriber('/odometry/filtered/local', Odometry, self.odom_callback)
            self.geo_pose_sub = rospy.Subscriber('/geopose',GeoPoseStamped, self.geopose_callback)
            self.lumen_sub = rospy.Subscriber('/lumen_brightness',Float64,self.lumen_callback)

            # self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.geo_pose_sub, self.lumen_sub], 10, 0.1)
            # self.ts.registerCallback(self.msg_callback)
            self.timer = rospy.Timer(rospy.Duration(1.0), self.nmea_send_callback)

            self.geopose_nmea_string=""
            self.odom_nmea_string=""
            self.lumen_nmea_string =""

    def nmea_send_callback(self, event):
        rospy.loginfo(self.odom_nmea_string)
        rospy.loginfo(self.geopose_nmea_string)
        rospy.loginfo(self.lumen_nmea_string)

    def odom_callback(self, odom_msg):
        # Convert the odom message to an NMEA string
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        x_ori = odom_msg.pose.pose.orientation.x
        y_ori = odom_msg.pose.pose.orientation.y
        z_ori = odom_msg.pose.pose.orientation.z
        w_ori = odom_msg.pose.pose.orientation.w
        u = odom_msg.twist.twist.linear.x
        v = odom_msg.twist.twist.linear.y
        w = odom_msg.twist.twist.linear.z
        p = odom_msg.twist.twist.angular.x
        q = odom_msg.twist.twist.angular.y
        r = odom_msg.twist.twist.angular.z

        # Construct the NMEA string
        self.odom_nmea_string = (
            f"{ODOM_NMEA_HEADER},"
            f"{odom_msg.header.stamp},"
            f"{odom_msg.header.frame_id},"
            f"{odom_msg.child_frame_id},"
            f"{x:.2f},"
            f"{y:.2f},"
            f"{z:.2f},"
            f"{x_ori:.2f},"
            f"{y_ori:.2f},"
            f"{z_ori:.2f},"
            f"{w_ori:.2f},"
            f"{u:.2f},"
            f"{v:.2f},"
            f"{w:.2f}"
            f"{p:.2f},"
            f"{q:.2f},"
            f"{r:.2f}"
        )        
        # Calculate checksum (CS)
        checksum = self.calculate_checksum(self.odom_nmea_string)
        self.odom_nmea_string += f"*{checksum:02X}"  
        

    def geopose_callback(self, geo_msg):
        latitude = geo_msg.pose.position.x
        longitude = geo_msg.pose.position.y
        altitude = geo_msg.pose.position.z
        x_ori = geo_msg.pose.orientation.x
        y_ori = geo_msg.pose.orientation.y
        z_ori = geo_msg.pose.orientation.z
        w_ori = geo_msg.pose.orientation.w
        self.geopose_nmea_string = (
            f"{GEOPOSE_NMEA_HEADER},"
            f"{geo_msg.header.stamp},"
            f"{latitude:.2f},"
            f"{longitude:.2f},"
            f"{altitude:.2f},"
            f"{x_ori:.2f},"
            f"{y_ori:.2f},"
            f"{z_ori:.2f},"
            f"{w_ori:.2f},"
        )        
        # Calculate checksum (CS)
        checksum = self.calculate_checksum(self.geopose_nmea_string)
        self.geopose_nmea_string += f"*{checksum:02X}"  
        

    def lumen_callback(self, lumen_msg):
        self.lumen_nmea_string = f"{LUMEN_NMEA_HEADER},{lumen_msg.data}"
        checksum = self.calculate_checksum(self.lumen_nmea_string)
        self.lumen_nmea_string += f"*{checksum:02X}"  
        

    def calculate_checksum(self, nmea_string):
        # Calculate the checksum for the NMEA string
        checksum = 0
        for char in nmea_string[1:]:  # Skip the starting '$'
            checksum ^= ord(char)
        return checksum

    def start(self):
        # Start the ROS loop
        rospy.spin()

if __name__ == '__main__':
    converter = RosToNMEAConverter()
    converter.start()
