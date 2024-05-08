import time
# import numpy as np
import threading
# import rospy
# import message_filters
from datetime import datetime
# from nav_msgs.msg import Odometry
# from tf2_msgs.msg import TFMessage
# from tf.transformations import quaternion_matrix, quaternion_from_euler, quaternion_from_matrix, euler_from_quaternion
from mvp_gui import *

def update_load():
    with app.app_context():
        while True:
            time.sleep(1)
            turbo.push(turbo.replace(render_template("tables/health_table.html"), 'power_health'))
            turbo.push(turbo.replace(render_template("tables/pose_table.html"), 'pose_info'))
            turbo.push(turbo.replace(render_template("tables/power_manager_table.html"), 'power_manager'))

# def update_pose():
#     rospy.init_node('get_gt', anonymous=True, disable_signals=True)
#     odom_sf_sub = message_filters.Subscriber('/bluerov2/odom_sf', Odometry)
#     ts = message_filters.ApproximateTimeSynchronizer([odom_sf_sub], 10, 0.1)
#     ts.registerCallback(callback)

# def callback(odom_sf_sub):
#     quad = [odom_sf_sub.pose.pose.orientation.x, odom_sf_sub.pose.pose.orientation.y, odom_sf_sub.pose.pose.orientation.z, odom_sf_sub.pose.pose.orientation.w]
#     euler_angles = euler_from_quaternion(quad)
    
#     with app.app_context():
#         poses = Poses.query.first()
#         poses.roll = euler_angles[0] * 180 / np.pi
#         poses.pitch = euler_angles[1] * 180 / np.pi 
#         poses.yaw = euler_angles[2] * 180 / np.pi
#         poses.x = odom_sf_sub.pose.pose.position.x
#         poses.y = odom_sf_sub.pose.pose.position.y
#         poses.z = odom_sf_sub.pose.pose.position.z

#         poses.u = odom_sf_sub.twist.twist.linear.x
#         poses.v = odom_sf_sub.twist.twist.linear.y
#         poses.w = odom_sf_sub.twist.twist.linear.z
#         poses.p = odom_sf_sub.twist.twist.angular.x
#         poses.q = odom_sf_sub.twist.twist.angular.y
#         poses.r = odom_sf_sub.twist.twist.angular.z

#         db.session.commit() 

# def shutdown_node():    
#     rospy.loginfo("Shutting down subscriber!")
#     rospy.signal_shutdown("Shutting down subscriber!")

# thread
update_t = threading.Thread(target=update_load)
update_t.daemon = True
update_t.start()

# rospy.init_node('get_gt', anonymous=True, disable_signals=True)
# pose_t = threading.Thread(target=update_pose)
# pose_t.daemon = True
# pose_t.start()

# rospy.on_shutdown(shutdown_node)