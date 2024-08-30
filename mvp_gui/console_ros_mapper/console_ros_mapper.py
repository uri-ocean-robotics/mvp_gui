import subprocess
from nav_msgs.msg import Odometry

from geographic_msgs.msg import GeoPoseStamped
import rospy

class ConsoleROSMapper():
    ns = 'alpha_rise'
    odom_sub_topic = 'odometry/filtered/local'
    geopose_sub_topic = 'odometry/geopose'

    #################### get topic data####################
    def get_geopose_data(self):
        # Run the shell command to get odometry data
        result = subprocess.run(['rostopic', 'echo', '-n', '1', self.ns + '/' + self.geopose_sub_topic],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True)
        # Capture the output
        if result.returncode == 0:
            # print(result.stdout)
            # Extract the pose message from stdout
            geopose_msg = self.parse_geo_pose_from_string(result.stdout)
            return geopose_msg
        else:
            print(f"Error: {result.stderr}")
            return None
 
    def get_odometry_data(self):
        # Run the shell command to get odometry data
        result = subprocess.run(['rostopic', 'echo', '-n', '1', self.ns + '/' + self.odom_sub_topic],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True)
        # Capture the output
        if result.returncode == 0:
            # print(result.stdout)
            # Extract the pose message from stdout
            odom_msg = self.parse_oometry_from_string(result.stdout)
            return odom_msg
        else:
            print(f"Error: {result.stderr}")
            return None


    #################### parsing ####################
    def parse_oometry_from_string(self, data_string):
        
        time_index =  3
        frame_index = 5
        child_frame_index = 6
        position_index = 9
        orientation_index = 13
        linear_index = 21
        angular_index = 25
        
        # Initialize Odometry message
        odometry = Odometry()

        # Split the output string into lines
        lines = data_string.split('\n')

        if 'secs' in lines[time_index].split(':')[0]:
            odometry.header.stamp= rospy.Time(int(lines[time_index].split(':')[1]), int(lines[time_index+1].split(':')[1])) 
            
        if 'frame_id' in lines[frame_index].split(':')[0]:
            s = lines[frame_index].split(':')[1]
            s= s.strip()
            odometry.header.frame_id = s.replace('"', '')

        if 'child_frame_id' in lines[child_frame_index].split(':')[0]:
            s = lines[child_frame_index].split(':')[1]
            s= s.strip()
            odometry.child_frame_id = s.replace('"', '')

        if 'position' in lines[position_index].split(':')[0]:
            odometry.pose.pose.position.x = float(lines[position_index+1].split(': ')[1])
            odometry.pose.pose.position.y = float(lines[position_index+2].split(': ')[1])
            odometry.pose.pose.position.z = float(lines[position_index+3].split(': ')[1])

        if 'orientation' in lines[orientation_index].split(':')[0]:
            odometry.pose.pose.orientation.x = float(lines[orientation_index+1].split(': ')[1])
            odometry.pose.pose.orientation.y = float(lines[orientation_index+2].split(': ')[1])
            odometry.pose.pose.orientation.z = float(lines[orientation_index+3].split(': ')[1])
            odometry.pose.pose.orientation.w = float(lines[orientation_index+4].split(': ')[1])

        if 'linear' in lines[linear_index].split(':')[0]:
            odometry.twist.twist.linear.x = float(lines[linear_index+1].split(': ')[1])
            odometry.twist.twist.linear.y = float(lines[linear_index+2].split(': ')[1])
            odometry.twist.twist.linear.z = float(lines[linear_index+3].split(': ')[1])

        if 'angular' in lines[angular_index].split(':')[0]:
            odometry.twist.twist.angular.x = float(lines[angular_index+1].split(': ')[1])
            odometry.twist.twist.angular.y = float(lines[angular_index+2].split(': ')[1])
            odometry.twist.twist.angular.z = float(lines[angular_index+3].split(': ')[1])

        return odometry

    def parse_geo_pose_from_string(self, data_string):
        time_index =  3
        frame_index = 5
        position_index = 7
        orientation_index = 11

        # Initialize Odometry message
        geopose = GeoPoseStamped()

        # Split the output string into lines
        lines = data_string.split('\n')

        if 'secs' in lines[time_index].split(':')[0]:
            geopose.header.stamp= rospy.Time(int(lines[time_index].split(':')[1]), int(lines[time_index+1].split(':')[1])) 
            
        if 'frame_id' in lines[frame_index].split(':')[0]:
            s = lines[frame_index].split(':')[1]
            s= s.strip()
            geopose.header.frame_id = s.replace('"', '')


        if 'position' in lines[position_index].split(':')[0]:
            geopose.pose.position.latitude = float(lines[position_index+1].split(': ')[1])
            geopose.pose.position.longitude = float(lines[position_index+2].split(': ')[1])
            geopose.pose.position.altitude = float(lines[position_index+3].split(': ')[1])

        if 'orientation' in lines[orientation_index].split(':')[0]:
            geopose.pose.orientation.x = float(lines[orientation_index+1].split(': ')[1])
            geopose.pose.orientation.y = float(lines[orientation_index+2].split(': ')[1])
            geopose.pose.orientation.z = float(lines[orientation_index+3].split(': ')[1])
            geopose.pose.orientation.w = float(lines[orientation_index+4].split(': ')[1])

        return geopose



def main():
    rospy.init_node('odometry_mapper', anonymous=True)
    odometry_publisher = rospy.Publisher('/topside/odometry/filtered/local', Odometry, queue_size=10)
    geopose_publisher = rospy.Publisher('/topside/odometry/geopose', GeoPoseStamped, queue_size=10)

    # Set the loop rate to 1 Hz
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Get the Odometry message
        mapper = ConsoleROSMapper()
        odometry_msg = mapper.get_odometry_data()
        geopose_msg =mapper.get_geopose_data()

        # Print the Odometry message or handle it further
        if odometry_msg:
            odometry_publisher.publish(odometry_msg)
            geopose_publisher.publish(geopose_msg)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    main()