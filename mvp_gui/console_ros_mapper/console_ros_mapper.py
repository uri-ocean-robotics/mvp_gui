import subprocess
from nav_msgs.msg import Odometry
from mvp_msgs.msg import HelmState
from geographic_msgs.msg import GeoPoseStamped
import yaml


import rospy

class ConsoleROSMapper():
    ns = 'alpha_rise'
    odom_sub_topic = 'odometry/filtered/local'
    geopose_sub_topic = 'odometry/geopose'
    get_helm_state_service_name = 'helm/get_state'

    #################### get topic data####################
    def get_helm_state_data(self):
        
        result = subprocess.run(['rosservice', 'call', '/' + self.ns + '/' + self.get_helm_state_service_name, '\"\"'],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True)
        # Capture the output
        if result.returncode == 0:
            # print(result.stdout)
            # Extract the pose message from stdout
            helm_state_info = self.parse_helm_state_srv_response(result.stdout)
            return helm_state_info
        else:
            print(f"Error: {result.stderr}")
            return None


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
            odom_msg = self.parse_odometry_from_string(result.stdout)
            return odom_msg
        else:
            print(f"Error: {result.stderr}")
            return None


    #################### parsing ####################
    def parse_helm_state_srv_response(self, data_string):
        helm_state_info = HelmState()

        data = yaml.safe_load(data_string)

        # Access the transitions
        helm_state_info.name = data['state']['name']
        helm_state_info.mode = data['state']['mode']
        helm_state_info.transitions = data['state']['transitions']

        return helm_state_info

    def parse_odometry_from_string(self, data_string):
        
        # Initialize Odometry message
        odometry = Odometry()
        documents = yaml.safe_load_all(data_string)

        # Extract the first document
        data = next(documents, None)  # Get the first document

        odometry.header.stamp= rospy.Time(int(data['header']['stamp']['secs']), int(data['header']['stamp']['nsecs'])) 

        odometry.header.frame_id = data['header']['frame_id']
        odometry.child_frame_id = data['child_frame_id']
        odometry.pose.pose.position.x = data['pose']['pose']['position']['x']
        odometry.pose.pose.position.y = data['pose']['pose']['position']['y']
        odometry.pose.pose.position.z = data['pose']['pose']['position']['z']
        odometry.pose.pose.orientation.x = data['pose']['pose']['orientation']['x']
        odometry.pose.pose.orientation.y = data['pose']['pose']['orientation']['y']
        odometry.pose.pose.orientation.z = data['pose']['pose']['orientation']['z']
        odometry.pose.pose.orientation.w = data['pose']['pose']['orientation']['w']

        odometry.twist.twist.linear.x = data['twist']['twist']['linear']['x']
        odometry.twist.twist.linear.y = data['twist']['twist']['linear']['y']
        odometry.twist.twist.linear.z = data['twist']['twist']['linear']['z']

        odometry.twist.twist.angular.x = data['twist']['twist']['angular']['x']
        odometry.twist.twist.angular.y = data['twist']['twist']['angular']['y']
        odometry.twist.twist.angular.z = data['twist']['twist']['angular']['z']

        return odometry

    def parse_geo_pose_from_string(self, data_string):
        geopose = GeoPoseStamped()

        documents = yaml.safe_load_all(data_string)

        # Extract the first document
        data = next(documents, None)  # Get the first document

        geopose.header.stamp= rospy.Time(int(data['header']['stamp']['secs']), int(data['header']['stamp']['nsecs'])) 

        geopose.header.frame_id = data['header']['frame_id']

        geopose.pose.position.latitude = data['pose']['position']['latitude']
        geopose.pose.position.longitude = data['pose']['position']['longitude']
        geopose.pose.position.altitude = data['pose']['position']['altitude']
        geopose.pose.orientation.x = data['pose']['orientation']['x']
        geopose.pose.orientation.y = data['pose']['orientation']['y']
        geopose.pose.orientation.z = data['pose']['orientation']['z']
        geopose.pose.orientation.w = data['pose']['orientation']['w']

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
        helm_info = mapper.get_helm_state_data()

        

        # Print the Odometry message or handle it further
        if odometry_msg:
            odometry_publisher.publish(odometry_msg)
            
        if(geopose_msg):
            geopose_publisher.publish(geopose_msg)

        if(helm_info):
            print(helm_info)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    main()