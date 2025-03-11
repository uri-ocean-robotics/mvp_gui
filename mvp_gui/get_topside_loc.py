import io
import pynmea2
import serial
import rospy
from geographic_msgs.msg import GeoPoseStamped
from mvp_gui import *

class gps_topside():
    def __init__(self):
        try:
            # serial port
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1.0)
            self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))

            # rosnode
            rospy.init_node('gps_topside_node', disable_signals=True)
            self.rate = rospy.Rate(0.5)
            self.setup_gpsdecay = 30
            self.topside_geopose_pub = rospy.Publisher('/topside/geopose', GeoPoseStamped, queue_size=10)
            self.clear_pervious_gps()

            self.main_loop()
        except serial.serialutil.SerialException as e:
            print("No Topside GPS Module!!!")
        except rospy.exceptions.ROSException as e:
            print("GPS top side node not working!")

    def main_loop(self):
        while not rospy.is_shutdown():
            self.read_gps()
            self.rate.sleep()

    def clear_pervious_gps(self):
        with app.app_context():
            db.session.query(PoseTopside).delete()

            pose_topside = PoseTopside()
            pose_topside.id = 1
            pose_topside.lat = 0
            pose_topside.lon = 0
            pose_topside.z = 0
            db.session.add(pose_topside)
            
            db.session.query(PoseHistoryTopside).delete()
            db.session.commit()

    def read_gps(self):
        try:
            line = self.sio.readline()
            msg = pynmea2.parse(line.strip())
            if msg.sentence_type == 'GGA':
                # print(msg.timestamp, msg.latitude, msg.longitude, msg.altitude)
                with app.app_context():
                    pose_topside = PoseTopside.query.first()
                    if pose_topside == None:
                        pose_topside = PoseTopside()
                        pose_topside.id = 1
                    if msg.latitude != None and msg.longitude != None:
                        pose_topside.lat = float(msg.latitude)
                        pose_topside.lon = float(msg.longitude)
                        pose_topside.z = float(msg.altitude)
                    else:
                        pose_topside.lat = 0.0
                        pose_topside.lon = 0.0
                        pose_topside.z = 0.0
                    
                    db.session.add(pose_topside)

                    # publish to topic
                    pose_topside_ros = GeoPoseStamped()
                    pose_topside_ros.header.stamp = rospy.Time.now()
                    pose_topside_ros.pose.position.latitude = pose_topside.lat
                    pose_topside_ros.pose.position.longitude = pose_topside.lon
                    pose_topside_ros.pose.position.altitude = pose_topside.z

                    self.topside_geopose_pub.publish(pose_topside_ros)

                    #increase id by 1
                    db.session.query(PoseHistoryTopside).update({PoseHistoryTopside.id: PoseHistoryTopside.id + 1})
                    db.session.query(PoseHistoryTopside).filter(PoseHistoryTopside.id > self.setup_gpsdecay).delete()
                    db.session.commit()

                    # Create a new instance of PoseHistory with the values from new_pose
                    new_pose_history_topside = PoseHistoryTopside()
                    new_pose_history_topside.id = 1
                    new_pose_history_topside.lat = pose_topside.lat
                    new_pose_history_topside.lon = pose_topside.lon
                    new_pose_history_topside.z = pose_topside.z

                    # Add the new entry to the session and commit
                    db.session.add(new_pose_history_topside)
                    db.session.commit()


        except serial.SerialException as e:
            print('Device error: {}'.format(e))
        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            


if __name__ == "__main__":
    gps_topside()

