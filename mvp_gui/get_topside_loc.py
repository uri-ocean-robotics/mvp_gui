import io
import pynmea2
import serial
from mvp_gui import *


ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

with app.app_context():
    pose_topside = PoseTopside.query.first()
    if pose_topside == None:
        pose_topside = PoseTopside()
        pose_topside.id = 1
        db.session.add(pose_topside)
    db.session.query(PoseHistoryTopside).delete()
    db.session.commit()

decay = 120
while True:
    try:
        line = sio.readline()
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

                #increase id by 1
                db.session.query(PoseHistoryTopside).update({PoseHistoryTopside.id: PoseHistoryTopside.id + 1})
                db.session.query(PoseHistoryTopside).filter(PoseHistoryTopside.id > decay).delete()
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

            # print(msg.timestamp, msg.latitude, msg.longitude, msg.altitude)
    
    except serial.SerialException as e:
        print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        continue
