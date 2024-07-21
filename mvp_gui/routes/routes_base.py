
from mvp_gui import *
# from mvp_gui.forms import WaypointForm
import xml.etree.ElementTree as ET

@app.context_processor
def inject_load():
    with app.app_context():
        vitals = Vitals.query.first()
        poses = Poses.query.first()
        items = PowerItems.query.all()
        waypoints = Waypoints.query.all()
        states = HelmStates.query.all()
        controller_state = ControllerState.query.first()
        return {'vitals': vitals, 'poses': poses, 'items': items, 'waypoints': waypoints, 'states':states, 'controller_state':controller_state}


@app.route("/vehicle_status", methods=['GET', 'POST'])
def vehicle_status_page():
    vitals = Vitals.query.first()
    poses = Poses.query.first()
    return render_template("vehicle_status.html", vitals=vitals, poses=poses, current_page='vehicle_status')


@app.route('/path/to/api/endpoint')
def get_latest_yaw():
    pose = Poses.query.first()  
    heading_data = {
        'yaw': float(pose.yaw)
    }
    return jsonify({"heading_data": heading_data})


@app.route('/vehicle_status/states')
def home_state_data():
    poses = Poses.query.first()
    pose_data = {
        "x": float(poses.x),
        "y": float(poses.y),
        "z": float(poses.z),
        "roll": float(poses.roll),
        "pitch": float(poses.pitch),
        "yaw": float(poses.yaw),
        "u": float(poses.u),
        "v": float(poses.v),
        "w": float(poses.w),
        "p": float(poses.p),
        "q": float(poses.q),
        "r": float(poses.r),
        "lat": float(poses.lat),
        "lon": float(poses.lon),
        "frame_id": str(poses.frame_id),
        "child_frame_id": str(poses.child_frame_id)
    }

    vitals = Vitals.query.first()
    vital_data ={
        "voltage": float(vitals.voltage),
        "current": float(vitals.current)
    }
    return jsonify({"pose_data": pose_data, "vital_data": vital_data})


