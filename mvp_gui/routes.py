import json
import os

from mvp_gui import *
import threading
from mvp_gui.forms import WaypointForm
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

# routes
@app.route("/", methods=['GET', 'POST'])
def home_page():
    vitals = Vitals.query.first()
    poses = Poses.query.first()
    return render_template("home.html", vitals=vitals, poses=poses)


@app.route("/power_manager",  methods=['GET', 'POST']) 
def power_manager_page():
    vitals = Vitals.query.first()
    items = PowerItems.query.all()
    # print("power manager")
    if request.method == 'POST':
        action = request.form.get('action')
        for item in items:
            # print(str(item.id))
            if action == str(item.id):
                if item.status == 'On':
                    item.status = 'Off'
                    db.session.commit()
                    ##call rosservice
                else:
                    item.status = "On"
                    db.session.commit()
                    ##call rosservice
            # return render_template("power_manager.html", items=items, vitals=vitals)
    return render_template("power_manager.html", items=items, vitals=vitals)


@app.route('/waypoint_drag', methods=['POST'])
def waypoint_drag():
    data = request.json
    waypoint_id = data['id']
    new_lon = data['lng']
    new_lat = data['lat']

    waypoint = Waypoints.query.get(waypoint_id)
    if waypoint:
        waypoint.lon = new_lon
        waypoint.lat = new_lat
        db.session.commit()

    return jsonify({"success": True})


@app.route("/mission", methods=['GET', 'POST'])
def mission_page():
    ## sort the waypoints by id
    waypoints = Waypoints.query.order_by(Waypoints.id).all()

    ##reassign the ID from 1 to N
    count = 1
    for entry in waypoints:
        entry.id = count
        count = count + 1
        db.session.commit()

    #button actiions
    if request.method == 'POST':
        # action = request.form.get('action')
        if 'add' in request.form:
            return redirect(url_for('add_waypoint_page')) 

        elif 'publish' in request.form:
            print("waypoint publish")

        ##delete a waypoint
        elif 'delete' in request.form:
            delete_id = request.form['delete']
            entry = Waypoints.query.get(delete_id)
            if entry:
                db.session.delete(entry)
                db.session.commit()
                return redirect(url_for('mission_page'))
    
        ##edit a waypoint
        elif 'edit' in request.form:
            edit_id = request.form['edit']
            return redirect(url_for('edit_waypoint_page', edit_id=edit_id)) 
        
        elif 'copy' in request.form:
            edit_id = request.form['copy']
            entry = Waypoints.query.get(edit_id)
            new_entry = Waypoints()
            new_entry.id = len(waypoints)+1 #already obtained before
            new_entry.lat = entry.lat
            new_entry.lon = entry.lon
            new_entry.alt = entry.alt

            db.session.add(new_entry)
            db.session.commit()
            return redirect(url_for('mission_page'))

    ##render the mission site
    return render_template("mission.html", waypoints=waypoints)

def generat_waypoints_from_kml(file_name, replace_flag):
    tree = ET.parse(file_name)
    root = tree.getroot()

    waypoints = Waypoints.query.all()
    id_data = len(waypoints)+1

    if replace_flag ==1:
        id_data=1
    new_waypoints = []
    for placemark in root.findall('.//{http://www.opengis.net/kml/2.2}Placemark'):
        coordinates = placemark.find('.//{http://www.opengis.net/kml/2.2}coordinates').text
        # Splitting coordinates into latitude, longitude, and altitude
        lon, lat, alt = map(float, coordinates.split(','))
        new_waypoints.append({'id': id_data, 'lat': lat, 'lon': lon, 'alt': alt})
        id_data = id_data +1
    if replace_flag == 1:
        db.session.query(Waypoints).delete()

    for waypoint_data in new_waypoints:
        waypoint = Waypoints(id=waypoint_data['id'],
                            lat=waypoint_data['lat'],
                            lon=waypoint_data['lon'],
                            alt=waypoint_data['alt'])
        db.session.add(waypoint)
    db.session.commit()
    

@app.route('/mission/upload', methods=['POST'])
def upload_file():
    fold_name = 'kml_uploads/'
    if request.method == 'POST':
        action = request.form.get('action')
        if action == 'replace':
        # if request.form.get('replace'):
            # print("replace pressed")
            if 'fileToUpload' not in request.files:
                return 'No file part'
            file = request.files['fileToUpload']
            if file.filename == '':
                return 'No selected file'
            ##do something
            if file:
                file.save(fold_name + file.filename)
                generat_waypoints_from_kml(fold_name + file.filename, 1)

        elif action == 'append':
        # elif request.form.get('append'):
            # print("append pressed")
            if 'fileToUpload' not in request.files:
                return 'No file part'
            file = request.files['fileToUpload']
            if file.filename == '':
                return 'No selected file'
            if file:
                file.save(fold_name + file.filename)
                generat_waypoints_from_kml(fold_name + file.filename, 0)
    return redirect(url_for('mission_page'))
    

@app.route('/mission/edit_waypoint', methods=['GET','POST'])
def edit_waypoint_page():
    edit_id = request.args.get('edit_id')
    entry = Waypoints.query.get(edit_id)
    form = WaypointForm()

    ## if submit pressed
    if form.validate_on_submit():
        entry.id = form.id.data
        entry.lat = form.lat.data
        entry.lon = form.lon.data
        entry.alt = form.alt.data
        db.session.commit()
        return redirect(url_for('mission_page'))
    return render_template('edit_waypoints.html', form=form, entry=entry)


@app.route('/mission/waypoints', methods=['GET','POST'])
def add_waypoint_page():
    form = WaypointForm()
    waypoints = Waypoints.query.all()
    id_data = len(waypoints)+1

    ## if submit pressed
    if form.validate_on_submit():
        ##get values    
        waypoint = Waypoints(id=id_data,
                           lat=form.lat.data,
                           lon=form.lon.data,
                           alt=form.alt.data,
                           )
        db.session.add(waypoint)
        db.session.commit()
        return redirect(url_for('mission_page'))
    return render_template('add_waypoints.html', form=form)


@app.route('/map', methods=['GET', 'POST'])
def map_page():
    host_ip = app.config['HOST_IP']
    # Get vehicle lat lon for map
    poses = Poses.query.first()
    vehicle_data = {
        "lat": float(poses.lat),
        "lon": float(poses.lon),
        "yaw": float(poses.yaw)
    }

    # Sort the waypoints by ID
    waypoints = Waypoints.query.order_by(Waypoints.id).all()
    waypoints_data = [
        {"id": waypoint.id, "lat": float(waypoint.lat), "lon": float(waypoint.lon)}
        for waypoint in waypoints
    ]

    ##current waypoint list
    cwaypoints = CurrentWaypoints.query.all()
    current_waypoints_data = [
        {"id": cwaypoint.id, "lat": float(cwaypoint.lat), "lon": float(cwaypoint.lon)}
        for cwaypoint in cwaypoints
    ]

    pose_history = PoseHistory.query.order_by(PoseHistory.id).all()
    pose_data = [
        {"id": pose.id, "lat": float(pose.lat), "lon": float(pose.lon)}
        for pose in pose_history
    ]

    states = HelmStates.query.all()

    #get controller mode
    controller_state =ControllerState.query.first()
    #button actiions
    if request.method == 'POST':
        ##state change
        if 'states' in request.form:
            selected_state = request.form.get('states')
            change_state = RosActions.query.filter_by(action='change_state').first()
            change_state.value = selected_state
            change_state.pending = 1
            db.session.commit()
            # return redirect(url_for('map_page'))

        ##controller state change
        elif 'controller_disable' in request.form:
            change_state = RosActions.query.filter_by(action='controller_state').first()
            change_state.value = "disable"
            change_state.pending = 1
            db.session.commit()

        elif 'controller_enable' in request.form:
            change_state = RosActions.query.filter_by(action='controller_state').first()
            change_state.value = "enable"
            change_state.pending = 1
            db.session.commit()
            # return redirect(url_for('map_page'))

        elif 'publish_waypoints' in request.form:
            publish_waypoints = RosActions.query.filter_by(action='publish_waypoints').first()
            publish_waypoints.pending = 1
            db.session.commit()
            # return redirect(url_for('map_page'))
        
    return render_template("map.html", items_jsn=waypoints_data, citems_jsn=current_waypoints_data, 
                                        vehicle_jsn=vehicle_data, host_ip=host_ip, states=states,
                                        pose_jsn =pose_data, controller_state = controller_state)


@app.route('/latest_data', methods=['GET', 'POST'])
def latest_data():
    poses = Poses.query.first()
    vehicle_data = {
        "lat": float(poses.lat),
        "lon": float(poses.lon),
        "yaw": float(poses.yaw)
    }

    waypoints = Waypoints.query.order_by(Waypoints.id).all()
    waypoints_data = [
        {"id": waypoint.id, "lat": float(waypoint.lat), "lon": float(waypoint.lon)}
        for waypoint in waypoints
    ]
    cwaypoints = CurrentWaypoints.query.all()
    current_waypoints_data = [
        {"id": cwaypoint.id, "lat": float(cwaypoint.lat), "lon": float(cwaypoint.lon)}
        for cwaypoint in cwaypoints
    ]

    pose_history = PoseHistory.query.order_by(PoseHistory.id).all()
    pose_data = [
        {"id": pose.id, "lat": float(pose.lat), "lon": float(pose.lon)}
        for pose in pose_history
    ]

    return jsonify({"vehicle": vehicle_data, "waypoints": waypoints_data, "current_waypoints": current_waypoints_data, "pose":pose_data})


@app.route("/monitor")
def monitor_page():
    return render_template("monitor.html")


@app.route('/tiles/<path:filename>')
def serve_tiles(filename):
    print(f"Request for tile: {filename}, {TILES_DIR}")
    return send_from_directory(TILES_DIR, filename)


@app.route('/path/to/api/endpoint')
def get_latest_yaw():
    pose = Poses.query.first()  # Or however you fetch the latest pose    
    yaw_data = {
        'yaw': pose.yaw
    }
    return jsonify(yaw_data)


@app.route('/mission/states')
def controller_helm_state_data():
    controller_state = ControllerState.query.first()
    controller_data = {
        "state" : str(controller_state.state)
    }

    helm_state = HelmStates.query.all()
    helm_state_data = [
        {"id": state.id, "name": str(state.name)}
        for state in helm_state
    ]

    return jsonify({"controller_data": controller_data, "helm_state_data": helm_state_data})



@app.route('/home/states')
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


