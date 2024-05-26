import json
import os

from mvp_gui import *
import threading
from mvp_gui.forms import WaypointForm
import xml.etree.ElementTree as ET
import yaml
from mvp_gui.utils import global_file_name, ros_source



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
    return render_template("home.html", vitals=vitals, poses=poses, current_page='home')


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
    return render_template("power_manager.html", items=items, vitals=vitals, current_page='power_manager')


@app.route('/waypoint_drag', methods=['POST'])
def waypoint_drag():
    data = request.json
    waypoint_id = data['id']
    new_lon = data['lng']
    new_lat = data['lat']
    new_alt = data['alt']

    waypoint = Waypoints.query.get(waypoint_id)
    if waypoint:
        waypoint.lon = new_lon
        waypoint.lat = new_lat
        waypoint.alt = new_alt
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
    return render_template("mission.html", waypoints=waypoints, current_page = "mission")

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
        "yaw": float(poses.yaw),
        "alt": float(poses.z)
    }

    # Sort the waypoints by ID
    waypoints = Waypoints.query.order_by(Waypoints.id).all()
    waypoints_data = [
        {"id": waypoint.id, "lat": float(waypoint.lat), "lon": float(waypoint.lon), "alt": float(waypoint.alt)}
        for waypoint in waypoints
    ]

    ##current waypoint list
    cwaypoints = CurrentWaypoints.query.all()
    current_waypoints_data = [
        {"id": cwaypoint.id, "lat": float(cwaypoint.lat), "lon": float(cwaypoint.lon), "alt": float(cwaypoint.alt)}
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
                                        pose_jsn =pose_data, controller_state = controller_state,
                                        current_page = "map")


## systems tools for launch files
@app.route('/systems', methods=['GET', 'POST'])
def systems_page():
    ##get roslaunch files
    dataset_config = yaml.safe_load(open(global_file_name, 'r'))
    roslaunch_folder = dataset_config['roslaunch_folder']

    roslaunch_list = RosLaunchList.query.all()
    rosnode_list = RosNodeList.query.all()
    rostopic_list = RosTopicList.query.all()

    remote_connection  = ssh_connection.is_connected()
    ## buttons
    if request.method == 'POST':
         ### remote connection
        if 'connect' in request.form:
            ssh_connection.connect()
            return redirect(url_for('systems_page'))
        
        elif 'disconnect' in request.form:
            ssh_connection.close()
            return redirect(url_for('systems_page'))
        
        ###roscore stuff
        elif 'roscore_start' in request.form:
            ssh_connection.execute_command(ros_source + "roscore", wait=False)
            print("Start ROS core")
            return redirect(url_for('systems_page'))
        
        elif 'roscore_stop' in request.form:
            ssh_connection.execute_command("killall -9 rosmaster")
            print("Stop ROS core")
            return redirect(url_for('systems_page'))

        elif 'rosnode_cleanup' in request.form:
            cleanup_action = RosActions.query.filter_by(action='rosnode_cleanup').first()
            cleanup_action.pending =1
            db.session.commit()

        ##roslaunch
        elif 'roslaunch_list' in request.form:
            if remote_connection: 
                command = "ls " +  roslaunch_folder
                response = ssh_connection.execute_command(command, wait=True)
                launch_list = response[0].splitlines()
                count = 0
                db.session.query(RosLaunchList).delete()
                for item in launch_list:
                    launch_ = RosLaunchList(id=count, name = item)
                    db.session.add(launch_)
                    db.session.commit()
                    count = count + 1
                    # print(item)
            else:
                db.session.query(RosLaunchList).delete()
                launch_ = RosLaunchList(id=0, name = 'Clicked without Connection')
                db.session.add(launch_)
                db.session.commit()
            return redirect(url_for('systems_page'))

        elif 'launch' in request.form:
            launch_id = request.form['launch']
            ##get the package name and launch file
            temp_launch = RosLaunchList.query.get(launch_id)
            command = ros_source + "roslaunch " + temp_launch
            # print(command)
            ssh_connection.execute_command(command, wait=False)
            return redirect(url_for('systems_page'))
        
        elif 'info' in request.form:
            launch_id = request.form['info']
            temp_launch = RosLaunchList.query.get(launch_id)
            command = "cat " + roslaunch_folder + "/" + temp_launch.name
            response = ssh_connection.execute_command(command, wait=True)
            
            return redirect(url_for('launch_file_data', response=response[0])) 
            
        ##get ros node list
        elif 'rosnode_list' in request.form:
            if remote_connection: 
                command = ros_source + "rosnode list"
                response = ssh_connection.execute_command(command, wait=True)
                node_list = response[0].splitlines()
                count = 0
                db.session.query(RosNodeList).delete()
                for item in node_list:
                    node_ = RosNodeList(id=count, name = item)
                    db.session.add(node_)
                    db.session.commit()
                    count = count + 1
                    # print(item)
            else:
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=count, name = 'No Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))
        
        elif 'kill_all_nodes' in request.form:
            if remote_connection: 
                command = ros_source + "rosnode kill -a"
                ssh_connection.execute_command(command, wait=False)
            else:
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=count, name = 'Clicked without Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))


        elif 'kill_node' in request.form:
            if remote_connection: 
                kill_id = request.form['kill_node']
                temp_kill = RosNodeList.query.get(kill_id)
                command = ros_source + "rosnode kill " + temp_kill.name
                ssh_connection.execute_command(command, wait=False)
            else:
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=count, name = 'Clicked without Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))
        
         ##get ros topic list
        elif 'rostopic_list' in request.form:
            if remote_connection: 
                command = ros_source + "rostopic list"
                response = ssh_connection.execute_command(command, wait=True)
                topic_list = response[0].splitlines()
                count = 0
                db.session.query(RosTopicList).delete()
                for item in topic_list:
                    node_ = RosTopicList(id=count, name = item)
                    db.session.add(node_)
                    db.session.commit()
                    count = count + 1
                    # print(item)
            else:
                db.session.query(RosTopicList).delete()
                node_ = RosTopicList(id=count, name = 'No Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))
            
    return render_template("systems.html", 
                           launch_list = roslaunch_list, 
                           node_list = rosnode_list, 
                           topic_list = rostopic_list,
                           remote_connection = str(remote_connection),
                           current_page = "systems")


@app.route('/launch_file_info', methods=['GET', 'POST'])
def launch_file_data():
    response = request.args.get('response')
    cat_string = response.splitlines()
    # cat_string = response
    if request.method == 'POST':
         ### remote connection
        if 'return' in request.form:
            return redirect(url_for('systems_page'))
    
    return render_template("roslaunch_info.html", info = cat_string)

##javascaript routes
@app.route('/latest_data', methods=['GET', 'POST'])
def latest_data():
    poses = Poses.query.first()
    vehicle_data = {
        "lat": float(poses.lat),
        "lon": float(poses.lon),
        "yaw": float(poses.yaw),
        "alt": float(poses.z)
    }

    waypoints = Waypoints.query.order_by(Waypoints.id).all()
    waypoints_data = [
        {"id": waypoint.id, "lat": float(waypoint.lat), "lon": float(waypoint.lon), "alt": float(waypoint.alt)}
        for waypoint in waypoints
    ]
    cwaypoints = CurrentWaypoints.query.all()
    current_waypoints_data = [
        {"id": cwaypoint.id, "lat": float(cwaypoint.lat), "lon": float(cwaypoint.lon), "alt": float(cwaypoint.alt)}
        for cwaypoint in cwaypoints
    ]

    pose_history = PoseHistory.query.order_by(PoseHistory.id).all()
    pose_data = [
        {"id": pose.id, "lat": float(pose.lat), "lon": float(pose.lon)}
        for pose in pose_history
    ]

    return jsonify({"vehicle": vehicle_data, "waypoints": waypoints_data, "current_waypoints": current_waypoints_data, "pose":pose_data})



@app.route('/tiles/<path:filename>')
def serve_tiles(filename):
    print(f"Request for tile: {filename}, {TILES_DIR}")
    return send_from_directory(TILES_DIR, filename)


@app.route('/path/to/api/endpoint')
def get_latest_yaw():
    pose = Poses.query.first()  # Or however you fetch the latest pose    
    heading_data = {
        'yaw': float(pose.yaw)
    }
    return jsonify({"heading_data": heading_data})


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


