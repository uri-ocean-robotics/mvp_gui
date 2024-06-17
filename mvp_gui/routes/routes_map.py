from mvp_gui import *

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
    for tile_dir in sorted(os.listdir(TILES_DIR_1)):
        loc_dir_1 = os.path.join(TILES_DIR_1, tile_dir)
        loc_dir_2 = os.path.join(TILES_DIR_2, tile_dir)
        full_path = os.path.join(loc_dir_1, filename)
        if os.path.exists(full_path):
            print(f"Request for tile: {filename}, {loc_dir_2}")
            return send_from_directory(loc_dir_2, filename)    
    return 'No Tile Available'