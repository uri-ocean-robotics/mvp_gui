from flask import request, redirect, url_for, render_template, jsonify
from mvp_gui import app, socketio
from mvp_gui.ros_manager import SSHConnection
from mvp_gui.remote_manager import *
from mvp_gui.gui_ros_manager import *

# Socket.IO event handlers
@socketio.on('connect', namespace='/terminal')
def handle_connect():
    """Handle client connection to terminal namespace"""
    print('Client connected to terminal')

@socketio.on('disconnect', namespace='/terminal')
def handle_disconnect():
    """Handle client disconnection from terminal namespace"""
    print('Client disconnected from terminal')

# Event emitter function
def emit_message(message):
    """Emit a message to connected clients"""
    socketio.emit('terminal_output', {'data': message}, namespace='/terminal')

# Create a global instance of the remote manager
ros_system = RemoteManager()

# Get SSH connection from the server manager 
ssh_connection = SSHConnection(yaml_config['remote_host'], yaml_config['remote_user'], yaml_config['remote_password'])


@app.route('/', methods=['GET', 'POST'])
def systems_page():
    """Main systems page route handler"""
    # Get data for the page
    roslaunch_list = RosLaunchList.query.all()
    rosnode_list = RosNodeList.query.all()
    rosnode_keyword = RosNodeKeywords.query.all()
    # rosthread_list = RosThreadList.query.all()
    rosactivelaunch_list = RosActiveLaunchList.query.all()

    remote_connection = ssh_connection.is_connected()
    
    # Check system status
    roscore_status = ros_system.check_roscore_status()
    mvpgui_status = ros_system.check_node_status('/mvp_gui_node') if roscore_status else False

    # Handle form submissions
    if request.method == 'POST':
        response = handle_post_request(request, ssh_connection)
        if response:
            return response

    # Render the page with data
    return render_template(
        "systems.html", 
        launch_list=roslaunch_list,
        # thread_list=rosthread_list, 
        active_launch_list = rosactivelaunch_list,
        node_list=rosnode_list, 
        keyword_list=rosnode_keyword,
        remote_connection=str(remote_connection),
        remote_hostname=str(ssh_connection.hostname),
        remote_username=str(ssh_connection.username),
        roscore_status=str(roscore_status),
        mvpgui_status=str(mvpgui_status),
        roslaunch_folder=ros_system.roslaunch_folder,
        current_page="systems"
    )


def handle_post_request(request, ssh_connection):
    """Handle POST requests to the systems page"""
    
    # SSH connection management
    if 'ssh_connect' in request.form:
        return handle_ssh_connect(request, ssh_connection)
        
    elif 'ssh_disconnect' in request.form:
        ssh_connection.close()
        return redirect(url_for('systems_page'))
        
    elif 'export_rosmasteruri' in request.form:
        if request.form['ros_master_uri']:
            ros_system.update_ros_master_uri(request.form['ros_master_uri'])
        return redirect(url_for('systems_page'))
    
    # ROS core management
    elif 'roscore_start' in request.form:
        if ssh_connection.is_connected():
            ros_system.start_roscore(ssh_connection, emit_message)
            return render_template("terminal.html")
            
    elif 'roscore_stop' in request.form:
        if ssh_connection.is_connected():
            ros_system.stop_roscore(ssh_connection)
        return redirect(url_for('systems_page'))
        
    elif 'rosnode_cleanup' in request.form:
        ros_system.cleanup_dead_nodes()
        ros_system.update_node_database()
        return redirect(url_for('systems_page'))
    
    ### mvp_gui nodes
    elif 'mvpgui_start' in request.form:
        stop_ros_process(env)
        start_ros_process(env)
        ros_system.get_node_list()
        return redirect(url_for('systems_page'))

    elif 'mvpgui_stop' in request.form:
        node_name = stop_ros_process(env)
        ros_system.cleanup_dead_nodes(node_name)
        ros_system.get_node_list()
        return redirect(url_for('systems_page'))

    # ROS launch file management
    elif 'roslaunch_list' in request.form:
        if ssh_connection.is_connected():
            ros_system.roslaunch_folder = request.form['roslaunch_folder']
            ros_system.update_launch_database(ssh_connection, ros_system.roslaunch_folder)
        else:
            # Clear and add placeholder when not connected
            db.session.query(RosLaunchList).delete()
            launch_ = RosLaunchList(id=0, folder_dir='', name='Clicked without Connection', pending=0)
            db.session.add(launch_)
            db.session.commit()
        return redirect(url_for('systems_page'))
        
    elif 'launch' in request.form:
        launch_id = request.form['launch']
        ros_system.start_launch_file(launch_id)
        return redirect(url_for('systems_page'))

    elif 'terminate_launch_file' in request.form:
        launch_id = request.form['terminate_launch_file']
        ros_system.terminate_launch_file(launch_id)
        return redirect(url_for('systems_page'))
        
    # elif 'terminate_thread' in request.form:
    #     if ssh_connection.is_connected():
    #         thread_id = request.form['terminate_thread']
    #         thread_entry = RosThreadList.query.get(thread_id)
    #         ros_system.terminate_thread(ssh_connection, thread_entry)
    #         time.sleep(2.0)
    #         ros_system.update_node_database()
    #     return redirect(url_for('systems_page'))
        
    elif 'info' in request.form:
        if ssh_connection.is_connected():
            launch_id = request.form['info']
            temp_launch = RosLaunchList.query.get(launch_id)
            response = ros_system.get_launch_file_content(ssh_connection, temp_launch)
            return redirect(url_for('launch_file_data', response=response))
    
    # Node management
    elif 'rosnode_list' in request.form:
        ros_system.handle_keywords('node', request.form['ros_node_keyword'])
        ros_system.update_node_database()
        return redirect(url_for('systems_page'))
        
    elif 'kill_all_nodes' in request.form:
        if ssh_connection.is_connected():
            ros_system.kill_all_nodes(ssh_connection)
            time.sleep(1.0)
            ros_system.update_node_database()
        else:
            # Add placeholder when not connected
            db.session.query(RosNodeList).delete()
            node_ = RosNodeList(id=0, name='Clicked without Connection')
            db.session.add(node_)
            db.session.commit()
        return redirect(url_for('systems_page'))
        
    elif 'kill_node' in request.form:
        if ssh_connection.is_connected():
            node_id = request.form['kill_node']
            temp_node = RosNodeList.query.get(node_id)
            ros_system.kill_specific_node(ssh_connection, temp_node.name)
            time.sleep(1.0)
            ros_system.update_node_database()
        else:
            # Add placeholder when not connected
            db.session.query(RosNodeList).delete()
            node_ = RosNodeList(id=0, name='Clicked without Connection')
            db.session.add(node_)
            db.session.commit()
        return redirect(url_for('systems_page'))
    
    # Keyword management
    elif 'remove_keywords' in request.form:
        ros_system.remove_all_keywords('node')
        return redirect(url_for('systems_page'))
        
    elif 'remove_single_keyword' in request.form:
        keyword_id = request.form['remove_single_keyword']
        ros_system.remove_single_keyword('node', keyword_id)
        return redirect(url_for('systems_page'))
    
    return None


def handle_ssh_connect(request, ssh_connection):
    """Handle SSH connection request"""
    ssh_connection.hostname = request.form['hostname']
    ssh_connection.username = request.form['username']
    ssh_connection.password = request.form['password']
    
    if ssh_connection.connect():
        # Connection successful
        return redirect(url_for('systems_page'))
    else:
        ssh_connection.close()
        return redirect(url_for('ssh_failed'))


@app.route('/ssh_failed', methods=['GET', 'POST'])
def ssh_failed():
    """SSH connection failure page"""
    if request.method == 'POST' and 'return' in request.form:
        return redirect(url_for('systems_page'))
    return render_template("ssh_failed.html")


@app.route('/launch_file_info', methods=['GET', 'POST'])
def launch_file_data():
    """Display launch file contents"""
    response = request.args.get('response')
    cat_string = response.splitlines() if response else []
    
    if request.method == 'POST' and 'return' in request.form:
        return redirect(url_for('systems_page'))
    
    return render_template("roslaunch_info.html", info=cat_string)


@app.route('/current_system_status')
def current_status():
    """API endpoint for current system status"""
    # Convert the list to something JSON serializable (e.g., a list of full paths)
    active_launches_data = [{"id": item.id, "full_path": item.full_path} for item in RosActiveLaunchList.query.all()]
    
    return jsonify({
        "remote_connection": {
            "data": ssh_connection.is_connected()
        },
        "roscore_status": {
            "data": ros_system.check_roscore_status()
        },
        "mvpgui_status": {
            "data": ros_system.check_node_status('/mvp_gui_node')
        },
        "connected_ros_master": {
            "data": ros_system.get_ros_master_uri()
        },
        "active_launches": active_launches_data
    })