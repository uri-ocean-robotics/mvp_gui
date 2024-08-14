from mvp_gui import *
import subprocess
import yaml 
import rosnode
import rosgraph
import threading
from sqlalchemy import func

# ros_master_uri = 'http://' + ssh_connection.hostname  + ':11311/'
# ros_hostname = ssh_connection.hostname 
# env['ROS_MASTER_URI'] = ros_master_uri
# os.environ['ROS_MASTER_URI'] = ros_master_uri
# ros_source = ros_source_base + f"export ROS_MASTER_URI={ros_master_uri} && export ROS_IP={ros_hostname} && ROS_HOSTNAME={ros_hostname} &&"

roslaunch_folder = roslaunch_folder_default

def check_mvpgui_status(mvpgui_node_name, env):
    mvpgui_command = 'rosnode list'
    try:
        # cleanup_dead_nodes()
        mvpgui_result = subprocess.run(['bash', '-c', mvpgui_command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=5)
        nodes = mvpgui_result.stdout.splitlines()
        mvpgui_status =  mvpgui_node_name in nodes
    except subprocess.TimeoutExpired:
        mvpgui_status = False
    except subprocess.CalledProcessError as e:
        mvpgui_status = False
    return mvpgui_status

def cleanup_dead_nodes():
    try:
        _, unpinged = rosnode.rosnode_ping_all()    
        if unpinged:
            if unpinged:
                master = rosgraph.Master("")
                rosnode.cleanup_master_blacklist(master, unpinged)
        time.sleep(1.0)
    except rosnode.ROSNodeIOException as e:
        pass


def check_roscore_status(env):
    roscore_status =False
    # node_command = 'source /opt/ros/noetic/setup.bash && rosnode list'
    node_command = 'rosnode list'

    node_name = '/rosout'
    try:
        node_result = subprocess.run(['bash', '-c', node_command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=5)
        # nodes = node_result.communicate()
        nodes = node_result.stdout.splitlines()
        roscore_status =  node_name in nodes
    except subprocess.TimeoutExpired:
        roscore_status = False
    except subprocess.CalledProcessError as e:
        roscore_status = False
    return roscore_status
        

def check_ros_master_uri(env):
    check_ros_master_cmd = 'echo $ROS_MASTER_URI'
    current_ros_master_uri = subprocess.run(['bash', '-c', check_ros_master_cmd], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=5)
    return current_ros_master_uri.stdout

# Event emitter function
def emit_message(message):
    socketio.emit('terminal_output', {'data': message}, namespace='/terminal')

# get node with cmd_line
def get_node(timeout_subprocess):
    try:
        command = "rosnode list"
        if len(RosNodeKeywords.query.all()) == 0:
            response = subprocess.run(['bash', '-c', command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=timeout_subprocess)
        else:
            keywords_list = RosNodeKeywords.query.all()
            command += " |grep '"
            for count, item in enumerate(keywords_list):
                if count != len(keywords_list) - 1:
                    command += str(item.name) + "\|"
                else:
                    command += str(item.name) 
            command += "'"
            response = subprocess.run(['bash', '-c', command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=timeout_subprocess)
        node_list = response.stdout.splitlines()
        count = 0
        db.session.query(RosNodeList).delete()
        for item in node_list:
            node_ = RosNodeList(id=count, name = item)
            db.session.add(node_)
            count = count + 1                
        db.session.commit()
    except subprocess.CalledProcessError as e:
        db.session.query(RosNodeList).delete()
        node_ = RosNodeList(id=0, name = 'empty')
        db.session.add(node_)
        db.session.commit()

## systems tools for launch files
@app.route('/', methods=['GET', 'POST'])
def systems_page():
    global ros_source
    global roslaunch_folder
    global env
    global roscore_status

    roslaunch_list = RosLaunchList.query.all()
    rosnode_list = RosNodeList.query.all()
    rosnode_keyword = RosNodeKeywords.query.all()


    remote_connection  = ssh_connection.is_connected()
    
    ##check ros master
    roscore_status = False
    roscore_status = check_roscore_status(env)
    
    ##check mvp_gui node 
    mvpgui_status = False
    mvpgui_node_name = '/mvp_gui_node'
    if roscore_status:
        mvpgui_status = check_mvpgui_status(mvpgui_node_name, env)

    timeout_subprocess = 5

    ## buttons
    if request.method == 'POST':
         ### remote connection
        if 'ssh_connect' in request.form:
            ssh_connection.hostname = request.form['hostname']
            ssh_connection.username = request.form['username']
            ssh_connection.password = request.form['password']
            ssh_connection_status = ssh_connection.connect()

            if ssh_connection_status:
                ros_master_uri = 'http://' + ssh_connection.hostname  + ':11311/'
                env['ROS_MASTER_URI'] = ros_master_uri
                os.environ['ROS_MASTER_URI'] = ros_master_uri
                # ros_source = ros_source_base + f"export ROS_MASTER_URI={ros_master_uri} && export ROS_IP={ros_hostname} && ROS_HOSTNAME={ros_hostname} &&"
                ros_source = ros_source_base + f"export ROS_MASTER_URI={ros_master_uri} &&"
                # return redirect(url_for('systems_page'))
            else:
                ssh_connection.close()
                # If SSH connection fails, you can render an error page or redirect to a different route
                # flash("SSH connection failed. Please check your credentials.")
                return redirect(url_for('ssh_failed'))  # Redirect to login page or error page
        
        elif 'ssh_disconnect' in request.form:
            ssh_connection.close()
            return redirect(url_for('systems_page'))
        
        elif 'export_rosmasteruri' in request.form:
            if request.form['ros_master_uri'] != '':
                ros_master_uri = 'http://' + request.form['ros_master_uri']  + ':11311/'
                env['ROS_MASTER_URI'] = ros_master_uri
                os.environ['ROS_MASTER_URI'] = ros_master_uri
                ros_source = ros_source_base + f"export ROS_MASTER_URI={ros_master_uri} &&"
            return redirect(url_for('systems_page'))

        
        ###roscore stuff
        elif 'roscore_start' in request.form:
            if ssh_connection.is_connected():
                print(ros_source + "roscore")
                # ssh_connection.execute_command(ros_source + "roscore", wait=False)
                # ssh_connection.execute_command_disp_terminal(ros_source + "roscore")
                threading.Thread(target=ssh_connection.execute_command_disp_terminal, args=(ros_source + "roscore", emit_message)).start()

                # time.sleep(1.0)
                return render_template("terminal.html")

            # return redirect(url_for('systems_page'))
        
        elif 'roscore_stop' in request.form:
            if ssh_connection.is_connected():
                ssh_connection.execute_command(ros_source + "killall -9 rosmaster && killall -9 roscore && killall -9 rviz")
            return redirect(url_for('systems_page'))

        elif 'rosnode_cleanup' in request.form:
            cleanup_dead_nodes()
            get_node(timeout_subprocess)
            mvpgui_status = check_mvpgui_status(mvpgui_node_name, env)
            return redirect(url_for('systems_page'))
        
        ### mvp_gui node
        elif 'mvpgui_start' in request.form:
            print(env['ROS_MASTER_URI'])
            stop_ros_process(env)
            cleanup_dead_nodes()
            start_ros_process(env)
            get_node(timeout_subprocess)
            return redirect(url_for('systems_page'))

        elif 'mvpgui_stop' in request.form:
            print(env['ROS_MASTER_URI'])
            stop_ros_process(env)
            cleanup_dead_nodes()
            get_node(timeout_subprocess)
            return redirect(url_for('systems_page'))

        ##roslaunch
        elif 'roslaunch_list' in request.form:
            if remote_connection: 
                roslaunch_folder = request.form['roslaunch_folder']
                command = "ls " +  roslaunch_folder
                response = ssh_connection.execute_command(command, wait=True)
                launch_list = response[0].splitlines()
                count = 0
                db.session.query(RosLaunchList).delete()
                for item in launch_list:
                    if item.endswith(".launch"):
                        launch_ = RosLaunchList(id=count, folder_dir = roslaunch_folder, name = item)
                        db.session.add(launch_)
                        # db.session.commit()
                        count = count + 1
                    # print(item)
                db.session.commit()

            else:
                db.session.query(RosLaunchList).delete()
                launch_ = RosLaunchList(id=0, folder_dir='', name = 'Clicked without Connection')
                db.session.add(launch_)
                db.session.commit()
            return redirect(url_for('systems_page'))

        elif 'launch' in request.form:
            if remote_connection: 
                launch_id = request.form['launch']
                ##get the package name and launch file
                temp_launch = RosLaunchList.query.get(launch_id)
                command = ros_source + "roslaunch " + temp_launch.folder_dir + temp_launch.name
                threading.Thread(target=ssh_connection.execute_command_disp_terminal, args=(command, emit_message)).start()
                return render_template("terminal.html")

                # return redirect(url_for('terminal_page'))

                # ssh_connection.execute_command(command, wait=True)
                # time.sleep(5)
            return redirect(url_for('systems_page'))
        
        elif 'launch_xvfb' in request.form:
            if remote_connection: 
                launch_id = request.form['launch_xvfb']
                
                ##get the package name and launch file
                temp_launch = RosLaunchList.query.get(launch_id)
                command = ros_source + "roslaunch " + temp_launch.folder_dir + temp_launch.name
                # ssh_connection.execute_command(command, wait=False)
                ssh_connection.execute_command_with_xvfb(command)
                # time.sleep(20)
            return redirect(url_for('systems_page'))
        
        elif 'info' in request.form:
            if remote_connection: 
                launch_id = request.form['info']
                temp_launch = RosLaunchList.query.get(launch_id)
                command = "cat " + temp_launch.folder_dir + temp_launch.name
                response = ssh_connection.execute_command(command, wait=True)
            return redirect(url_for('launch_file_data', response=response[0])) 
            
        elif 'rosnode_list' in request.form:
            rosnode_keyword = request.form['ros_node_keyword']
            count = len(RosNodeKeywords.query.all())
            for keyword in rosnode_keyword.split(','):
                if keyword.strip() != '':
                    keyword_ = RosNodeKeywords(id=count, name = keyword.strip())
                    db.session.add(keyword_)
                    count += 1 

            # Get the IDs of the duplicates to be deleted
            subquery = db.session.query(
                RosNodeKeywords.id
            ).filter(
                RosNodeKeywords.id.notin_(
                    db.session.query(func.min(RosNodeKeywords.id)).group_by(RosNodeKeywords.name)
                )
            )
            # Step 2: Delete the Duplicates
            db.session.query(RosNodeKeywords).filter(RosNodeKeywords.id.in_(subquery)).delete(synchronize_session=False)
            # Retrieve the remaining entries sorted by their current ID
            remaining_entries = db.session.query(RosNodeKeywords).order_by(RosNodeKeywords.id).all()
            # Update the IDs to be sequential starting from 0
            for index, entry in enumerate(remaining_entries):
                entry.id = index
            db.session.commit()

            get_node(timeout_subprocess)

            return redirect(url_for('systems_page'))
        
        elif 'kill_all_nodes' in request.form:
            if remote_connection: 
                command = ros_source + "rosnode kill -a"
                ssh_connection.execute_command(command, wait=False)
                cleanup_dead_nodes()
                time.sleep(1.0)
                get_node(timeout_subprocess)
            else:
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=0, name = 'Clicked without Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))

        elif 'kill_node' in request.form:
            if remote_connection: 
                kill_id = request.form['kill_node']
                temp_kill = RosNodeList.query.get(kill_id)
                command = ros_source + "rosnode kill " + temp_kill.name
                ssh_connection.execute_command(command, wait=False)
                cleanup_dead_nodes()
                time.sleep(1.0)
                get_node(timeout_subprocess)
            else:
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=0, name = 'Clicked without Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))

        elif 'remove_keywords' in request.form:
            db.session.query(RosNodeKeywords).delete()
            db.session.commit()
            get_node(timeout_subprocess)
            return redirect(url_for('systems_page'))
    
        elif 'remove_single_keyword' in request.form:
            keyword_id = request.form['remove_single_keyword']
            db.session.query(RosNodeKeywords).filter(RosNodeKeywords.id == keyword_id).delete()
            db.session.query(RosNodeKeywords).filter(RosNodeKeywords.id > int(keyword_id)).update({RosNodeKeywords.id: RosNodeKeywords.id - 1})
            db.session.commit()
            get_node(timeout_subprocess)
            return redirect(url_for('systems_page'))
        
    return render_template("systems.html", 
                           launch_list = roslaunch_list, 
                           node_list = rosnode_list, 
                           keyword_list = rosnode_keyword,
                           remote_connection = str(remote_connection),
                           remote_hostname  = str(ssh_connection.hostname),
                           remote_username = str(ssh_connection.username),
                           roscore_status = str(roscore_status),
                           mvpgui_status = str(mvpgui_status),
                           roslaunch_folder = roslaunch_folder,
                           current_page = "systems")

@socketio.on('connect', namespace='/terminal')
def handle_connect():
    print('Client connected')

@socketio.on('disconnect', namespace='/terminal')
def handle_disconnect():
    print('Client disconnected')

@app.route('/ssh_failed', methods=['GET', 'POST'])
def ssh_failed():
    if request.method == 'POST':
         ### remote connection
        if 'return' in request.form:
            return redirect(url_for('systems_page'))
    return render_template("ssh_failed.html")


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
 


@app.route('/current_system_status')
def current_status():
    global env
    remote_connection_tab  = {
        "data": ssh_connection.is_connected()
    }
    connected_ros_master = {
        "data": check_ros_master_uri(env)
    }
    
    roscore_status = {
        # "data": check_roscore_status(ssh_connection, ssh_connection.is_connected())
        "data": check_roscore_status(env)
    }

    mvpgui_status = {
        "data": check_mvpgui_status('/mvp_gui_node', env)
    }



    return jsonify({"remote_connection": remote_connection_tab, "roscore_status": roscore_status, "mvpgui_status": mvpgui_status, "connected_ros_master": connected_ros_master})
