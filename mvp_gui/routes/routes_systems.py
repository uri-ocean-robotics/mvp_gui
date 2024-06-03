from mvp_gui import *
import subprocess
import yaml 
import rosnode
import rosgraph

ros_master_uri = 'http://' + ssh_connection.hostname  + ':11311/'
ros_hostname = ssh_connection.hostname 
env['ROS_MASTER_URI'] = ros_master_uri
os.environ['ROS_MASTER_URI'] = ros_master_uri
ros_source += f"export ROS_MASTER_URI={ros_master_uri} && export ROS_IP={ros_hostname} && ROS_HOSTNAME={ros_hostname} &&"

roslaunch_folder = roslaunch_folder_default

def check_mvpgui_status(mvpgui_node_name, env):
    mvpgui_command = 'source /opt/ros/noetic/setup.bash && rosnode list'
    try:
        mvpgui_result = subprocess.run(['bash', '-c', mvpgui_command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        nodes = mvpgui_result.stdout.splitlines()
        mvpgui_status =  mvpgui_node_name in nodes
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
    except rosnode.ROSNodeIOException as e:
        pass
    

def check_roscore_status(ssh_connection, remote_connection):
    roscore_status =False
    if remote_connection:
        command = ros_source + "rosnode list"
        response = ssh_connection.execute_command(command, wait=True)
        node_output = response[0].splitlines()
        #if any node listed?
        if node_output:
            roscore_status = True
    
    return roscore_status


## systems tools for launch files
@app.route('/', methods=['GET', 'POST'])
def systems_page():
    global roslaunch_folder
    server_ip = app.config['HOST_IP']
    env['ROS_IP'] = server_ip
    os.environ['ROS_IP'] = server_ip

    ##get roslaunch files
    dataset_config = yaml.safe_load(open(global_file_name, 'r'))
    # roslaunch_folder = dataset_config['roslaunch_folder']
    # roslaunch_folder = roslaunch_folder_default

    roslaunch_list = RosLaunchList.query.all()
    rosnode_list = RosNodeList.query.all()
    rostopic_list = RosTopicList.query.all()

    remote_connection  = ssh_connection.is_connected()
    
    ##check ros master
    roscore_status = check_roscore_status(ssh_connection, remote_connection)
    # roscore_status =False
    # if remote_connection:
    #     command = ros_source + "rosnode list"
    #     response = ssh_connection.execute_command(command, wait=True)
    #     node_output = response[0].splitlines()
    #     #if any node listed?
    #     if node_output:
    #         roscore_status = True
    
    ##check mvp_gui node 
    mvpgui_status = False
    mvpgui_node_name = '/mvp_gui_node'
    try:
        mvpgui_status = check_mvpgui_status(mvpgui_node_name, env)
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while listing ROS nodes: {e.stderr}")
        pass
    

    ## buttons
    if request.method == 'POST':
         ### remote connection
        if 'ssh_connect' in request.form:
            ssh_connection.hostname = request.form['hostname']
            ssh_connection.username = request.form['username']
            ssh_connection.password = request.form['password']
            ssh_connection.connect()
            if ssh_connection.connect():
                # If SSH connection is successful, redirect to systems_page
                return redirect(url_for('systems_page'))
            else:
                ssh_connection.close()
                # If SSH connection fails, you can render an error page or redirect to a different route
                # flash("SSH connection failed. Please check your credentials.")
                return redirect(url_for('ssh_failed'))  # Redirect to login page or error page
        
        elif 'ssh_disconnect' in request.form:
            ssh_connection.close()
            return redirect(url_for('systems_page'))
        
        ###roscore stuff
        elif 'roscore_start' in request.form:
            ssh_connection.execute_command(ros_source + "roscore", wait=False)
            time.sleep(1.0)
            return redirect(url_for('systems_page'))
        
        elif 'roscore_stop' in request.form:
            ssh_connection.execute_command("killall -9 rosmaster && killall -9 roscore && killall -9 rviz")
            return redirect(url_for('systems_page'))

        elif 'rosnode_cleanup' in request.form:
            # cleanup_action = RosActions.query.filter_by(action='rosnode_cleanup').first()
            # cleanup_action.pending =1
            # db.session.commit()
            cleanup_dead_nodes()
            mvpgui_status = check_mvpgui_status(mvpgui_node_name, env)

        
        ### mvp_gui node
        elif 'mvpgui_start' in request.form:
            stop_ros_process(env)
            cleanup_dead_nodes()
            start_ros_process(env)
            time.sleep(1.0)
            return redirect(url_for('systems_page'))

        elif 'mvpgui_stop' in request.form:
            stop_ros_process(env)
            cleanup_dead_nodes()
            time.sleep(1.0)
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

                ssh_connection.execute_command(command, wait=False)
                # ssh_connection.execute_command_with_x11(command)
                time.sleep(20)
            return redirect(url_for('systems_page'))
        
        elif 'launch_xvfb' in request.form:
            if remote_connection: 
                launch_id = request.form['launch_xvfb']
                
                ##get the package name and launch file
                temp_launch = RosLaunchList.query.get(launch_id)
                command = ros_source + "roslaunch " + temp_launch.folder_dir + temp_launch.name
                # ssh_connection.execute_command(command, wait=False)
                ssh_connection.execute_command_with_xvfb(command)
                time.sleep(20)
            return redirect(url_for('systems_page'))
        
        elif 'info' in request.form:
            if remote_connection: 
                launch_id = request.form['info']
                temp_launch = RosLaunchList.query.get(launch_id)
                command = "cat " + temp_launch.folder_dir + temp_launch.name
                response = ssh_connection.execute_command(command, wait=True)
            
            return redirect(url_for('launch_file_data', response=response[0])) 
            
        ##get ros node list
        elif 'rosnode_list' in request.form:
            if remote_connection: 
                cleanup_dead_nodes()
                time.sleep(0.5)
                command = ros_source + "rosnode list"
                response = ssh_connection.execute_command(command, wait=True)
                node_list = response[0].splitlines()
                count = 0
                db.session.query(RosNodeList).delete()
                for item in node_list:
                    node_ = RosNodeList(id=count, name = item)
                    db.session.add(node_)
                    # db.session.commit()
                    count = count + 1
                    # print(item)
                db.session.commit()

            else:
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=0, name = 'No Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))
        
        elif 'kill_all_nodes' in request.form:
            if remote_connection: 
                command = ros_source + "rosnode kill -a"
                ssh_connection.execute_command(command, wait=False)
                cleanup_dead_nodes()
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
            else:
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=0, name = 'Clicked without Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))
        
         ##get ros topic list
        elif 'rostopic_list' in request.form:
            if remote_connection: 
                cleanup_dead_nodes()
                time.sleep(0.5)
                command = ros_source + "rostopic list"
                response = ssh_connection.execute_command(command, wait=True)
                topic_list = response[0].splitlines()
                count = 0
                db.session.query(RosTopicList).delete()
                for item in topic_list:
                    node_ = RosTopicList(id=count, name = item)
                    db.session.add(node_)
                    # db.session.commit()
                    count = count + 1
                    # print(item)
                db.session.commit()
            else:
                db.session.query(RosTopicList).delete()
                node_ = RosTopicList(id=0, name = 'No Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))
        
        elif 'echo_1' in request.form:
            if remote_connection: 
                topic_id = request.form['echo_1']
                topic_name = RosTopicList.query.get(topic_id)
                command = ros_source + "rostopic echo -n 1 " +  topic_name.name
                response = ssh_connection.execute_command(command, wait=False, timeout=3)
                return redirect(url_for('echo_topic', response=response[0])) 
            
    return render_template("systems.html", 
                           launch_list = roslaunch_list, 
                           node_list = rosnode_list, 
                           topic_list = rostopic_list,
                           remote_connection = str(remote_connection),
                           remote_hostname  = str(ssh_connection.hostname),
                           remote_username = str(ssh_connection.username),
                           roscore_status = str(roscore_status),
                           mvpgui_status = str(mvpgui_status),
                           roslaunch_folder = roslaunch_folder,
                           current_page = "systems")


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

@app.route('/echo', methods=['GET', 'POST'])
def echo_topic():
    response = request.args.get('response')
    if response != None:
        cat_string = response.splitlines()
        # cat_string = response
        if request.method == 'POST':
            ### remote connection
            if 'return' in request.form:
                return redirect(url_for('systems_page'))

        return render_template("echo.html", info = cat_string)
    else:
        return redirect(url_for('systems_page'))


    


@app.route('/current_system_status')
def current_status():
    remote_connection_tab  = {
        "data": ssh_connection.is_connected()
    }
    if ssh_connection.is_connected():
        roscore_status = {
            "data": check_roscore_status(ssh_connection, ssh_connection.is_connected())
        }
        mvpgui_node_name = '/mvp_gui_node'
        try:
            mvpgui_status = {
                "data": check_mvpgui_status(mvpgui_node_name, env)
            }
        except subprocess.CalledProcessError as e:
            mvpgui_status = {
                "data": False
            }
    else:
        roscore_status = {
            "data": False
        }
        mvpgui_status = {
            "data": False
        }


    return jsonify({"remote_connection": remote_connection_tab, "roscore_status": roscore_status, "mvpgui_status": mvpgui_status})
