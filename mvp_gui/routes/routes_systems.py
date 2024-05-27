from mvp_gui import *
import yaml 


## systems tools for launch files
@app.route('/', methods=['GET', 'POST'])
def systems_page():
    ##get roslaunch files
    dataset_config = yaml.safe_load(open(global_file_name, 'r'))
    roslaunch_folder = dataset_config['roslaunch_folder']

    roslaunch_list = RosLaunchList.query.all()
    rosnode_list = RosNodeList.query.all()
    rostopic_list = RosTopicList.query.all()

    remote_connection  = ssh_connection.is_connected()
    
    ##check ros master
    roscore_status =False
    if remote_connection:
        command = ros_source + "rosnode list"
        response = ssh_connection.execute_command(command, wait=True)
        node_output = response[0].splitlines()
        #if any node listed?
        if node_output:
            roscore_status = True
    
    ##check mvp_gui node 
    mvpgui_status = False
    mvpgui_node_name = '/mvp_gui_node'
    try:
        mvpgui_command = 'source /opt/ros/noetic/setup.bash && rosnode list'
        mvpgui_result = subprocess.run(['bash', '-c', mvpgui_command], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        nodes = mvpgui_result.stdout.splitlines()
        mvpgui_status =  mvpgui_node_name in nodes
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
            # print("Start ROS core")
            time.sleep(1.0)
            return redirect(url_for('systems_page'))
        
        elif 'roscore_stop' in request.form:
            ssh_connection.execute_command("killall -9 rosmaster")
            # print("Stop ROS core")
            return redirect(url_for('systems_page'))

        elif 'rosnode_cleanup' in request.form:
            cleanup_action = RosActions.query.filter_by(action='rosnode_cleanup').first()
            cleanup_action.pending =1
            db.session.commit()
        
        ### mvp_gui node
        elif 'mvpgui_start' in request.form:
            stop_ros_process()
            start_ros_process()
            time.sleep(1.0)
            return redirect(url_for('systems_page'))

        elif 'mvpgui_stop' in request.form:
            stop_ros_process()
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
                        launch_ = RosLaunchList(id=count, name = roslaunch_folder+ item)
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
            
            command = ros_source + "roslaunch " + temp_launch.name

            # ssh_connection.execute_command(command, wait=False)
            ssh_connection.execute_command_with_x11(command)
            time.sleep(20)
            return redirect(url_for('systems_page'))
        
        elif 'info' in request.form:
            launch_id = request.form['info']
            temp_launch = RosLaunchList.query.get(launch_id)
            command = "cat " + temp_launch.name
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
                node_ = RosNodeList(id=0, name = 'No Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))
        
        elif 'kill_all_nodes' in request.form:
            if remote_connection: 
                command = ros_source + "rosnode kill -a"
                ssh_connection.execute_command(command, wait=False)
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
            else:
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=0, name = 'Clicked without Connection')
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
                node_ = RosTopicList(id=0, name = 'No Connection')
                db.session.add(node_)
                db.session.commit()
            return redirect(url_for('systems_page'))
            
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