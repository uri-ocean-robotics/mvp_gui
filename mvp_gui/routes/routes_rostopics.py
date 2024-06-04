from mvp_gui import *
import rosnode
import rosgraph

def cleanup_dead_nodes():
    try:
        _, unpinged = rosnode.rosnode_ping_all()    
        if unpinged:
            if unpinged:
                master = rosgraph.Master("")
                rosnode.cleanup_master_blacklist(master, unpinged)
    except rosnode.ROSNodeIOException as e:
        pass

@app.route('/ros_topics', methods=['GET', 'POST'])
def ros_topics_page():
    global ros_source
    ros_master_uri = 'http://' + ssh_connection.hostname  + ':11311/'
    ros_hostname = ssh_connection.hostname 
    env['ROS_MASTER_URI'] = ros_master_uri
    os.environ['ROS_MASTER_URI'] = ros_master_uri
    ros_source = ros_source_base + f"export ROS_MASTER_URI={ros_master_uri} && export ROS_IP={ros_hostname} && ROS_HOSTNAME={ros_hostname} &&"

    server_ip = app.config['HOST_IP']
    env['ROS_IP'] = server_ip
    os.environ['ROS_IP'] = server_ip

    rostopic_list = RosTopicList.query.all()

    remote_connection  = ssh_connection.is_connected()

    ## buttons
    if 'rostopic_list' in request.form:
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
        return redirect(url_for('ros_topics_page'))
    
    elif 'echo_1' in request.form:
        if remote_connection: 
            topic_id = request.form['echo_1']
            topic_name = RosTopicList.query.get(topic_id)
            command = ros_source + "rostopic echo -n 1 " +  topic_name.name
            response = ssh_connection.execute_command(command, wait=False, timeout=5)
            return redirect(url_for('echo_topic', response=response[0])) 
            
    return render_template("ros_topics.html",  
               topic_list = rostopic_list,
                current_page = "ros_topics")



@app.route('/echo', methods=['GET', 'POST'])
def echo_topic():
    response = request.args.get('response')
    if response != None:
        cat_string = response.splitlines()
        # cat_string = response
        if request.method == 'POST':
            ### remote connection
            if 'return' in request.form:
                return redirect(url_for('ros_topics_page'))

        return render_template("echo.html", info = cat_string)
    else:
        return redirect(url_for('ros_topics_page'))
