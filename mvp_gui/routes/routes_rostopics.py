from mvp_gui import *
import rosnode
import rosgraph
from sqlalchemy import func, and_

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
    rostopic_keyword = RosTopicKeywords.query.all()

    remote_connection  = ssh_connection.is_connected()

    ## buttons
    if 'rostopic_list' in request.form:
        rostopic_keyword = request.form['ros_topic_keyword']
        count = len(RosTopicKeywords.query.all())
        for keyword in rostopic_keyword.split(','):
            if keyword.strip() != '':
                keyword_ = RosTopicKeywords(id=count, name = keyword.strip())
                db.session.add(keyword_)
                count += 1 

        # Get the IDs of the duplicates to be deleted
        subquery = db.session.query(
            RosTopicKeywords.id
        ).filter(
            RosTopicKeywords.id.notin_(
                db.session.query(func.min(RosTopicKeywords.id)).group_by(RosTopicKeywords.name)
            )
        ).subquery()

        # Step 2: Delete the Duplicates
        db.session.query(RosTopicKeywords).filter(RosTopicKeywords.id.in_(subquery)).delete(synchronize_session=False)

        # Retrieve the remaining entries sorted by their current ID
        remaining_entries = db.session.query(RosTopicKeywords).order_by(RosTopicKeywords.id).all()

        # Update the IDs to be sequential starting from 0
        for index, entry in enumerate(remaining_entries):
            entry.id = index

        db.session.commit()
        
        if remote_connection:             
            cleanup_dead_nodes()
            time.sleep(0.5)
            command = ros_source + "rostopic list"
            if len(RosTopicKeywords.query.all()) == 0:
                response = ssh_connection.execute_command(command, wait=True)
            else:
                keywords_list = RosTopicKeywords.query.all()
                command += " |grep '"
                for count, item in enumerate(keywords_list):
                    if count != len(keywords_list) - 1:
                        command += str(item.name) + "\|"
                    else:
                        command += str(item.name) 
                command += "'"
                print(command)
                response = ssh_connection.execute_command(command, wait=True)
            
            topic_list = response[0].splitlines()
            print(topic_list)
            count = 0
            db.session.query(RosTopicList).delete()
            for item in topic_list:
                node_ = RosTopicList(id=count, name = item)
                db.session.add(node_)
                count = count + 1                
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
            
    elif 'remove_keywords' in request.form:
        db.session.query(RosTopicKeywords).delete()
        db.session.commit()
        return redirect(url_for('ros_topics_page'))
    
    elif 'remove_single_keyword' in request.form:
        keyword_id = request.form['remove_single_keyword']
        db.session.query(RosTopicKeywords).filter(RosTopicKeywords.id == keyword_id).delete()
        db.session.query(RosTopicKeywords).filter(RosTopicKeywords.id > int(keyword_id)).update({RosTopicKeywords.id: RosTopicKeywords.id - 1})
        db.session.commit()
        return redirect(url_for('ros_topics_page'))


    return render_template("ros_topics.html",  
               topic_list = rostopic_list,
               keyword_list = rostopic_keyword,
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
