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
    global env
    # ros_source = ros_source_base + f"export ROS_MASTER_URI={env['ROS_MASTER_URI']} &&"
    os.environ['ROS_MASTER_URI'] = env['ROS_MASTER_URI']

    rostopic_list = RosTopicList.query.all()
    rostopic_keyword = RosTopicKeywords.query.all()
    cleanup_dead_nodes()

    timeout_subprocess = 5

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
        )
        # Step 2: Delete the Duplicates
        db.session.query(RosTopicKeywords).filter(RosTopicKeywords.id.in_(subquery)).delete(synchronize_session=False)
        # Retrieve the remaining entries sorted by their current ID
        remaining_entries = db.session.query(RosTopicKeywords).order_by(RosTopicKeywords.id).all()
        # Update the IDs to be sequential starting from 0
        for index, entry in enumerate(remaining_entries):
            entry.id = index
        db.session.commit()
        
        try:
            command = "rostopic list"
            if len(RosTopicKeywords.query.all()) == 0:
                response = subprocess.run(['bash', '-c', command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=timeout_subprocess)
            else:
                keywords_list = RosTopicKeywords.query.all()
                command += " |grep '"
                for count, item in enumerate(keywords_list):
                    if count != len(keywords_list) - 1:
                        command += str(item.name) + "\|"
                    else:
                        command += str(item.name) 
                command += "'"
                response = subprocess.run(['bash', '-c', command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=timeout_subprocess)
            topic_list = response.stdout.splitlines()
            count = 0
            db.session.query(RosTopicList).delete()
            for item in topic_list:
                node_ = RosTopicList(id=count, name = item)
                db.session.add(node_)
                count = count + 1                
            db.session.commit()
        except subprocess.CalledProcessError as e:
            db.session.query(RosTopicList).delete()
            topic_ = RosTopicList(id=0, name = 'empty')
            db.session.add(topic_)
            db.session.commit()

        return redirect(url_for('ros_topics_page'))
    
    elif 'echo_1' in request.form:
        topic_id = request.form['echo_1']
        topic_name = RosTopicList.query.get(topic_id)
        command_source = "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && "
        command = command_source + "rostopic echo -n 1 " +  topic_name.name
        try:
            response = subprocess.run(['bash', '-c', command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=timeout_subprocess)  
            return redirect(url_for('echo_topic', response=response.stdout)) 
        except subprocess.TimeoutExpired:
            return redirect(url_for('echo_topic', response="No incoming message after {} s".format(timeout_subprocess))) 
        except subprocess.CalledProcessError as e:
            return redirect(url_for('echo_topic', response="empty")) 

            
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
