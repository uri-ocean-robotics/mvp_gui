from flask import request, redirect, url_for, render_template, jsonify
from mvp_gui import app, socketio
from mvp_gui.remote_manager import *
from mvp_gui.gui_ros_manager import *
from mvp_gui.routes.routes_systems import ros_system

def handle_post_request(request):
    """Handle POST requests to the topic page"""
    
    if 'rostopic_list' in request.form:
        ros_system.handle_keywords('topic', request.form['ros_topic_keyword'])
        ros_system.update_topic_database()
        return redirect(url_for('ros_topics_page'))

    elif 'echo_1' in request.form:
        topic_id = request.form['echo_1']
        topic_name = RosTopicList.query.get(topic_id)
        command = yaml_config['ros_source_base'] + "rostopic echo -n 1 " +  topic_name.name
        try:
            response = subprocess.run(
                ['bash', '-c', command], 
                env=env, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                text=True, 
                check=True, 
                timeout=SUBPROCESS_TIMEOUT)  
            return redirect(url_for('echo_topic', response=response.stdout)) 
        except subprocess.TimeoutExpired:
            return redirect(url_for('echo_topic', response="No incoming message after {} s".format(SUBPROCESS_TIMEOUT))) 
        except subprocess.CalledProcessError as e:
            return redirect(url_for('echo_topic', response="empty")) 
    
    # Keyword management
    elif 'remove_keywords' in request.form:
        ros_system.remove_all_keywords('topic')
        return redirect(url_for('ros_topics_page'))
        
    elif 'remove_single_keyword' in request.form:
        keyword_id = request.form['remove_single_keyword']
        ros_system.remove_single_keyword('topic', keyword_id)
        return redirect(url_for('ros_topics_page')) 
    
    return None

@app.route('/ros_topics', methods=['GET', 'POST'])
def ros_topics_page():
    rostopic_list = RosTopicList.query.all()
    rostopic_keyword = RosTopicKeywords.query.all()
    ros_system.cleanup_dead_nodes()

    # Handle form submissions
    if request.method == 'POST':
        response = handle_post_request(request)
        if response:
            return response
 
    return render_template(
        "ros_topics.html",  
        topic_list = rostopic_list,
        keyword_list = rostopic_keyword,
        current_page = "ros_topics"
    )

@app.route('/echo', methods=['GET', 'POST'])
def echo_topic():
    response = request.args.get('response')
    if response != None:
        cat_string = response.splitlines()
        if request.method == 'POST':
            ### remote connection
            if 'return' in request.form:
                return redirect(url_for('ros_topics_page'))

        return render_template("echo.html", info = cat_string)
    else:
        return redirect(url_for('ros_topics_page'))