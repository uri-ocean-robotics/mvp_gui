from flask import request, redirect, url_for, render_template, jsonify
from mvp_gui import app, socketio, env, SUBPROCESS_TIMEOUT, yaml_config
from mvp_gui.models import RosTopicList, RosTopicKeywords
from mvp_gui.routes.routes_systems import ros_system
import subprocess
import re

def handle_post_request(request):
    """Handle POST requests to the topic page"""
    
    if 'rostopic_list' in request.form:
        ros_system.handle_keywords('topic', request.form['ros_topic_keyword'])
        ros_system.update_topic_database()
        return redirect(url_for('ros_topics_page'))

    elif 'echo_1' in request.form:
        topic_id = request.form['echo_1']
        with app.app_context():
            topic = RosTopicList.query.get(topic_id)
        
        if not topic or not topic.name or ' ' not in topic.name:
            return redirect(url_for('echo_topic', response="Invalid topic format"))

        # Topic name is in format '/topic_name [package/msg/Type]'
        match = re.match(r'(\S+)\s+\[(\S+)\]', topic.name)
        if not match:
            return redirect(url_for('echo_topic', response=f"Could not parse topic and type from '{topic.name}'"))
        
        topic_name = match.group(1)
        topic_type = match.group(2)
        
        # ROS 2 command: ros2 topic echo --once <topic_name> <topic_type>
        command = yaml_config['ros_source_base'] + f"ros2 topic echo --once {topic_name} {topic_type}"
        try:
            response = subprocess.run(
                ['bash', '-c', command],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
                timeout=SUBPROCESS_TIMEOUT)
            return redirect(url_for('echo_topic', response=response.stdout or response.stderr))
        except subprocess.TimeoutExpired:
            return redirect(url_for('echo_topic', response=f"No incoming message on {topic_name} after {SUBPROCESS_TIMEOUT}s"))
        except subprocess.CalledProcessError as e:
            return redirect(url_for('echo_topic', response=f"Error echoing topic:\n{e.stderr}"))
    
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
    with app.app_context():
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
    if response is not None:
        cat_string = response.splitlines()
        if request.method == 'POST':
            if 'return' in request.form:
                return redirect(url_for('ros_topics_page'))

        return render_template("echo.html", info = cat_string)
    else:
        return redirect(url_for('ros_topics_page'))