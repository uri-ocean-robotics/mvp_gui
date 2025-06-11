from mvp_gui import db, yaml_config, env, SUBPROCESS_TIMEOUT, app
from mvp_gui.models import *
import subprocess
import threading
import time
from datetime import datetime
from sqlalchemy import func
import re

class RemoteManager:
    """Manages remote ROS 2 system operations and state"""
    def __init__(self):
        self.roslaunch_folder = yaml_config['roslaunch_folder']
        self.ros_source = yaml_config['ros_source_base']
        self.threads = []
        self.launch_files = []

    def check_roscore_status(self):
        """Check if ROS 2 is active by trying to list nodes."""
        node_command = self.ros_source + 'ros2 node list'
        try:
            # In ROS 2, there is no roscore. A good health check is if we can communicate with the daemon.
            result = subprocess.run(
                ['bash', '-c', node_command],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
                timeout=SUBPROCESS_TIMEOUT
            )
            # If the command succeeds, we assume ROS 2 is running.
            return result.returncode == 0
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return False

    def check_node_status(self, node_name):
        """Check if a specific node is running"""
        try:
            command = self.ros_source + 'ros2 node list'
            result = subprocess.run(
                ['bash', '-c', command],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
                timeout=SUBPROCESS_TIMEOUT
            )
            return node_name in result.stdout.splitlines()
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return False

    def cleanup_dead_nodes(self, node_name=None):
        """
        Clean up dead ROS nodes. This is less of an issue in ROS 2 due to DDS discovery.
        This function is kept for API compatibility but is a no-op.
        """
        # In ROS 2, node discovery is handled by DDS and the "zombie" node issue
        # from ROS 1's master is not present in the same way.
        return True

    def get_ros_master_uri(self):
        """ROS_MASTER_URI is a ROS 1 concept. This returns the ROS_DOMAIN_ID for ROS 2."""
        try:
            command = 'echo $ROS_DOMAIN_ID'
            result = subprocess.run(
                ['bash', '-c', command],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
                timeout=SUBPROCESS_TIMEOUT
            )
            domain_id = result.stdout.strip()
            return f"ROS_DOMAIN_ID: {domain_id}" if domain_id else "ROS_DOMAIN_ID: Not Set (default 0)"
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return "Unknown"

    def start_roscore(self, ssh_connection, emit_callback):
        """
        In ROS 2, there is no roscore. This function can be used to start a base launch file.
        For now, it runs the ROS 2 daemon, which is usually not necessary to start manually.
        """
        if not ssh_connection.is_connected():
            return False
        
        # The 'ros2 daemon' is the closest equivalent, but it starts automatically.
        # This command will just ensure it's running.
        command = self.ros_source + "ros2 daemon start"
        
        thread = threading.Thread(
            target=ssh_connection.execute_command_disp_terminal, 
            args=(command, emit_callback)
        )
        thread.daemon = True
        thread.start()
        return True

    def stop_roscore(self, ssh_connection):
        """Stops the ROS 2 daemon. This is generally not needed."""
        if not ssh_connection.is_connected():
            return False
            
        # `ros2 daemon stop` will stop the background daemon process.
        # This does not stop running ROS nodes.
        command = self.ros_source + "ros2 daemon stop"
        
        ssh_connection.execute_command(command)
        return True

    def get_node_list(self, keywords=None):
        """Get list of ROS 2 nodes, optionally filtered by keywords"""
        try:
            command = self.ros_source + "ros2 node list"
            
            if keywords and len(keywords) > 0:
                keyword_patterns = "|".join(re.escape(str(kw.name)) for kw in keywords)
                command += f" | grep -E '{keyword_patterns}'"
                
            response = subprocess.run(
                ['bash', '-c', command],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
                timeout=SUBPROCESS_TIMEOUT
            )
            
            return response.stdout.splitlines()
        except subprocess.CalledProcessError:
            return ["empty"]

    def get_topic_list(self, keywords=None):
        """Get list of ROS 2 topics with their types, optionally filtered by keywords"""
        try:
            # Get topics with their types
            command = self.ros_source + "ros2 topic list -t"
            
            # Add keyword filtering if provided
            if keywords and len(keywords) > 0:
                keyword_patterns = "|".join(re.escape(str(kw.name)) for kw in keywords)
                command += f" | grep -E '{keyword_patterns}'"
            
            response = subprocess.run(
                ['bash', '-c', command],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
                timeout=SUBPROCESS_TIMEOUT
            )
            
            topic_list = response.stdout.splitlines()
            with app.app_context():
                db.session.query(RosTopicList).delete()
                for count, item in enumerate(topic_list):
                    # item is in format '/topic_name [package/msg/Type]'
                    node_ = RosTopicList(id=count, name=item)
                    db.session.add(node_)
                db.session.commit()
            
            return topic_list
        except subprocess.CalledProcessError:
            with app.app_context():
                db.session.query(RosTopicList).delete()
                topic_ = RosTopicList(id=0, name='empty')
                db.session.add(topic_)
                db.session.commit()
            return ["empty"]
    
    def update_node_database(self, timeout=SUBPROCESS_TIMEOUT):
        """Update the database with current ROS 2 nodes"""
        try:
            with app.app_context():
                keywords_list = RosNodeKeywords.query.all()
                node_list = self.get_node_list(keywords_list if keywords_list else None)
                db.session.query(RosNodeList).delete()
                for count, item in enumerate(node_list):
                    node_ = RosNodeList(id=count, name=item)
                    db.session.add(node_)
                db.session.commit()
            return True
        except Exception as e:
            with app.app_context():
                db.session.query(RosNodeList).delete()
                node_ = RosNodeList(id=0, name='error: ' + str(e))
                db.session.add(node_)
                db.session.commit()
            return False

    def update_topic_database(self, timeout=SUBPROCESS_TIMEOUT):
        """Update the database with current ROS 2 topics"""
        try:
            with app.app_context():
                keywords_list = RosTopicKeywords.query.all()
                self.get_topic_list(keywords_list if keywords_list else None)
            return True
        except Exception as e:
            with app.app_context():
                db.session.query(RosTopicList).delete()
                topic_ = RosTopicList(id=0, name='error: ' + str(e))
                db.session.add(topic_)
                db.session.commit()
            return False
    
    def handle_keywords(self, db_type, keywords_string=None):
        """Process and add keywords to the database"""
        if not keywords_string or not keywords_string.strip():
            return
        
        with app.app_context():
            if db_type == 'node':
                kw_type = RosNodeKeywords
            elif db_type == 'topic':
                kw_type = RosTopicKeywords
            else:
                return

            existing_keywords = {kw.name for kw in kw_type.query.all()}
            max_id = db.session.query(func.max(kw_type.id)).scalar() or -1
            
            for keyword in keywords_string.split(','):
                keyword_stripped = keyword.strip()
                if keyword_stripped and keyword_stripped not in existing_keywords:
                    max_id += 1
                    keyword_ = kw_type(id=max_id, name=keyword_stripped)
                    db.session.add(keyword_)
                    existing_keywords.add(keyword_stripped)
            
            db.session.commit()

    def remove_all_keywords(self, db_type):
        with app.app_context():
            if db_type == 'node':
                db.session.query(RosNodeKeywords).delete()
                db.session.commit()
                self.update_node_database()
            elif db_type == 'topic':
                db.session.query(RosTopicKeywords).delete()
                db.session.commit()
                self.update_topic_database()
    
    def remove_single_keyword(self, db_type, keyword_id=None):
        if keyword_id is not None:
            with app.app_context():
                kw_type = None
                update_db_func = None
                if db_type == 'node':
                    kw_type = RosNodeKeywords
                    update_db_func = self.update_node_database
                elif db_type == 'topic':
                    kw_type = RosTopicKeywords
                    update_db_func = self.update_topic_database
                
                if kw_type:
                    db.session.query(kw_type).filter(kw_type.id == keyword_id).delete()
                    # Re-index remaining entries
                    remaining_entries = db.session.query(kw_type).order_by(kw_type.id).all()
                    for index, entry in enumerate(remaining_entries):
                        entry.id = index
                    db.session.commit()
                    update_db_func()
        
    def get_launch_files(self, ssh_connection, folder_path):
        """Get list of launch files (.py, .xml, .yaml) from specified folder"""
        if not ssh_connection.is_connected():
            return []
            
        # Look for common ROS 2 launch file extensions
        command = f"ls -1 {folder_path} | grep -E '\\.launch(\\.(py|xml|yaml))?$'"
        response, error = ssh_connection.execute_command(command, wait=True)
        
        if not response:
            return []
            
        return response.splitlines()
    
    def update_launch_database(self, ssh_connection, folder_path):
        """Update database with launch files from the specified folder"""
        launch_files = self.get_launch_files(ssh_connection, folder_path)
        with app.app_context():
            db.session.query(RosLaunchList).delete()
            for count, item in enumerate(launch_files):
                launch_ = RosLaunchList(id=count, folder_dir=folder_path, name=item, pending=0)
                db.session.add(launch_)
            db.session.commit()
        return len(launch_files) > 0

    def start_launch_file(self, launch_id):
        """Signal the ROS 2 node to start a launch file via service call."""
        with app.app_context():
            launch_file = RosLaunchList.query.get(launch_id)
            if launch_file:
                launch_file.pending = 1
                db.session.commit()
        return True
    
    def terminate_launch_file(self, launch_id):
        """Signal the ROS 2 node to stop a launch file via service call."""
        with app.app_context():
            launch_file = RosActiveLaunchList.query.get(launch_id)
            if launch_file:
                launch_file.pending = 1
                db.session.commit()
        return True

    def get_launch_file_content(self, ssh_connection, launch_file):
        """Get the content of a launch file"""
        if not ssh_connection.is_connected():
            return "Error: Not connected to remote server"
            
        command = f"cat {launch_file.folder_dir}{launch_file.name}"
        response, error = ssh_connection.execute_command(command, wait=True)
        
        return response if response else "Error reading file"

    def kill_all_nodes(self, ssh_connection):
        """Kill all running ROS 2 nodes (except daemon)"""
        if not ssh_connection.is_connected():
            return "Error: Not connected to remote server"
        
        # This is a bit brute-force. A more graceful shutdown would use lifecycle transitions.
        command = f"{self.ros_source} ros2 node list | grep -v '/ros2cli_daemon' | xargs -r -I {{}} pkill -f \"__node:={{}}\""
        
        ssh_connection.execute_command(command, wait=False)
        time.sleep(1.0) # Give time for nodes to die
        
        return True
    
    def kill_specific_node(self, ssh_connection, node_name):
        """
        Kill a specific ROS 2 node.
        Note: This is difficult without PID. The most reliable way is to kill the launch file.
        This uses pkill which can be risky if node names are substrings of other processes.
        A better approach would be to use lifecycle management.
        """
        if not ssh_connection.is_connected():
            return False
            
        # The `pkill -f` command searches the full command line for the pattern.
        # ROS 2 nodes are often run with `__node:=<node_name>`, which makes this somewhat reliable.
        command = f"{self.ros_source} pkill -SIGINT -f \"/__node:={node_name.lstrip('/')}\""
        
        ssh_connection.execute_command(command, wait=False)
        time.sleep(1.0)
        
        return True
    
    def update_ros_master_uri(self, hostname):
        """
        ROS_MASTER_URI is a ROS 1 concept. In ROS 2, networking is configured via
        DDS settings like ROS_DOMAIN_ID. This function is a no-op.
        The environment on the remote machine should be configured correctly.
        """
        return True