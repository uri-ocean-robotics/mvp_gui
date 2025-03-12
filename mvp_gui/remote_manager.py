from mvp_gui import db, yaml_config, env, SUBPROCESS_TIMEOUT, ROS_PORT
from mvp_gui.models import *
import subprocess
import rosnode
import rosgraph
import threading
import time
from datetime import datetime
from sqlalchemy import func

class RemoteManager:
    """Manages remote ROS system operations and state"""
    def __init__(self):
        self.roslaunch_folder = yaml_config['roslaunch_folder']
        self.ros_source = yaml_config['ros_source_base']
        self.threads = []
        self.launch_files = []

    def check_roscore_status(self):
        """Check if roscore is running by looking for /rosout node"""
        node_command = 'rosnode list'
        node_name = '/rosout'
        try:
            result = subprocess.run(
                ['bash', '-c', node_command], 
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
    
    def check_node_status(self, node_name):
        """Check if a specific node is running"""
        try:
            result = subprocess.run(
                ['bash', '-c', 'rosnode list'], 
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

    def cleanup_dead_nodes(self):
        """Clean up dead ROS nodes"""
        try:
            _, unpinged = rosnode.rosnode_ping_all()    
            if unpinged:
                master = rosgraph.Master("")
                rosnode.cleanup_master_blacklist(master, unpinged)
            return True
        except rosnode.ROSNodeIOException:
            return False

    def get_ros_master_uri(self):
        """Get the current ROS_MASTER_URI"""
        try:
            result = subprocess.run(
                ['bash', '-c', 'echo $ROS_MASTER_URI'], 
                env=env, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                text=True, 
                check=True, 
                timeout=SUBPROCESS_TIMEOUT
            )
            return result.stdout.strip()
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return "Unknown"

    def start_roscore(self, ssh_connection, emit_callback):
        """Start roscore via SSH"""
        if not ssh_connection.is_connected():
            return False
            
        command = self.ros_source + "roscore"
        
        thread = threading.Thread(
            target=ssh_connection.execute_command_disp_terminal, 
            args=(command, emit_callback)
        )
        thread.daemon = True
        thread.start()
        return True

    def stop_roscore(self, ssh_connection):
        """Stop roscore and related processes"""
        if not ssh_connection.is_connected():
            return False
            
        command = self.ros_source + "killall -9 rosmaster && killall -9 roscore && killall -9 rviz"
        
        ssh_connection.execute_command(command)
        return True

    def get_node_list(self, keywords=None):
        """Get list of ROS nodes, optionally filtered by keywords"""
        try:
            command = "rosnode list"
            
            # Add keyword filtering if provided
            if keywords and len(keywords) > 0:
                command += " | grep '"
                command += "\\|".join(str(kw.name) for kw in keywords)
                command += "'"
                
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
        """Get list of ROS topics, optionally filtered by keywords"""
        try:
            command = "rostopic list"
            
            # Add keyword filtering if provided
            if keywords and len(keywords) > 0:
                command += " | grep '"
                command += "\\|".join(str(kw.name) for kw in keywords)
                command += "'"
            
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
            count = 0
            db.session.query(RosTopicList).delete()
            for item in topic_list:
                node_ = RosTopicList(id=count, name=item)
                db.session.add(node_)
                count = count + 1                
            db.session.commit()
            
            return topic_list
        except subprocess.CalledProcessError:
            db.session.query(RosTopicList).delete()
            topic_ = RosTopicList(id=0, name='empty')
            db.session.add(topic_)
            db.session.commit()
            return ["empty"]
        
    def update_node_database(self, timeout=SUBPROCESS_TIMEOUT):
        """Update the database with current ROS nodes"""
        try:
            # Get all keywords for filtering
            keywords_list = RosNodeKeywords.query.all()
            
            # Get nodes with optional keyword filtering
            node_list = self.get_node_list(keywords_list if keywords_list else None)
            
            # Clear existing nodes
            db.session.query(RosNodeList).delete()
            
            # Add new nodes to database
            for count, item in enumerate(node_list):
                node_ = RosNodeList(id=count, name=item)
                db.session.add(node_)
                
            db.session.commit()
            return True
        except Exception as e:
            # Handle errors by adding an empty node
            db.session.query(RosNodeList).delete()
            node_ = RosNodeList(id=0, name='error: ' + str(e))
            db.session.add(node_)
            db.session.commit()
            return False

    def update_topic_database(self, timeout=SUBPROCESS_TIMEOUT):
        """Update the database with current ROS topics"""
        try:
            # Get all keywords for filtering
            keywords_list = RosTopicKeywords.query.all()
            
            # Get topics with optional keyword filtering
            topic_list = self.get_topic_list(keywords_list if keywords_list else None)
            
            # Clear existing topics
            db.session.query(RosTopicList).delete()
            
            # Add new topics to database
            for count, item in enumerate(topic_list):
                topic_ = RosTopicList(id=count, name=item)
                db.session.add(topic_)
                
            db.session.commit()
            return True
        except Exception as e:
            # Handle errors by adding an empty node
            db.session.query(RosTopicList).delete()
            topic_ = RosTopicList(id=0, name='error: ' + str(e))
            db.session.add(topic_)
            db.session.commit()
            return False
    
    def handle_keywords(self, db_type, keywords_string=None):
        """Process and add keywords to the database"""
        if not keywords_string.strip():
            return
        
        if db_type == 'node':
            kw_type = RosNodeKeywords
        elif db_type == 'topic':
            kw_type = RosTopicKeywords

        count = len(kw_type.query.all())
        
        for keyword in keywords_string.split(','):
            if keyword.strip():
                keyword_ = kw_type(id=count, name=keyword.strip())
                db.session.add(keyword_)
                count += 1
        
        # Remove duplicates
        subquery = db.session.query(
            kw_type.id
        ).filter(
            kw_type.id.notin_(
                db.session.query(func.min(kw_type.id)).group_by(kw_type.name)
            )
        )
        
        db.session.query(kw_type).filter(kw_type.id.in_(subquery)).delete(synchronize_session=False)
        
        # Reindex remaining entries
        remaining_entries = db.session.query(kw_type).order_by(kw_type.id).all()
        for index, entry in enumerate(remaining_entries):
            entry.id = index
            
        db.session.commit()

    def remove_all_keywords(self, db_type):
        if db_type == 'node':
            db.session.query(RosNodeKeywords).delete()
            db.session.commit()
            self.update_node_database()
        elif db_type == 'topic':
            db.session.query(RosTopicKeywords).delete()
            db.session.commit()
            self.update_topic_database()
    
    def remove_single_keyword(self, db_type, keyword=None):
        if keyword != None:
            if db_type == 'node':
                db.session.query(RosNodeKeywords).filter(RosNodeKeywords.id == keyword).delete()
                db.session.query(RosNodeKeywords).filter(RosNodeKeywords.id > int(keyword)).update(
                    {RosNodeKeywords.id: RosNodeKeywords.id - 1}
                )
                db.session.commit()
                self.update_node_database()
            elif db_type == 'topic':
                db.session.query(RosTopicKeywords).filter(RosTopicKeywords.id == keyword).delete()
                db.session.query(RosTopicKeywords).filter(RosTopicKeywords.id > int(keyword)).update(
                    {RosTopicKeywords.id: RosTopicKeywords.id - 1}
                )
                db.session.commit()
                self.update_node_database()

        
    def get_launch_files(self, ssh_connection, folder_path):
        """Get list of launch files from specified folder"""
        if not ssh_connection.is_connected():
            return []
            
        command = f"ls {folder_path}"
        response = ssh_connection.execute_command(command, wait=True)
        
        if not response or not response[0]:
            return []
            
        return [item for item in response[0].splitlines() if item.endswith(".launch")]
    
    def update_launch_database(self, ssh_connection, folder_path):
        """Update database with launch files from the specified folder"""
        launch_files = self.get_launch_files(ssh_connection, folder_path)
        
        # Clear existing launch files
        db.session.query(RosLaunchList).delete()
        
        # Add new launch files to database
        for count, item in enumerate(launch_files):
            launch_ = RosLaunchList(id=count, folder_dir=folder_path, name=item)
            db.session.add(launch_)
            
        db.session.commit()
        return len(launch_files) > 0

    def start_launch_file(self, ssh_connection, launch_file, emit_callback):
        """Start a launch file via SSH and track it in the database"""
        if not ssh_connection.is_connected():
            return None
                    
        # Construct commands
        base_command = f"{self.ros_source} roslaunch {launch_file.folder_dir}{launch_file.name}"
        # base_command = f"roslaunch {launch_file.folder_dir}{launch_file.name}"
        command_with_pid = f"bash -c '( {base_command} & echo $! >> /tmp/ros_launch_pid.txt; wait $!)'"
        
        # Start thread to execute command
        thread = threading.Thread(
            target=ssh_connection.execute_command_disp_terminal, 
            args=(command_with_pid, emit_callback)
        )
        thread.daemon = True
        thread.start()
        thread_id = threading.get_native_id()
        
        # Wait for PID file to be updated
        time.sleep(2.0)
        pid_val, _ = ssh_connection.execute_command('tail -1 /tmp/ros_launch_pid.txt')
        
        # Add thread to database
        thread_id_db = db.session.query(RosThreadList).count()
        thread_entry = RosThreadList(
            id=thread_id_db, 
            name=launch_file.name, 
            thread=thread_id, 
            pid=pid_val.strip(),
            start_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        db.session.add(thread_entry)
        db.session.commit()
        
        return True
    
    def terminate_thread(self, ssh_connection, thread_entry):
        """Terminate a running thread/process"""
        if not ssh_connection.is_connected():
            return False
            
        # Kill the process
        stdout, stderr = ssh_connection.kill_terminal_session(thread_entry.pid)
        print(stdout)
        # Remove from database and reindex
        db.session.query(RosThreadList).filter(RosThreadList.id == thread_entry.id).delete()
        db.session.query(RosThreadList).filter(RosThreadList.id > int(thread_entry.id)).update(
            {RosThreadList.id: RosThreadList.id - 1}
        )
        db.session.commit()
        
        return True
    
    def get_launch_file_content(self, ssh_connection, launch_file):
        """Get the content of a launch file"""
        if not ssh_connection.is_connected():
            return "Error: Not connected to remote server"
            
        command = f"cat {launch_file.folder_dir}{launch_file.name}"
        response = ssh_connection.execute_command(command, wait=True)
        
        return response[0] if response and response[0] else "Error reading file"

    def kill_all_nodes(self, ssh_connection):
        """Kill all running ROS nodes"""
        if not ssh_connection.is_connected():
            return False
            
        command = f"{self.ros_source}rosnode kill -a"
        
        ssh_connection.execute_command(command, wait=False)
        self.cleanup_dead_nodes()
        
        return True
    
    def kill_specific_node(self, ssh_connection, node_name):
        """Kill a specific ROS node"""
        if not ssh_connection.is_connected():
            return False
            
        command = f"{self.ros_source}rosnode kill {node_name}"
        
        ssh_connection.execute_command(command, wait=False)
        self.cleanup_dead_nodes()
        
        return True
    
    def update_ros_master_uri(self, hostname):
        """Update ROS_MASTER_URI in the environment"""
        if not hostname:
            return False
            
        ros_master_uri = f'http://{hostname}:{ROS_PORT}/'
        env['ROS_MASTER_URI'] = ros_master_uri
        
        return True