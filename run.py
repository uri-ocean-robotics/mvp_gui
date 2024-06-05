import subprocess
import signal
import os
import threading

from mvp_gui.models import *
from mvp_gui.ros_manager import SSHConnection

# Global state for processes
flask_process = None
process_lock_server = threading.Lock()
project_path = os.getcwd()
env = os.environ.copy()
env['PYTHONPATH'] = project_path

INIT_CNT = 1

def start_processes():
    global flask_process
    with process_lock_server:
        if flask_process is None:
            flask_process = subprocess.Popen(['bash', '-c', 'python3 ./mvp_gui/server.py'], 
                                            env=env, 
                                            preexec_fn=os.setsid)

def stop_processes():
    global flask_process
    with process_lock_server:
        if flask_process:
            os.killpg(os.getpgid(flask_process.pid), signal.SIGTERM)

if __name__ == "__main__":
    start_processes()

    # Clear pervious ROS launchfiles, nodes, topics, topic keywords
    db.session.query(RosLaunchList).delete()
    db.session.query(RosNodeList).delete()
    db.session.query(RosTopicList).delete()
    db.session.query(RosTopicKeywords).delete()
    db.session.commit()
    
    try:
        while True:
            pass
    except KeyboardInterrupt:
        stop_processes()
