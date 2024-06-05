import subprocess
import signal
import os
import threading

# Global state for processes
flask_process = None
process_lock = threading.Lock()
project_path = os.getcwd()
env = os.environ.copy()
env['PYTHONPATH'] = project_path

def start_processes():
    global flask_process
    with process_lock:
        if flask_process is None:
            flask_process = subprocess.Popen(['bash', '-c', 'python3 ./mvp_gui/server.py'], 
                                            env=env, 
                                            preexec_fn=os.setsid)

def stop_processes():
    global flask_process
    with process_lock:
        if flask_process:
            os.killpg(os.getpgid(flask_process.pid), signal.SIGTERM)
            # kill_rosnode('/mvp_gui_node')

# def check_rosnode_exists(node_name):
#     try:
#         # Source the ROS environment before running rosnode list
#         command = 'source /opt/ros/noetic/setup.bash && rosnode list'
#         result = subprocess.run(['bash', '-c', command], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
#         nodes = result.stdout.splitlines()
#         return node_name in nodes
            
#     except subprocess.CalledProcessError as e:
#         print(f"An error occurred while listing ROS nodes: {e.stderr}")
#         return False

# def kill_rosnode(node_name, timeout=5):
#     try:
#         if check_rosnode_exists(node_name):
#             command = f'source /opt/ros/noetic/setup.bash && rosnode kill {node_name}'
#             result = subprocess.run(['bash', '-c', command], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
#             if result.returncode != 0:
#                 print(f"Kill command failed with error: {result.stderr}")
#             else:
#                 print(f"Kill command successful for node: {node_name}")
#     except subprocess.TimeoutExpired:
#         print(f"Kill command timed out for node: {node_name}")
#     except subprocess.CalledProcessError as e:
#         print(f"An error occurred while killing ROS node: {e.stderr}")

if __name__ == "__main__":
    start_processes()
    try:
        while True:
            pass
    except KeyboardInterrupt:
        stop_processes()
