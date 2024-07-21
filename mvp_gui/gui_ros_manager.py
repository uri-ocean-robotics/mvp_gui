import subprocess
import time
import threading
import os
import signal

# Initialize global variables
ros_process = None
process_lock_gui = threading.Lock()

def start_ros_process(env):
    global ros_process
    with process_lock_gui:
        if ros_process is None:
            ros_process = subprocess.Popen(
                ['bash', '-c', 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && python3 ./mvp_gui/gui_ros.py'],
                # stdout=subprocess.DEVNULL,
                # stderr=subprocess.DEVNULL, 
                env=env,
                preexec_fn=os.setsid
            )

def stop_ros_process(env):
    global ros_process
    print("TO STOP: ", ros_process)
    with process_lock_gui:
        if ros_process:
            node_name = '/mvp_gui_node'
            kill_rosnode(node_name, env)  # Explicitly kill the node
            os.killpg(os.getpgid(ros_process.pid), signal.SIGTERM)
            ros_process.wait()  # Wait for the process to terminate
            ros_process = None


def kill_rosnode(node_name, env, timeout=5):
    try:
        command = f'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosnode kill {node_name}'
        result = subprocess.run(['bash', '-c', command], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        print("KILLING ROSNODE: ", result.stdout)
        if result.returncode != 0:
            print(f"Kill command failed with error: {result.stderr}")
        else:
            print(f"Kill command successful for node: {node_name}")
    except subprocess.TimeoutExpired:
        print(f"Kill command timed out for node: {node_name}")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while killing ROS node: {e.stderr}")

