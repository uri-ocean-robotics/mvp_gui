import subprocess
import threading
import os
import signal
import time
import tempfile

# Initialize global variables
gui_node = '/mvp_gui_node'
ros_process = None
process_lock_gui = threading.Lock()
TIMEOUT = 2
ROS_PID_FILE = os.path.join(tempfile.gettempdir(), "ros_gui_pid.txt")

def start_ros_process(env):
    """Launch ROS GUI process in a separate terminal that will persist if parent script terminates."""
    global ros_process
    
    with process_lock_gui:
        if ros_process is None:
            # First, check if there's an existing PID file from a previous run
            cleanup_existing_process()
            
            # Command that launches the ROS node and saves its PID to a temp file
            ros_cmd = (
                'source /opt/ros/noetic/setup.bash && '
                'source ~/catkin_ws/devel/setup.bash && '
                'python3 ./mvp_gui/gui_ros.py & '
                'echo $! > ' + ROS_PID_FILE + ' && '
                'wait'  # Wait for background process to complete
            )
            
            ros_process = subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c', ros_cmd],
                env=env
            )
            print(f"Started ROS GUI process (Terminal PID: {ros_process.pid})")
            
            # Give the process time to write the PID file
            time.sleep(1)

def cleanup_existing_process():
    """Check for and clean up any existing ROS process from a previous run."""
    if os.path.exists(ROS_PID_FILE):
        try:
            with open(ROS_PID_FILE, 'r') as f:
                old_pid = int(f.read().strip())
                
            print(f"Found existing ROS process (PID: {old_pid}), attempting to terminate...")
            try:
                os.kill(old_pid, signal.SIGTERM)
                time.sleep(0.5)  # Give it a moment to terminate
                
                # Check if it's still running
                if is_process_running(old_pid):
                    print(f"Process {old_pid} didn't terminate, sending SIGKILL...")
                    os.kill(old_pid, signal.SIGKILL)
            except ProcessLookupError:
                print(f"Process {old_pid} not found, it may have already terminated")
            except Exception as e:
                print(f"Error terminating existing process {old_pid}: {e}")
                
            # Remove the stale PID file regardless of termination success
            os.remove(ROS_PID_FILE)
            print(f"Removed stale PID file")
        except (ValueError, IOError) as e:
            print(f"Error processing existing PID file: {e}")
            # Still try to remove the file if it's corrupt
            try:
                os.remove(ROS_PID_FILE)
            except:
                pass

def is_process_running(pid):
    """Check if a process with the given PID is running."""
    try:
        os.kill(pid, 0)  # Signal 0 doesn't kill but checks if process exists
        return True
    except ProcessLookupError:
        return False
    except Exception:
        return False

def get_ros_pid():
    """Read the actual ROS process PID from the temp file."""
    try:
        if os.path.exists(ROS_PID_FILE):
            with open(ROS_PID_FILE, 'r') as f:
                pid = int(f.read().strip())
                return pid
        return None
    except (ValueError, IOError) as e:
        print(f"Error reading PID file: {e}")
        return None

def kill_rosnode(node_name, env, timeout=TIMEOUT):
    """Kill a specific ROS node using rosnode kill command."""
    try:
        command = (
            'source /opt/ros/noetic/setup.bash && '
            'source ~/catkin_ws/devel/setup.bash && '
            f'rosnode kill {node_name}'
        )
        
        result = subprocess.run(
            ['bash', '-c', command], 
            env=env, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True, 
            timeout=timeout
        )
        
        if result.returncode == 0:
            print(f"Successfully killed ROS node: {node_name}")
            return True
        else:
            print(f"Failed to kill ROS node: {node_name}\nError: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print(f"Timeout while trying to kill ROS node: {node_name}")
        return False
    except Exception as e:
        print(f"Error killing ROS node {node_name}: {str(e)}")
        return False

def stop_ros_process(env):
    """Stop the ROS GUI process and kill associated ROS nodes."""
    global ros_process
    global gui_node
    
    with process_lock_gui:
        # First check if we have a tracked PID file, regardless of ros_process status
        ros_pid = get_ros_pid()
        if ros_pid:
            try:
                # First try to gracefully kill the ROS node
                kill_rosnode(gui_node, env)
                
                # Now terminate the actual process
                print(f"Killing gui_node with PID: {ros_pid}")
                os.kill(ros_pid, signal.SIGTERM)
                
                # Give it a moment to terminate
                time.sleep(0.5)
                
                # Verify termination
                if is_process_running(ros_pid):
                    print(f"Process {ros_pid} didn't terminate gracefully, using SIGKILL...")
                    os.kill(ros_pid, signal.SIGKILL)
            except ProcessLookupError:
                print(f"Process with PID {ros_pid} not found, may have already terminated")
            except Exception as e:
                print(f"Error killing process with PID {ros_pid}: {e}")
            finally:
                ros_process = None
                # Always clean up the PID file
                if os.path.exists(ROS_PID_FILE):
                    os.remove(ROS_PID_FILE)
                    print(f"Removed PID file")
    
    return gui_node
