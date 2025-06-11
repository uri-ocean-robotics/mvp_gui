import subprocess
import threading
import os
import signal
import time
import tempfile

# Initialize global variables
# In ROS 2, nodes are often namespaced. The actual name might be different.
# The gui_ros.py will create a node named 'mvp_gui_node'.
gui_node = '/mvp_gui_node' # This might need adjustment based on how the node is launched.
ros_process = None
process_lock_gui = threading.Lock()
TIMEOUT = 2
ROS_PID_FILE = os.path.join(tempfile.gettempdir(), "ros_gui_pid.txt")

def start_ros_process(env):
    """Launch ROS 2 GUI process in a separate terminal that will persist if parent script terminates."""
    global ros_process
    
    with process_lock_gui:
        if ros_process is None:
            # First, check if there's an existing PID file from a previous run
            cleanup_existing_process()
            
            # This new command is more robust and keeps the terminal open for debugging.
            # It runs the python script as a module to ensure imports work correctly.
            ros_cmd = (
                'echo "Setting up environment...";'
                # Add current directory to PYTHONPATH to help with local package imports
                'export PYTHONPATH=$(pwd):$PYTHONPATH;'
                # Source ROS 2 environments
                'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash;'
                'echo "Launching ROS node...";'
                # Use -m to run as a module (solves import issues) and -u for unbuffered output
                'python3 -u -m mvp_gui.gui_ros & '
                'GUI_PID=$!;'
                'echo "ROS GUI process started with PID: $GUI_PID";'
                # Store the PID
                'echo $GUI_PID > ' + ROS_PID_FILE + '; '
                # Wait for the specific background process to finish
                'wait $GUI_PID; '
                'EXIT_CODE=$?;'
                'echo "ROS GUI process exited with code: $EXIT_CODE";'
                # This is the key for debugging: keep the terminal open to see any errors.
                'echo "Press Enter to close this terminal...";'
                'read'
            )
            
            ros_process = subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c', ros_cmd],
                env=env
            )
            print(f"Started ROS 2 GUI process (Terminal PID: {ros_process.pid})")
            
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

# The `rosnode kill` command is from ROS 1 and is not directly available in ROS 2.
# We will rely on process ID (PID) killing, which is more robust.
# The original kill_rosnode function is removed.

def stop_ros_process(env):
    """Stop the ROS GUI process by terminating its process ID."""
    global ros_process
    global gui_node
    
    with process_lock_gui:
        # Get the PID from the file, as this is the most reliable source
        ros_pid = get_ros_pid()
        if ros_pid:
            try:
                # Terminate the actual process using its PID
                print(f"Killing gui_node process with PID: {ros_pid}")
                os.kill(ros_pid, signal.SIGTERM)
                
                # Give it a moment to terminate
                time.sleep(0.5)
                
                # Verify termination and use SIGKILL if necessary
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