import paramiko
import time 
import sys
import socket
from flask_socketio import emit
import threading
import select

class SSHConnection:
    def __init__(self, hostname, username, password):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.ssh_client = None  
        self.ssh_state = False

    def connect(self):
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.ssh_client.connect(self.hostname, username=self.username, password=self.password, timeout=5)
            self.ssh_state = True  # Update connection state
            print("SSH connection is established.")
            return True
        except paramiko.AuthenticationException:
            print("Authentication failed. Please check your credentials.")
            return False
        except paramiko.SSHException as ssh_exception:
            print(f"SSH connection failed: {ssh_exception}")
            return False
        except (paramiko.SSHException, socket.error) as se:        
            print(f"SSH connection failed: {se}")
            return False
        except Exception as e:
            print(f"An error occurred: {e}")
            return False

    def is_connected(self):
        if self.ssh_client and self.ssh_client.get_transport() and self.ssh_client.get_transport().is_active():
            return True
        return False

    def execute_command(self, command, wait=True, timeout=None):
        stdin, stdout, stderr = self.ssh_client.exec_command(command)
        if wait:
            output = stdout.read().decode()
            error = stderr.read().decode()
            return output, error
        elif timeout != None:
            start_time = time.time()
            while not stdout.channel.eof_received:
                time.sleep(1)
                if time.time() > start_time + timeout:
                    stdout.channel.close()
                    return "empty", None
            output = stdout.read().decode()
            error = stderr.read().decode()
            return output, error
        else:
            # If we don't want to wait, we return immediately.
            return None, None


    def execute_command_disp_terminal(self, command, message_callback):
        
        stdin, stdout, stderr = self.ssh_client.exec_command(command, get_pty=True)
        stdout.channel.setblocking(0)  # Set stdout channel to non-blocking mode
        stderr.channel.setblocking(0)  # Set stderr channel to non-blocking mode
    
        while True:
            if stdout.channel.recv_ready():
                stdout_data = stdout.channel.recv(1024).decode('utf-8')
                # print("stdout_data: {}".format(stdout_data))
                message_callback({'type': 'stdout', 'data': stdout_data})

            if stderr.channel.recv_stderr_ready():
                stderr_data = stderr.channel.recv_stderr(1024).decode('utf-8')
                # print("stdout_data: {}".format(stderr_data))
                message_callback({'type': 'stderr', 'data': stderr_data})
            
            # Check if command has finished executing
            if stdout.channel.exit_status_ready() and not stdout.channel.recv_ready():
                break
            
            time.sleep(0.1)

    # def kill_terminal_session(self):
    #     # Command to find the parent process (shell) PID of the roslaunch process
    #     find_shell_pid_command = "ps -o ppid= -p $(cat /tmp/ros_launch_pid.txt)"
    #     stdin, stdout, stderr = self.ssh_client.exec_command(find_shell_pid_command)
        
    #     shell_pid = stdout.read().decode('utf-8').strip()
        
    #     if shell_pid:
    #         print(f"Killing shell with PID: {shell_pid}")
    #         # Command to kill the shell and all its child processes
    #         kill_shell_command = f"kill -SIGHUP {shell_pid}"
    #         self.ssh_client.exec_command(kill_shell_command)
    #     else:
    #         print("Shell PID not found.")

    def kill_terminal_session(self, id):
        kill_shell_command = f"kill -SIGHUP {id} & sed -i '/{id}/d' /tmp/ros_launch_pid.txt"
        self.ssh_client.exec_command(kill_shell_command)

        # Command to find the parent process (shell) PID of the roslaunch process
        # ppid = "cat /tmp/ros_launch_pid.txt"
        # stdin, stdout, stderr = self.ssh_client.exec_command(ppid)
        # print(stdout)
        # find_shell_pid_command = f"sed -n '{id}p' /tmp/ros_launch_pid.txt"
        # find_shell_pid_command = "ps -o ppid= -p $(cat /tmp/ros_launch_pid.txt)"

        # stdin, stdout, stderr = self.ssh_client.exec_command(find_shell_pid_command)
        
        # shell_pid = stdout.read().decode('utf-8').strip()
        
        # if shell_pid:
        #     print(f"Killing shell with PID: {shell_pid}")
        #     # Command to kill the shell and all its child processes
        #     kill_shell_command = f"kill -SIGHUP {shell_pid}"
        #     self.ssh_client.exec_command(kill_shell_command)
        #     self.remove_terminated_pid_from_file(id - 1)
        # else:
        #     print("Shell PID not found.")



    # def remove_terminated_pid_from_file(self, pid_index):
    #     # Read the contents of the PID file line by line
    #     read_pid_command = "cat /tmp/ros_launch_pid.txt"
    #     stdin, stdout, stderr = self.ssh_client.exec_command(read_pid_command)
    #     pids = stdout.read().decode('utf-8').strip().splitlines()

    #     # Ensure the index is within the valid range
    #     if 0 <= pid_index < len(pids):
    #         # Remove the PID at the specified index
    #         del pids[pid_index]

    #         # Write the remaining PIDs back to the file, each on a new line
    #         if pids:
    #             new_content = '\n'.join(pids)
    #             update_pid_command = f"echo -e '{new_content}' > /tmp/ros_launch_pid.txt"
    #         else:
    #             # If no PIDs remain, truncate the file
    #             update_pid_command = "truncate -s 0 /tmp/ros_launch_pid.txt"
            
    #         self.ssh_client.exec_command(update_pid_command)

        
    def execute_command_with_xvfb(self, command):
        # Open a new session with X11 forwarding
        transport = self.ssh_client.get_transport()
        session = transport.open_session()

        # Request X11 forwarding
        session.get_pty()
        session.request_x11()

        # Execute the command with DISPLAY set
        # session.exec_command(f'export DISPLAY=$DISPLAY; {command}')
        # session.exec_command(f'export DISPLAY={os.getenv("DISPLAY")}; {command}')

        ros_master_uri = 'http://' + self.hostname  + ':11311/'
        ros_hostname = self.hostname 
        ros_ip = self.hostname

        # Start Xvfb and set DISPLAY variable
        full_command = (
            'if pgrep -x "Xvfb" > /dev/null; then pkill -x "Xvfb"; fi; '
            'rm -f /tmp/.X99-lock; '
            'Xvfb :99 -screen 0 1200x800x24 & '
            'export DISPLAY=:99; '
            f'export ROS_MASTER_URI={ros_master_uri}; '
            f'export ROS_HOSTNAME={ros_hostname}; '
            f'export ROS_IP={ros_ip}; '
            f'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && {command}'
        )

        # Execute the command
        session.exec_command(full_command)

        # Get the output and error streams
        stdout = session.makefile('r', 2048)
        stderr = session.makefile_stderr('r', 2048)

        # Print the output and error
        for line in stdout:
            print(line, end="")

        for line in stderr:
            print(line, end="")

        # Close the session
        # session.close()

    def close(self):
        if self.ssh_client:
            self.ssh_client.close()
            self.ssh_state = False



