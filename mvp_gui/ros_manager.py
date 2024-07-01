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

    def responseGen(self, channel):
        # Small outputs (i.e. 'whoami') can end up running too quickly
        # so we yield channel.recv in both scenarios
        while True:
            if channel.recv_ready():
                yield channel.recv(4096).decode('utf-8')
                continue
            if channel.exit_status_ready():
                yield channel.recv(4096).decode('utf-8')
                break

    def execute_command_disp_terminal(self, command, message_callback):
        time.sleep(1)
        self.transport = self.ssh_client.get_transport()
        self.channel = self.transport.open_session()
        print("TRANSPORT: {}, CHANNEL: {}".format(self.transport, self.channel))
        # # We're not handling stdout & stderr separately
        # self.channel.set_combine_stderr(1)
        # self.channel.exec_command(command)
        # # Command was sent, no longer need stdin
        # self.channel.shutdown_write()

        # for response in self.responseGen(self.channel):
        #     print("Emitting response:", response)  # Debugging print statement
        #     message_callback(response)

        print("CMD: ", command)
        self.channel.exec_command(command)

        # stdout = self.channel.makefile('r')
        # stderr = self.channel.makefile_stderr('r')

        # def read_output(file, output_type):
        #     for line in file:
        #         print(line)
        #         message_callback({'type': output_type, 'data': line})

        # stdout_thread = threading.Thread(target=read_output, args=(stdout, 'stdout'))
        # stderr_thread = threading.Thread(target=read_output, args=(stderr, 'stderr'))

        # stdout_thread.start()
        # stderr_thread.start()

        # stdout_thread.join()
        # stderr_thread.join()

        # stdout.close()
        # stderr.close()
        # self.channel.close()

        def read_stdout():
            while True:
                if self.channel.recv_ready():
                    stdout_data = self.channel.recv(1024).decode('utf-8')
                    print("stdout_data: {}".format(stdout_data))

                    message_callback({'type': 'stdout', 'data': stdout_data})
                if self.channel.exit_status_ready():
                    break

        def read_stderr():
            while True:
                if self.channel.recv_stderr_ready():
                    stderr_data = self.channel.recv_stderr(1024).decode('utf-8')
                    print("stdout_data: {}".format(stderr_data))

                    message_callback({'type': 'stderr', 'data': stderr_data})
                if self.channel.exit_status_ready():
                    break

        stdout_thread = threading.Thread(target=read_stdout)
        stderr_thread = threading.Thread(target=read_stderr)

        stdout_thread.start()
        stderr_thread.start()

        stdout_thread.join()
        stderr_thread.join()

        self.channel.close()


        
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



