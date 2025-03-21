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
                message_callback({'type': 'stdout', 'data': stdout_data})

            if stderr.channel.recv_stderr_ready():
                stderr_data = stderr.channel.recv_stderr(1024).decode('utf-8')
                message_callback({'type': 'stderr', 'data': stderr_data})
            
            # Check if command has finished executing
            if stdout.channel.exit_status_ready() and not stdout.channel.recv_ready():
                break
            
            time.sleep(0.1)

    def kill_terminal_session(self, id):
        if id != None:
            kill_shell_command = f"kill -SIGHUP {id} & sed -i '/{id}/d' /tmp/ros_launch_pid.txt"
            self.ssh_client.exec_command(kill_shell_command)

    def close(self):
        if self.ssh_client:
            self.ssh_client.close()
            self.ssh_state = False



