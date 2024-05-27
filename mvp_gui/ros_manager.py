import paramiko
import os 

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
        # self.ssh_client.connect(self.hostname, username = self.username, password = self.password)
        try:
            self.ssh_client.connect(self.hostname, username=self.username, password=self.password)
            self.ssh_state = True  # Update connection state
            print("SSH connection is established.")
            return True
        except paramiko.AuthenticationException:
            print("Authentication failed. Please check your credentials.")
            return False
        except paramiko.SSHException as ssh_exception:
            print(f"SSH connection failed: {ssh_exception}")
            return False
        except Exception as e:
            print(f"An error occurred: {e}")
            return False

    def is_connected(self):
        if self.ssh_client and self.ssh_client.get_transport() and self.ssh_client.get_transport().is_active():
            return True
        return False

    def execute_command(self, command, wait=True):
        stdin, stdout, stderr = self.ssh_client.exec_command(command)
        if wait:
            output = stdout.read().decode()
            error = stderr.read().decode()
            return output, error
        else:
            # If we don't want to wait, we return immediately.
            return None, None
        
    def execute_command_with_x11(self, command):
        # Open a new session with X11 forwarding
        transport = self.ssh_client.get_transport()
        session = transport.open_session()

        # Request X11 forwarding
        session.get_pty()
        session.request_x11()

        # Execute the command with DISPLAY set
        # session.exec_command(f'export DISPLAY=$DISPLAY; {command}')
        session.exec_command(f'export DISPLAY={os.getenv("DISPLAY")}; {command}')

        # Execute the command
        # session.exec_command(command)

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



