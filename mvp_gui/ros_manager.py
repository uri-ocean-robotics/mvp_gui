import paramiko
import time

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
        print(self.hostname)
        print(self.username)
        print(self.password)

        self.ssh_client.connect(self.hostname, username = self.username, password = self.password)
        # time.sleep(1)
        # if self.is_connected():
        #     print("SSH connection is established.")
        # else:
        #     print("SSH connection is not established.")

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

    def close(self):
        if self.ssh_client:
            self.ssh_client.close()
            self.ssh_state = False
