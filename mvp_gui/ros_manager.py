import paramiko

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
        # print(success)

    def is_connected(self):
        if self.ssh_client and self.ssh_client.get_transport() and self.ssh_client.get_transport().is_active():
            return True
        return False

    def execute_command(self, command):
        stdin, stdout, stderr = self.ssh_client.exec_command(command)
        output = stdout.read().decode()
        error = stderr.read().decode()
        return output, error

    def close(self):
        if self.ssh_client:
            self.ssh_client.close()
            self.ssh_state = False

# # # Define SSH connection parameters
# hostname = '192.168.0.118'
# username = 'mingxi'
# password = 'qwer1234'
# # port = 22

# # Create SSHConnection instance
# ssh_connection = SSHConnection(hostname, username, password)

# # Connect to SSH server
# ssh_connection.connect()

# # Check if SSH connection is established
# if ssh_connection.is_connected():
#     print("SSH connection is established.")
# else:
#     print("SSH connection is not established.")

# # Close SSH connection
# ssh_connection.close()
