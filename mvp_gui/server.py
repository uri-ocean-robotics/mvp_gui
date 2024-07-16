import socket
from mvp_gui import app, socketio
from mvp_gui.ros_manager import *

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

if __name__ == '__main__':
    host_ip = get_ip()
    #host_ip = '192.168.2.200'
    app.config['HOST_IP'] = host_ip
    # app.run(debug=False, host=host_ip, port=5000)
    socketio.run(app, debug=False, host=host_ip, port=5000)


