import time
import numpy as np
import threading
from datetime import datetime
from mvp_gui import *

def update_load():
    with app.app_context():
        while True:
            time.sleep(1)
            turbo.push(turbo.replace(render_template("tables/health_table.html"), 'power_health'))
            turbo.push(turbo.replace(render_template("tables/pose_table.html"), 'pose_info'))
            turbo.push(turbo.replace(render_template("tables/power_manager_table.html"), 'power_manager'))
            turbo.push(turbo.replace(render_template("tables/waypoints_table.html"), 'mission_waypoints'))




# def shutdown_node():    
#     rospy.loginfo("Shutting down subscriber!")
#     rospy.signal_shutdown("Shutting down subscriber!")

# thread
update_t = threading.Thread(target=update_load)
update_t.daemon = True
update_t.start()

# rospy.init_node('get_gt', anonymous=True, disable_signals=True)
# pose_t = threading.Thread(target=update_pose)
# pose_t.daemon = True
# pose_t.start()

# rospy.on_shutdown(shutdown_node)