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

# thread
update_t = threading.Thread(target=update_load)
update_t.daemon = True
update_t.start()


