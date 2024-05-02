from mvp_gui import app, turbo
from flask import render_template, request, redirect, url_for
from mvp_gui.models import PowerItems, Vitals, Poses
from mvp_gui import db
import time
import threading
import random
from datetime import datetime

counter_fr = 0
# flask turbo setup
@app.before_request
def before_first_request():
    global counter_fr
    print("INIT: ", counter_fr)
    if counter_fr == 0 :
        update_t = threading.Thread(target=update_load).start()
        random_t = threading.Thread(target=random_pose).start()
    counter_fr += 1    

def random_pose():
    with app.app_context():
        while True:
            time.sleep(1)
            print("random_pose: ", datetime.now())
            vitals = Vitals.query.first()
            poses = Poses.query.first()
            poses.roll = random.random()
            poses.pitch = random.random()
            poses.yaw = random.random()
            poses.x = random.random()
            poses.y = random.random()
            poses.z = random.random()

            poses.u = random.random()
            poses.v = random.random()   
            poses.w = random.random()
            poses.p = random.random()
            poses.q = random.random()
            poses.r = random.random()

            poses.lat = random.random()
            poses.lon = random.random()

            vitals.voltage = random.random()
            vitals.current = random.random()
            db.session.commit()  
    
def update_load():
    with app.app_context():
        while True:
            time.sleep(1)
            turbo.push(turbo.replace(render_template("tables/health_table.html"), 'power_health'))
            turbo.push(turbo.replace(render_template("tables/pose_table.html"), 'pose_info'))
            turbo.push(turbo.replace(render_template("tables/power_manager_table.html"), 'power_manager'))
            # turbo.push(turbo.replace(render_template("tables/map_table.html"), 'map'))

@app.context_processor
def inject_load():
    vitals = Vitals.query.first()
    poses = Poses.query.first()
    items = PowerItems.query.all()
    return {'vitals': vitals, 'poses': poses, 'items': items}


# routes
@app.route("/", methods=['GET', 'POST'])
def home_page():
    vitals = Vitals.query.first()
    poses = Poses.query.first()
    return render_template("home.html", vitals=vitals, poses=poses)


@app.route("/power_manager",  methods=['GET', 'POST']) 
def power_manager_page():
    vitals = Vitals.query.first()
    items = PowerItems.query.all()
    # print("power manager")
    if request.method == 'POST':
        action = request.form.get('action')
        for item in items:
            # print(str(item.id))
            if action == str(item.id):
                if item.status == 'On':
                    item.status = 'Off'
                    db.session.commit()
                    ##call rosservice
                else:
                    item.status = "On"
                    db.session.commit()
                    ##call rosservice
            # return render_template("power_manager.html", items=items, vitals=vitals)
    return render_template("power_manager.html", items=items, vitals=vitals)

@app.route("/map", methods=['GET', 'POST'])
def map_page():
    print("map lnk")
    return render_template("map.html")

@app.route("/mission")
def mission_page():
    return render_template("mission.html")

@app.route("/monitor")
def monitor_page():
    return render_template("monitor.html")






