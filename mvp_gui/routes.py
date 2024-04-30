from mvp_gui import app
from flask import render_template, request, redirect, url_for
from mvp_gui.models import PowerItems, Vitals, Poses
from mvp_gui import db

@app.route("/")
@app.route("/home")
def home_page():
    vitals = Vitals.query.first()
    poses = Poses.query.first()
    return render_template("home.html",vitals=vitals, poses=poses)


@app.route("/power_manager",  methods=['GET', 'POST']) 
def power_manager_page():
    vitals = Vitals.query.first()
    items = PowerItems.query.all()
    if request.method == 'POST':
        action = request.form.get('action')
        # print(action)
        for item in items:
            # print(str(item.id))
            if action == str(item.id):
                if item.status == 'On':
                    item.status = 'Off'
                    db.session.commit()
                else:
                    item.status = "On"
                    db.session.commit()
                # return redirect(url_for('power_manager_page'))
                return render_template("power_manager.html", items=items, vitals=vitals)
    return render_template("power_manager.html", items=items, vitals=vitals)

@app.route("/mission")
def mission_page():
    return render_template("mission.html")

@app.route("/monitor")
def monitor_page():
    return render_template("monitor.html")

