from mvp_gui import app
from flask import render_template
from mvp_gui.models import PowerItems, Vitals, Poses

@app.route("/")
@app.route("/home")
def home_page():
    vitals = Vitals.query.first()
    poses = Poses.query.first()
    return render_template("home.html",vitals=vitals, poses=poses)


@app.route("/power_manager")
def power_manager_page():
    vitals = Vitals.query.first()
    items = PowerItems.query.all()
    return render_template("power_manager.html", items=items, vitals=vitals)

@app.route("/mission")
def mission_page():
    return render_template("mission.html")