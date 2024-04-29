from mvp_gui import app
from flask import render_template
from mvp_gui.models import Item

@app.route("/")
@app.route("/home")
def home_page():
    return render_template("home.html")

@app.route("/power_manager")
def power_manager_page():
    items = Item.query.all()
    return render_template("power_manager.html", items=items)
