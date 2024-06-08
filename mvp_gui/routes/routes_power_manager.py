from mvp_gui import *

@app.route("/power_manager",  methods=['GET', 'POST']) 
def power_manager_page():
    items = PowerItems.query.all()
    # print("power manager")
    if request.method == 'POST':
        if 'switch' in request.form:
            switch_id = request.form['switch']
            item = PowerItems.query.get(switch_id)
            switch_power = RosActions.query.filter_by(action='set_power').first()
            if item.status == '1':
                switch_power.value = item.name + "=" + "false"
                switch_power.pending = 1
                ##call rosservice
            else:
                switch_power.value = item.name + "=" + "true"
                switch_power.pending = 1
            db.session.commit()
            return redirect(url_for('power_manager_page'))
                    ##call rosservice
            # return render_template("power_manager.html", items=items, vitals=vitals)
    return render_template("power_manager.html", items=items, current_page='power_manager')


@app.route('/power_manager/states')
def power_manager_data():
    power_items = PowerItems.query.all()
    power_items_data = [
        {"id": item.id, "name": item.name, "status": item.status}
        for item in power_items
    ]

    return jsonify({"power_items_data": power_items_data})

