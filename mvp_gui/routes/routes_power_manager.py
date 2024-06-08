from mvp_gui import *
from decimal import Decimal

@app.route("/power_manager",  methods=['GET', 'POST']) 
def power_manager_page():
    items = PowerItems.query.all()
    lumen_item =  LedItems.query.first()
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

        elif 'lumen' in request.form:
            lumen_adj  = request.form['lumen']
            if lumen_adj == 'increase':
                delta = 0.1
            else:
                delta = -0.1
            lumen_item.status = lumen_item.status + Decimal(delta)
            if(lumen_item.status > 1.9):
                lumen_item.status = 1.9
            if(lumen_item.status< 1.0):
                lumen_item.status = 1.0
            db.session.commit()
            return redirect(url_for('power_manager_page'))

    return render_template("power_manager.html", items=items, lumen_item = lumen_item, current_page='power_manager')


@app.route('/power_manager/states')
def power_manager_data():
    power_items = PowerItems.query.all()
    power_items_data = [
        {"id": item.id, "name": item.name, "status": item.status}
        for item in power_items
    ]

    return jsonify({"power_items_data": power_items_data})

