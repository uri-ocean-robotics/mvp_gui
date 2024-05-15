from flask_wtf import FlaskForm
from wtforms import StringField, SubmitField, FloatField, IntegerField


class WaypointForm(FlaskForm):
    id = IntegerField(label='id')
    lat = FloatField(label='latitude')
    lon = FloatField(label='longitude')
    alt = FloatField(label='altitude')
    

