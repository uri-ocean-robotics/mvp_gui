from flask_wtf import FlaskForm
from wtforms import StringField, SubmitField, FloatField, IntegerField


class WaypointForm(FlaskForm):
    id = IntegerField(label='id')
    type = StringField(label='ll or xy')
    lat = FloatField(label='latitude')
    lon = FloatField(label='longitude')
    x = FloatField(label='x')
    y = FloatField(label='y')
    z = FloatField(label='depth')
    

