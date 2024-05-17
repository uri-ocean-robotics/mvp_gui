from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()

class Vitals(db.Model):
    id = db.Column(db.Integer(), primary_key=True)
    name = db.Column(db.String(length=30), nullable=False, unique=True)
    voltage = db.Column(db.Numeric(10,2), nullable=False)
    current = db.Column(db.Numeric(10,2), nullable=False)
    def __repr__(self):
        return f'vitals {self.name}'

class Poses(db.Model):
    id = db.Column(db.Integer(), primary_key=True)
    frame_id = db.Column(db.String(length=30), nullable=False, unique=True)
    x = db.Column(db.Numeric(10,2), nullable=False)
    y = db.Column(db.Numeric(10,2), nullable=False)
    z = db.Column(db.Numeric(10,2), nullable=False)
    roll = db.Column(db.Numeric(10,2), nullable=False)
    pitch = db.Column(db.Numeric(10,2), nullable=False)
    yaw = db.Column(db.Numeric(10,2), nullable=False)
    child_frame_id = db.Column(db.String(length=30), nullable=False, unique=True)
    u = db.Column(db.Numeric(10,2), nullable=False)
    v = db.Column(db.Numeric(10,2), nullable=False)
    w = db.Column(db.Numeric(10,2), nullable=False)
    p = db.Column(db.Numeric(10,2), nullable=False)
    q = db.Column(db.Numeric(10,2), nullable=False)
    r = db.Column(db.Numeric(10,2), nullable=False)
    lat = db.Column(db.Numeric(10,8), nullable=False)
    lon = db.Column(db.Numeric(10,8), nullable=False)
    def __repr__(self):
        return f'poses {self}'
    

class PowerItems(db.Model):
    id = db.Column(db.Integer(), primary_key=True)
    device = db.Column(db.String(length=30), nullable=False, unique=True)
    gpio = db.Column(db.Integer(), nullable=False, unique=True)
    status = db.Column(db.String(length=5), nullable=False)
    def __repr__(self):
        return f'item {self}'
    
class CurrentWaypoints(db.Model):
    id = db.Column(db.Integer(), primary_key=True)
    lat = db.Column(db.Numeric(10,8), nullable=False)
    lon = db.Column(db.Numeric(10,8), nullable=False)
    alt = db.Column(db.Numeric(10,2), nullable=False)
    def __repr__(self):
        return f'cwaypoints {self}'

class Waypoints(db.Model):
    id = db.Column(db.Integer(), primary_key=True)
    lat = db.Column(db.Numeric(10,8), nullable=True)
    lon = db.Column(db.Numeric(10,8), nullable=True)
    alt = db.Column(db.Numeric(10,2), nullable=True)
    def __repr__(self):
        return f'waypoints {self}'


class HelmStates(db.Model):
    ## the first entry will be the current state the rest will be the connected states
    id = db.Column(db.Integer(), primary_key=True)
    name = db.Column(db.String(length=20), nullable=True)
    def __repr__(self):
        return f'states {self}'

class ControllerState(db.Model):
    ## the first entry will be the current state the rest will be the connected states
    id = db.Column(db.Integer(), primary_key=True)
    state = db.Column(db.String(length=20), nullable=False)
    def __repr__(self):
        return f'controllerstates {self}'

class RosActions(db.Model):
    id = db.Column(db.Integer(), primary_key=True)
    action = db.Column(db.String(length=20), nullable=False)
    value = db.Column(db.String(length=20), nullable=True)
    pending = db.Column(db.Integer(), nullable=False)
    def __repr__(self):
        return f'rosactions {self}'