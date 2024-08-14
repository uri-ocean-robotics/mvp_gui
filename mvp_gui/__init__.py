from flask import Flask, render_template, request, redirect, url_for, jsonify, send_from_directory, abort
from flask_socketio import SocketIO, emit
# from turbo_flask import Turbo
from mvp_gui.models import *
from mvp_gui.ros_manager import *
from mvp_gui.gui_ros_manager import *
from mvp_gui.forms import *
import yaml

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///mvp_database.db'
app.config['SECRET_KEY'] = 'd5036a36d957701b9048179e'

# turbo = Turbo(app)
socketio = SocketIO(app, cors_allowed_origins="*")

db.init_app(app)
app.app_context().push()

# Path to the directory where tiles are stored  
# TILES_DIR = "../offline_map/beach_pond_tiles_directory/"
TILES_DIR_1 = "./offline_map/"
TILES_DIR_2 = "../offline_map/"


global_file_name = './config/gui_config.yaml'
config = yaml.safe_load(open(global_file_name, 'r'))
roslaunch_folder_default = config['roslaunch_folder']
ros_source_base = config['ros_source_base']

project_path = os.getcwd()
env = os.environ.copy()
env['PYTHONPATH'] = project_path

# Create SSHConnection instance
ssh_connection = SSHConnection(config['remote_host'], config['remote_user'], config['remote_password'])


from mvp_gui.routes import routes_base
from mvp_gui.routes import routes_systems
from mvp_gui.routes import routes_rostopics
from mvp_gui.routes import routes_map
from mvp_gui.routes import routes_mission
from mvp_gui.routes import routes_power_manager

