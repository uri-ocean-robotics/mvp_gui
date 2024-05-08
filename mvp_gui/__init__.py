from flask import Flask, render_template, request, redirect, url_for
from turbo_flask import Turbo
from mvp_gui.models import *


app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///mvp_database.db'
app.config['SECRET_KEY'] = 'd5036a36d957701b9048179e'

turbo = Turbo(app)

db.init_app(app)
app.app_context().push()

from mvp_gui import utils
from mvp_gui import routes
