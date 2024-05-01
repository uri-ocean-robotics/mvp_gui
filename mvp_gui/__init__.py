from flask import Flask, render_template
from flask_sqlalchemy import SQLAlchemy
from turbo_flask import Turbo
import time
import threading
import random
from datetime import datetime



app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///mvp_database.db'
app.config['SECRET_KEY'] = 'd5036a36d957701b9048179e'
turbo = Turbo(app)

db = SQLAlchemy(app)
app.app_context().push()


from mvp_gui import routes
