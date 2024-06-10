# MVP GUI #
## Dependencies
- [Flask](https://flask.palletsprojects.com/en/3.0.x/)
    ```
    pip3 install Flask
    ```

- [Flask SQLAlchemy](https://flask-sqlalchemy.palletsprojects.com/en/3.1.x/quickstart/#installation)
    ```
    pip3 install Flask-SQLAlchemy
    ```
- [Turbo Flask](https://turbo-flask.readthedocs.io/en/latest/quickstart.html)
    ```
    pip3 install turbo-flask
    ```
- [Flask WTF](https://flask-wtf.readthedocs.io/en/1.2.x/)
    ```
    pip3 install Flask-WTF
    ```
- [psutil](https://psutil.readthedocs.io/en/latest/)
    ```
    pip3 install psutil 
    ```
- [paramiko](https://www.paramiko.org)
    ```
    pip3 install paramiko
    ```
- [python3-rosnode](https://packages.debian.org/sid/python3-rosnode)
    ```
    sudo apt-get install python3-rosnode
    ```
- [mvp_msgs](https://github.com/uri-ocean-robotics/mvp_msgs)
    ```
    git clone https://github.com/uri-ocean-robotics/mvp_msgs.git
    ```

## Offline Map (mbtiles)
A test map is provided, if you want to load your own map
- Download the .mbtiles file for the region of interest
- Put the .mbtiles file into offline_map directory
- Extract the .mbtiles using [mb-util](https://github.com/mapbox/mbutil)
    ```
    mb-util your_file.mbtiles your_file_tiles_directory
    ```
- set the `TILES_DIR` in `__init__.py` to the correct directory


## User instruction

MVP GUI has the following web pages for user to operate the vehicles running mvp framework.
Please go to `doc` folder to find the readme for each page to see detailed instructions.

To run the gui, 
```
python3 run.py
```