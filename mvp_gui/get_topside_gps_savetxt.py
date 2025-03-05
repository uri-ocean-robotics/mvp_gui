import io
import pynmea2
import serial
import os
import datetime
import time
import numpy as np
from pathlib import Path

class gps_topside():
    def __init__(self):
        self.get_date()
        self.gps_topside_dir = os.path.join(Path.home(), 'gps_topside')
        if not os.path.exists(self.gps_topside_dir):
            os.makedirs(self.gps_topside_dir) 
        self.gps_topside_txt = os.path.join(self.gps_topside_dir, self.formatted_date + '.txt')
        if os.path.exists(self.gps_topside_txt):
            os.remove(self.gps_topside_txt)

        self.setup_gps()
        self.main_loop()

    def get_date(self):
        self.current_date = datetime.date.today()
        self.formatted_date = self.current_date.strftime("%Y_%m_%d")
        

    def setup_gps(self):
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1.0)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))


    def main_loop(self):
        while True:
            self.read_gps()
            time.sleep(1.0)

    def read_gps(self):
        try:
            line = self.sio.readline()
            msg = pynmea2.parse(line.strip())
            if msg.sentence_type == 'GGA':
                date_time = (msg.timestamp).strftime("%H-%M-%S") # H:M:S
                if msg.latitude != None and msg.longitude != None and msg.altitude != None:
                    print(date_time, msg.latitude, msg.longitude, msg.altitude)
                    with open(self.gps_topside_txt, "a") as f:
                        f.write('{},{},{},{}\n'.format(date_time, msg.latitude, msg.longitude, msg.altitude))


        except serial.SerialException as e:
            print('Device error: {}'.format(e))
        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            


if __name__ == "__main__":
    gps_topside()

