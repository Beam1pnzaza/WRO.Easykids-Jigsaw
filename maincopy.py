from le_eyes import demeyes
from read_lidar import lidar
import donkeycar as dk

import numpy as np
import time
import cv2

V = dk.Vehicle()

class webcam_video():
    def __init__(self, port=0):
        self.vs = cv2.VideoCapture(port)
        if not self.vs.isOpened():
            print('Not working')
        self.running = True
        self.ret = False
        self.img =  None

    def update(self):
        while self.running:
            self.ret, self.img = self.vs.read()

    def run_threaded(self):
        return self.ret, self.img

    def shutdown(self):
        self.running = False
        time.sleep(0.5)
        self.vs.release()

V.add(webcam_video(), outputs=['cam/notEmpty', 'cam/img'], threaded=True)
V.add(lidar(), outputs=['slide'], threaded=True)
V.add(demeyes(), inputs=['slide', 'cam/notEmpty', 'cam/img'], threaded=True)
V.start(rate_hz=20)


