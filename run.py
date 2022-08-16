from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
import RPi.GPIO as GPIO

from camera import camera
import numpy as np
import threading
import time
import cv2

cam = camera()
pigpio_factory = PiGPIOFactory()

sL, sR, bt = 18, 27, 22
in1, in2, en = 19, 16, 13
GPIO.setmode(GPIO.BCM)
GPIO.setup(sL, GPIO.IN) # Sensor left
GPIO.setup(sR, GPIO.IN) # Sensor right
GPIO.setup(bt, GPIO.IN, pull_up_down=GPIO.PUD_UP) # button
GPIO.setup(in1, GPIO.OUT) # In 1
GPIO.setup(in2, GPIO.OUT) # In 2
GPIO.setup(en, GPIO.OUT) # En

motor = GPIO.PWM(en, 1000)
motor.start(0) 
servo = AngularServo(7, min_angle=0, max_angle=270, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=pigpio_factory)

# Lines
lowerBL = np.array([76,52,28])
upperBL = np.array([111,255,136])
lowerOR = np.array([0,141,109])
upperOR = np.array([41,255,166])

# Blocks
lowerGR = np.array([55,181,57])
upperGR = np.array([78,255,255])
lowerRD = np.array([0,185,69])
upperRD = np.array([43,255,255])

def drive(speed = 0, angle = 102, direction = 0):
    if speed == 0 or direction == 0: # No movement
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        time.sleep(0.5)

    if speed > 100:
        speed = 100
    elif speed < 0:
        speed = 0

    if angle > 135:
        angle = 135
    elif angle < 45:
        angle = 45

    motor.ChangeDutyCycle(speed)
    servo.angle = angle
    if direction > 0: # Forward
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    elif direction < 0: # Backward
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)

def sensorDetect():
    # Sensor inputs
    sensorL = GPIO.input(sL)    
    sensorR = GPIO.input(sR) 

    # Sensor logic
    if not sensorL and not sensorR: # -----------------------11
        drive(35, 135, 1) # Forward left
    elif sensorL and not sensorR: # -------------------------01
        drive(35, 135, 1) # Forward left
    elif not sensorL and sensorR: # --------------------------10
        drive(35, 50, 1) # Forward right
    else:
        return False
    return True

def turnTimer(times, dArgs=[0, 102, 0]):
    timeS = time.time()
    while time.time() - timeS < times:
        # Turning
        drive(*dArgs)
            
        #if sensorDetect():
        #    break

def findContour(mask, times, setArea=1000, name='something'):
    x, y, w, h = 0, 0, 0, 0
    image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour) 
        if area >= setArea:
            x, y, w, h = cv2.boundingRect(contour)
            if name != '':
                pass
                #print(f'~~~\nFound {name}!\nX: {x}, Y: {y}, W: {w}, H: {h}, Area: {area}\n~~~') #------------read -----------
            break
    
    return x, y, w, h

try:
    timeIt = time.time()
    vs = threading.Thread(target=cam.update, daemon=True)
    vs.start()
    drive(30, 99, 0)
    time.sleep(1)
    print('[INFO] Waiting for the button to be pressed')
    while GPIO.input(22): # Not pressed
        pass
    time.sleep(1)
    print('[INFO] GO!')

    mode = 1
    numLines = 0
    run = True
    while run:
        while mode == 0:
            if numLines == 12:
                mode = 2
                break
            ret, frame = cam.get()
            if not ret:
                continue
            frame = cv2.flip(frame, -1)
            crop = frame[:, 300:340]

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            crop = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            maskBL = cv2.inRange(crop, lowerBL, upperBL)
            maskOR = cv2.inRange(hsv, lowerOR, upperOR)
            maskGR = cv2.inRange(hsv, lowerGR, upperGR) 
            maskRD = cv2.inRange(hsv, lowerRD, upperRD) 
            
            bx, by, bw, bh = findContour(maskBL, 0, 100, 'BL')
            ox, oy, ow, oh = findContour(maskOR, 0, 100, 'OR')
            gx, gy, gw, gh = findContour(maskGR, 0, 100, 'GR')
            rx, ry, rw, rh = findContour(maskRD, 0, 100, 'RD')
            #cv2.imwrite('a.jpg', cv2.rectangle(maskOR, (ox, oy), (ox+ow, oy+oh), (0, 0 ,255), 2))    

            if False:  #------------------------------read Found------------------
                continue
            
            if by > 330 and oy > 210 and bh > 15: # Lane In
                #turnTimer(0.1, [30, 99, 0])
                #turnTimer(1.5, [40,85, 1]) # Forward

                #turnTimer(0.25, [30, 50, 0]) 
                turnTimer(0.5, [40, 99, 1]) # forward
                turnTimer(0.8, [40, 50, 1]) # Backward right
                turnTimer(1.2, [40, 135, 1]) # Backward left
                turnTimer(0.5, [40, 50, -1]) # Backward left

                turnTimer(0.1, [40, 99, -1]) # Backward right
                numLines += 1
                mode = 1

                #turnTimer(0.1, [30, 98, 0]) # stop
            elif by > 300 : # Lane Mid - Lane Out
                turnTimer(0.1, [30, 125, 0])
                while True:
                    ret, frame = cam.get()
                    if not ret:
                        continue
                    frame = cv2.flip(frame, -1)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    maskOR = cv2.inRange(hsv, lowerOR, upperOR)
                    ox, oy, ow, oh = findContour(maskOR, 0, 100, 'OR')
                    if oy > 215 and oh > 80:
                        break
                    drive(35, 135, 1)
                    time.sleep(0.01)
                turnTimer(0.1, [30, 98, 0])
                numLines += 1
                mode = 1
            
            elif not sensorDetect():
                drive(30, 99, 1)
                time.sleep(0.01)

        while mode == 1:
            ret, frame = cam.get()
            if not ret:
                continue
            frame = cv2.flip(frame, -1)

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            crop = hsv[:, 300:340]
            maskBL = cv2.inRange(crop, lowerBL, upperBL)
            maskGR = cv2.inRange(hsv, lowerGR, upperGR) 
            maskRD = cv2.inRange(hsv, lowerRD, upperRD) 
            
            bx, by, bw, bh = findContour(maskBL, 0, 500, 'BL')
            gx, gy, gw, gh = findContour(maskGR, 0, 100, 'GR')
            rx, ry, rw, rh = findContour(maskRD, 0, 100, 'RD')
            #cv2.imwrite('a.jpg', cv2.rectangle(maskOR, (ox, oy), (ox+ow, oy+oh), (0, 0 ,255), 2))    

            if False:  #------------------------------read Found------------------
                continue

            if by > 300:
                mode = 0

            if gx < 350 and gx != 0 and gy > 75 and gw > 35 and gh > 75: #---------------------------green-------------------
                while gx < 350:

                    #turnTimer(0.25, [30, 98, 0])
                    #turnTimer(0.5, [30, 98, -1])

                    #turnTimer(0.1, [30, 50, 0])
                    turnTimer(0.8, [40, 50, -1])

                    #turnTimer(0.25, [30, 135, 0])
                    turnTimer(0.55, [40, 135,  1])

                    #turnTimer(0.25, [30, 50, 0])
                    turnTimer(0.50, [40, 50, 1])
                    turnTimer(0.1, [30, 99, 0])

                    ret, frame = cam.get()
                    if not ret:
                        continue
                    frame = cv2.flip(frame, -1)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    maskGR = cv2.inRange(hsv, lowerGR, upperGR)
                    gx, gy, gw, gh = findContour(maskGR, 0, 100, 'GR')
            
            elif rx > 250 and ry > 70 and rw > 35 and rh > 75:#-----------------------red--------------
                while rx > 250:

                    #turnTimer(0.25, [30, 98, 0])
                    #turnTimer(0.5, [30, 98, -1])
                    
                    #turnTimer(0.25, [30, 135, 0])
                    turnTimer(0.8, [40, 135, -1])

                    #turnTimer(0.25, [30, 50, 0])
                    turnTimer(0.55, [40, 50,  1])

                    #turnTimer(0.25, [30, 135, 0])
                    turnTimer(0.55, [40, 135, 1])
                    turnTimer(0.1, [30, 99, 0])    

                    ret, frame = cam.get()
                    if not ret:
                        continue
                    frame = cv2.flip(frame, -1)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    maskRD = cv2.inRange(hsv, lowerRD, upperRD)
                    rx, ry, rw, rh = findContour(maskRD, 0, 100, 'RD')

            elif not sensorDetect():
                drive(30, 99, 1)
                time.sleep(0.01)

    while mode == 2:
        turnTimer(0.1, [30, 99, 0])
        turnTimer(2, [30, 99, 1])
        run = False
        break

except KeyboardInterrupt:
    pass
    #cv2.imwrite('a.jpg', frame)
except Exception as e:
    print(e)

print(f'[INFO] Total time: {time.time() - timeIt:.2f}s')
drive(0, 98, 0) # Forward center
time.sleep(0.5)
GPIO.cleanup()
cam.shutdown()