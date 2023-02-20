mport cv2
import numpy as np
from imutils.video import VideoStream
import imutils
from time import sleep
import RPi. GPIO as GPIO
import time
GPIO.setmode (GPIO.BCM)
from gpiozero import DistanceSensor
import smbus
import PCA9685
import ServoPCA9685
12cBus smbus.SMBus (0)
pca9685 PCA9685. PCA9685 (12cBus)
servo00 ServoPCA9685. ServoPCA9685 (pca9685, PCA9685. CHANNEL23)
servo01 = ServoPCA9685. ServoPCA9685 (pca9685, PCA9685.CHANNEL25)
servo02= ServoPCA9685. ServoPCA9685 (pca9685, PCA9685. CHANNEL20)
servo03 ServoPCA9685. ServoPCA9685 (pca9685, PCA9685. CHANNEL21)
def servo_to() :
for angle in range (0,90+1):
servo23.set_angle (angle)
servo25.set_angle(-angle)
time.sleep (1)
for angle in range (0,45+1):
servo20.set_angle (angle)
servo21.set_angle (angle)
time.sleep(0.5)
def servo_out()
for angle in reversed (range (0,45+1)):
servo20.set_angle (angle)
servo21.set_angle (angle)
time.sleep (0.5)
for angle in reserved (range (0,90+1)):
servo23.set angle (angle)
servo25.set_angle(-angle)
time.sleep (1)
servo23.disable()
servo25.disable()
servo20.disable()
servo21.disable()
vs=cv2. VideoCapture (-1)
BLOOBSIZE=3000
GPIO.cleanup()
GPIO.setmode (GPIO.BCM)
GPIO.setup (14, GPIO. OUT)
GPIO.output (14, GPIO. LOW)
GPIO.setup (15,GPIO. OUT)
GPIO.output (15, GPIO. LOW)
GPIO.setup (17,GPIO. OUT)
GPIO.output (17, GPIO. LOW)
GPIO.setup (27,GPIO. OUT)
GPIO.output (27, GPIO. LOW)
CONTCOLOR=(0,255,0)
RT=2
def checkSize (height, widnt):
if height*widnt>=BLOOBSIZE:
return True
return False
else:
def stop():
GPIO.output (14, GPIO. LOW)
GPIO.output (15,
GPIO. LOW)
GPIO.output (17,
GPIO.output (27,
def forward ():
GPIO.output (14, GPIO.HIGH)
GPIO.output (15, GPIO.HIGH)
GPIO.output (17, GPIO.HIGH)
GPIO.output (27, GPIO.HIGH)
def right():
GPIO.output (14,
GPIO.output (15,
GPIO.output (17,
GPIO.output (27,
def left():
GPIO. LOW)
GPIO. LOW)
def avoidance():
GPIO. LOW)
GPIO. LOW)
GPIO. LOW)
GPIO. LOW)
GPIO.output (14,
GPIO. LOW)
GPIO.output (15,
GPIO. LOW)
GPIO.output (17,
GPIO. LOW)
GPIO.output (27, GPIO. LOW)
sensor_distance-DistanceSensor (echo=6, trigger=19)
def avoidance():
while checkSize()!=True:
distance=sensor.distance*1
print (distance)
if distance<20:
def stop()
time.sleep (1)
def right()
time.sleep (2)
def forward()
time.sleep (3)
forward()
else:
sensor_color=ColorSensor (scl=4, sda=3)
frameSize=(640,480)
sleep (2)
while True:
while True:
image=vs. read ()
img_copy-image.copy()
hsv=cv2.cvt Color (image, cv2.COLOR_BGR2HSV)
binary1=cv2.inRange (hsv, (18,60, 100), (255,255,0))
roil=cv2.bitwise_and (frame, frame, mask-binary1)
cv2.imshow("roi". roil)
gray-cv2.cvtColor (roil, cv2. RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
contours,_=cv2. find Contours (gray, cv2. RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
if len(contours) !=0:
maxc=max (contours, key=cv2.contourArea)
x,y,w, h=cv2.boundingRect (maxc)
cv2. rectangle (image, (x,y), (x+w, y+h), CONTCOLOR, RT)
cv2.imshow('Image', image)
avoidance ()
break
else:
right()
continue
while True:
image=vs.read()
img_copy-image.copy()
hsv=cv2.cvtColor (image, cv2.COLOR_BGR2HSV)
binl=cv2.inRange (hsv, (0,50,50), (10,255,255)) #red
bin2=cv2.inRange (hsv, (25,50,50), (32,255,255)) #blue
rl=cv2.bitwise_and (frame, frame, mask-binl)
r2=cv2.bitwise_and (frame, frame, mask-bin2)
cv2.imshow("bitwise".rl, r2)
gray=cv2.cvtColor (r1, r2, cv2. RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
contours,_=cv2. find Contours (gray, cv2. RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
if len(contours) !=0:
maxc=max(contours, key=cv2.contourArea)
x,y,w, h=cv2.boundingRect(maxc)
cv2.rectangle (image, (x,y), (x+w, y+h), CONTCOLOR, RT)
cv2.imshow('Image', image)
avoidance ()
servo_to()
break
else:
right()
continue
[20.02, 19:07] Ольга Николаевна: while True:
if valueofcolor==red:
image=vs. read ()
img_copy-image.copy()
hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
binary1=cv2.inRange (hsv, (18,60,100), (136,0,255))
roil=cv2.bitwise_and (frame, frame, mask-binary1)
cv2.imshow("roi". roil)
gray=cv2.cvtColor (roil, cv2. RETR_EXTERNAL, cv2. CHAIN_APPROX_NONE)
contours, =cv2.findContours (gray, cv2. RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
if len(contours) !=0:
maxc=max(contours, key=cv2.contourArea)
x,y,w, h=cv2.boundingRect (maxc)
cv2. rectangle (image, (x,y), (x+w, y+h), CONTCOLOR, RT)
cv2.imshow('Image', image)
avoidance ()
servo_out()
break
else:
right()
continue
if valueofcolor==red:
image=vs.read()
img_copy-image.copy()
hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
binary1=cv2.inRange (hsv, (18,60,100), (136,0,255))
roil=cv2.bitwise_and (frame, frame, mask-binary1)
cv2.imshow("roi". roil)
gray=cv2.cvtColor (roil, cv2. RETR_EXTERNAL, cv2. CHAIN_APPROX_NONE)
contours,_=cv2.findContours (gray, cv2. RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
if len(contours) !=0:
maxc=max(contours, key=cv2.contourArea)
x,y,w, h=cv2.boundingRect (maxc)
cv2. rectangle(image, (x,y), (x+w, y+h), CONTCOLOR, RT)
cv2.imshow('Image', image)
avoidance()
servo_out()
break
else:
right()
continue
k=cv2.waitkey (1)
if k==27:
break
cv2.destroyAllWindows()
vs.stop
