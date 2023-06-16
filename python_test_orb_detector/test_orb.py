from picamera2.picamera2 import *
from time import sleep

camera = Picamera2()

camera.start_preview()
sleep(20)
camera.stop_preview()
