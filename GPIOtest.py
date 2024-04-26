import RPi.GPIO as GPIO
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from pymavlink.dialects.v20 import common
import argparse
fire=27
GPIO.setmode(GPIO.BCM)

GPIO.setup(fire, GPIO.OUT)
GPIO.output(fire, GPIO.LOW)


def connectMyCopter():
        parser = argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect')
        args = parser.parse_args()

        connection_string = args.connect
        baud_rate = 57600

        vehicle = connect(connection_string,baud=baud_rate) #,wait_ready=True)
        return vehicle

connectMyCopter()
GPIO.output(fire, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(fire, GPIO.LOW)
print("Fired")
