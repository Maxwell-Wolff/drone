from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
#import exceptions
import math
import argparse

try:
        from collections.abc import MutableMapping
except ImportError:
        from collections import MutableMapping

def connectMyCopter():
        parser = argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect')
        args = parser.parse_args()

        connection_string = args.connect
        baud_rate = 57600

        vehicle = connect(connection_string,baud=baud_rate) #,wait_ready=True)
        return vehicle

vehicle = connectMyCopter()
vehicle.armed=False
print("Vehicle DISARMED")
