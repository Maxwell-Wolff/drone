from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
#import exceptions
import math
import argparse
import os
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

def arm():
        while vehicle.is_armable==False:
                print("Waiting for vehicle to become armable..")
                time.sleep(1)

        print("Vehicle is now armable")
        print("")

        vehicle.armed=True
        while vehicle.armed==False:
                print("Waiting for drone to become armed..")
                time.sleep(1)

        print("Vehicle is now armed")
        print("Props are spinning!!")

        return None

while (os.getppid() != 1):

        vehicle = connectMyCopter()
        arm()
        time.sleep(5)
        vehicle.armed=False
        while vehicle.armed!=False:
                print("Waiting for drone to disarm")
                time.sleep(1)
        vehicle.close()
        print("End of Test")
vehicle.armed=False
vehicle.close()
print("ssh lost")

