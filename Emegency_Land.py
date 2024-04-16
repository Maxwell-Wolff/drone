from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

import argparse

# Connect to the Vehicle
def connectMyCopter():
        parser = argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect')
        args = parser.parse_args()

        connection_string = args.connect
        baud_rate = 57600

        vehicle = connect(connection_string,baud=baud_rate) #,wait_ready=True)
        return vehicle


def Land():
##This function ensures that the vehicle has landed (before vechile.close is called)

  print("Landing")
  ##thread_distance.join()
  time.sleep(1)
  vehicle.mode = VehicleMode("LAND")
  while vehicle.armed:
    time.sleep(1)
  vehicle.close()


#----begin programming form here


##----------------------------------------------------------------------------------------------------------------->
vehicle = connectMyCopter()
Land()

