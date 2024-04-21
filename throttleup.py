from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
#import exceptions
import math
import argparse
import os
import sys
from pymavlink.dialects.v20 import common

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

if __name__ == "__main__":
	try:
		vehicle = connectMyCopter()
		arm()
		vehicle.channels.overrides['3'] = 10
		#time.sleep(3)
		#vehicle.channels.overrides['3'] = 0
		time.sleep(1)
		#vehicle.channels.overrrides['3']=0
		vehicle.channels.overrides['3'] = 1350
		time.sleep(1)
		vehicle.channels.overrides['3'] = 1500
		time.sleep(1)
 		# vehicle.channels.overrides['3'] = 0
 		# time.sleep(1)
		msg = vehicle.message_factory.command_long_encode(
			0,0, #target system, target component
			common.MAV_CMD_DO_FLIGHTTERMINATION, #command
			0, #confirmationlast_refresh_time=0
			1, #param 1
			0,0,0, #unused params
			0,0,0)
		vehicle.send_mavlink(msg)
		vehicle.close
		sys.exit()

	except KeyboardInterrupt:
		msg = vehicle.message_factory.command_long_encode(
			0,0, #target system, target component
			common.MAV_CMD_DO_FLIGHTTERMINATION, #command
			0, #confirmationlast_refresh_time=0
			1, #param 1
			0,0,0, #unused params
			0,0,0)
		vehicle.send_mavlink(msg)
	
