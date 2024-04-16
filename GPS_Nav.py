# This script has successfully navigated waypoints, safely returned to break
# and landed, as well ass killed motors on command. For each failsafe, it takes
# a "ctrl+c" KeyboardInterrupt.
#
# On first interrupt, the copter will stop its path. If it has drifted, it will
# return to the breakpoint and land.
#
# On second interrupt, it will immediately kill the motors. THIS WILL MAKE IT
# FALL FROM ALTITUDE.
#
# On third interrupt, it will exit the script.
#
##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as common
import sys

init=0
#########FUNCTIONS#################

def connectMyCopter():	#Connects copter

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect

	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	vehicle = connect(connection_string,wait_ready=True)

	return vehicle

def arm_and_takeoff(targetHeight):# What name says- time.sleep() very important
	while vehicle.is_armable!=True: # to allow vehicle to switch modes.
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Look out! Virtual props are spinning!!")

	vehicle.simple_takeoff(targetHeight) ##meters

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")

	return None

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon

	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.01:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None

def breakgoto(targetLocation): # Goto handled differently for landing.
	global init				   # Time is needed so that the landcoord is not
	vehicle.simple_goto(targetLocation) # immediately equal to current location.
										# This allows vehicle that has drifed to
	while vehicle.mode.name=="GUIDED":	#return to its position when given Land.
		if init==0:
			time.sleep(0.5)
			init=1
		elif init==1:
			currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
			if currentDistance<1:
				print("Returned to Break Point")
				time.sleep(1)
				init=0
				break
				return None

def Land():
	landcoord=vehicle.location.global_relative_frame
	breakgoto(landcoord)
	print("Landing")
	vehicle.mode = VehicleMode("LAND")
	while vehicle.armed:
		time.sleep(1)
	vehicle.close

##########MAIN EXECUTABLE###########

try: # try enables use of except KeyboardInterrupt
	while True:

		vehicle = connectMyCopter()

		#MAIN VARIABLES
		wp1 =LocationGlobalRelative(28.86489259,-82.512742,4)
		wp2 =LocationGlobalRelative(28.86494432,-82.512295,4)
		vehicle.airspeed=2.23 #2.23 is 5mph


		arm_and_takeoff(4)
		goto(wp1)
		goto(wp2)
		Land()
		while True:
			time.sleep(1)

except KeyboardInterrupt: #Emergency Interrupts
	try:
		print("\n\nEMERGENCY LAND\nNEXT INTERRUPT WILL KILL MOTORS")
		Land()
	except KeyboardInterrupt:
		try:
			print("\n\nKILLING MOTORS")
			msg = vehicle.message_factory.command_long_encode(
			0,0, #target system, target component
			common.MAV_CMD_DO_FLIGHTTERMINATION, #command
			0, #confirmation
			1, #param 1
			0,0,0, #unused params
			0,0,0)
			vehicle.send_mavlink(msg)
			while True:
				time.sleep(1)
		except KeyboardInterrupt:
			print("\n\nKilling Program")
			exit()
