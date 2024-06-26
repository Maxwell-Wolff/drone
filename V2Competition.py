################IMPORTS################
from __future__ import print_function
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import cv2 as cv
from cv2 import aruco
import sys
import time
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from pymavlink.dialects.v20 import common
from array import array
import socket
import sys
from threading import Thread
from datetime import datetime
import pytz
import flask as Flask
import argparse


################VARIABLES################

#######GPIO#######
gpfire = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpfire, GPIO.OUT)
GPIO.output(gpfire, GPIO.LOW)

#######Flight Parameters#######
START = False
airspeed = 0.5 # 1.11m/s = see everything
velocity=-.5 #m/s
seekingalt=3 #m
FiringAlt=2
fire_time = 1

wp1 = LocationGlobalRelative(37.22294647349227, -80.43222861834133 ,seekingalt)
wp2 = LocationGlobalRelative( 37.22274303730939, -80.43246465273391 ,seekingalt)
wp3 = LocationGlobalRelative(37.22275958988012, -80.43248678095821 ,seekingalt)
wp4 = LocationGlobalRelative(37.22297210320749, -80.43224806435663 ,seekingalt)
wp5 = LocationGlobalRelative(37.2229907915368, -80.43227220423769 ,seekingalt)
wp6 = LocationGlobalRelative(37.22277507453977, -80.43250756807801 ,seekingalt)
wp7 = LocationGlobalRelative( 37.22279429687148, -80.43253975458609 ,seekingalt)
wp8 = LocationGlobalRelative( 37.223007878005234, -80.43230372019352 ,seekingalt)
wp9 = LocationGlobalRelative( 37.22303243979685, -80.43233523614934 ,seekingalt)
wp10 = LocationGlobalRelative( 37.222818324779205, -80.43257194109415 ,seekingalt)
wp11= LocationGlobalRelative( 37.222834877333426, -80.43260211594549 ,seekingalt)
wp12 = LocationGlobalRelative( 37.22305006020841, -80.4323573643737 ,seekingalt)
wp13 = LocationGlobalRelative(37.22306661271176, -80.43239022143403 ,seekingalt)
wp14 = LocationGlobalRelative( 37.22285303174431, -80.43262558527437 ,seekingalt)
wp15 = LocationGlobalRelative(37.22287011824399, -80.43265643067792 ,seekingalt)
wp16 = LocationGlobalRelative( 37.223084453533566, -80.43240904426426 ,seekingalt)
wp17 = LocationGlobalRelative( 37.22310250415807, -80.43244121893053 ,seekingalt)
wp18 = LocationGlobalRelative( 37.22288764321843, -80.43267960395801 ,seekingalt)
wp19 = LocationGlobalRelative( 37.22291326352526, -80.43270812241245 ,seekingalt)
wp20 = LocationGlobalRelative( 37.223121137056694, -80.43247046862737 ,seekingalt)







waypoints=[wp1,wp2,wp3,wp4,wp5,wp6,wp7,wp8,wp9,wp10,wp11,wp12,wp13,wp14,wp15,wp16,wp17,wp18,wp19,wp20]


#######Function Variables#######
init = 0
waypointinit=0
wpinit_time = 0
sub = ''
Fire = False

tracking = False
tracking_time=0
time_taken=0
currentwp=0

reached = 0
ugh=0
flush=0
flush_time=0

just_flushed=0
wpinit_time=0
codeinit=0
interrupt = 0
Kill_Interrupt=0

fire_init_time=0
ids_to_find = [16, 17, 18, 19] ##arucoID
targsleft = 3
index = 0
time_last_seen=0

#######CAMERA Parameters#######
alias = ["GMU","GWU","VT","Howard", "USF"]
MARKER_SIZE = 30.48  #CM
horizontal_res = 1536
vertical_res = 864

horizontal_fov = 102 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 67 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

#######CAMERA INTRINSICS#######
calib_data_path = r"/home/rpi/drone/MultiMatrix.npz"
calib_data = np.load(calib_data_path)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
param_markers =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(marker_dict, param_markers)

#######INIT CAMERA#######
picam2=Picamera2()
picam2.preview_configuration.main.size = (1536, 864)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

centerx_array = np.array([0, 0, 0, 0, 0])
centery_array = np.array([0, 0, 0, 0, 0])
tempx= np.array([50, 50, 50, 50, 50])
tempy= np.array([50, 50, 50, 50, 50])
distance = np.array([0, 0, 0, 0, 0, 0])
found_count=0
found=False
notfound_count=0
notfound=0

#####
time_last=0
time_to_wait = .1 ##100 ms
last_refresh_time=0
refresh_time=10


################FUNCTIONS###############

#Connect to AutoPilot
def connectMyCopter():
  parser = argparse.ArgumentParser(description='commands')
  parser.add_argument('--connect')
  args = parser.parse_args()

  connection_string = args.connect
  baud_rate = 57600

  print("Connecting...")
  vehicle = connect(connection_string,baud=baud_rate) #,wait_ready=True)
  
  print("Setting Parameters...")
  vehicle.parameters['PLND_ENABLED']=1
  vehicle.parameters['PLND_TYPE']=1
  vehicle.parameters['PLND_EST_TYPE']=0
  vehicle.parameters['PLND_OPTIONS']=0
  vehicle.parameters['RC8_OPTION']=39
  vehicle.parameters['EK2_ALT_SOURCE']=2
  vehicle.parameters['EK2_ENABLE']=1
  #vehicle.parameters['RC7_OPTION']=0
  #vehicle.parameters['RC9_OPTION']=0
  #vehicle.parameters['RC10_OPTION']=0
  #vehicle.parameters['RC11_OPTION']=0
  vehicle.airspeed= airspeed
  
  return vehicle

#Arm and Take off to set seeking altitude
def arm_and_takeoff(targetHeight):
    while vehicle.is_armable !=True:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
        if interrupt == True:
            return None
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode !='GUIDED':
        print('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
        if interrupt == True:
            return None
    print('Vehicle now in GUIDED mode')

    vehicle.armed = True
    while vehicle.armed ==False:
        print('Waiting for vehicle to become armed.')
        time.sleep(1)
        if interrupt == True:
            return None
    print('Props are spinning!')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('Current Altitude: %d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >=.95*targetHeight:
            break
        time.sleep(1)
        if interrupt == True:
            return None
    print('Target altitude reached!')

    return None

#Math for goto() distance parameter, used to determine satisfaction of reaching a waypoint
def get_distance_meters(targetLocation,currentLocation):
    dLat=targetLocation.lat - currentLocation.lat
    dLon=targetLocation.lon - currentLocation.lon
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

#Determine optimal waypoint within the field to go to, given time previous area searched
def WHEREINEEDTOBE():
    global currentwp, reached, waypointinit

    if flush==1:
        return(currentwp)
    if waypointinit == 0:
        currentwp=0
        waypointinit = 1
    for i in range(0,21):
        if currentwp==i and reached ==1 and currentwp < 21:

            currentwp = i+1
            reached=0
    if currentwp == 21 and reached ==1:
      Land()
        #currentwp=0
    print("SET WAYPOINT: ", currentwp)
    return(currentwp)

#goto() Will move the drone to a GPS waypoint until it has satisfied a closeness condition,
#It will then call subscriber() to continue looking for a marker. The drone will autonomously 
#continue its path to a waypoint until the closeness condition is satisfied, a different 
#waypoint is passed, or the flight mode is switched from guided.
#Pass 0 for automated waypoint decision via WHEREINEEDTOBE(), 1 for manual gps location
def goto(targetLocation):
  global reached, flush, found
  if interrupt == True:
    return None
  if found == True:
    return None
  
  
  
  if flush==1:
    return None
  
  if targetLocation==0:
    WHEREINEEDTOBE()
    print("WP=",currentwp)
    targetLocation = waypoints[currentwp]
  
  clock_start=0
  distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
  vehicle.simple_goto(targetLocation)
  print("GOING TO WAYPOINT: ", currentwp)
  
  while vehicle.mode.name=="GUIDED":
    if found == True:
      break
    currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
    
    #if currentDistance<distanceToTargetLocation*.03:
    if currentDistance<1:
      print("Reached waypoint.")
      reached=1
      clock_start=0
      time.sleep(0.5)
      break
  
    #if tracking == False:
      #AltCorrect(seekingalt)
    #subscriber()
    #if reached ==1:
      #goto(0)
    #time.sleep(1)

  return None

# Goto handled differently for landing. Time is needed so that the landcoord is not
# immediately equal to current location. This allows vehicle that has drifed to
# return to its position when given Land.
def breakgoto(targetLocation): 
    global init				   
    vehicle.simple_goto(targetLocation)  
	
    while vehicle.mode.name=="GUIDED":
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
            if interrupt == True:
                return None

#Simple land, set mode to land and wait until successful. Interrupts are needed to be able
#to immediately kill motors without being stuck in the while vehicle.armed loop.
def Land():
    if Kill_Interrupt==True:
        return None
    landcoord=vehicle.location.global_relative_frame
    breakgoto(landcoord)
    time.sleep(1)
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(1)
        if interrupt==True:
            return None

    vehicle.close

##Send velocity command to drone. Specific MAVLINK message type.
def send_local_ned_velocity(vx,vy,vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Change drone yaw. Pass **rvec for camera alignment**, degrees, relative frame (1 for relative, 0 for global angle)
# **no longer in use
def align_yaw(rvec,deg,relative):
    dummy_yaw_initializer(False,0)

    yaw = deg

    msg = vehicle.message_factory.command_long_encode(
        0, 0, # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        -yaw, #Param 1, yaw in degrees
        15,  #Param 2, yaw speed deg/s
        0, #Param 3, Direction -1 ccw, 1 cw
        relative, # Param 4, relative offset 1, absolute angle 0
        0, 0, 0) # Param 5-7 not used
    vehicle.send_mavlink(msg)
    vehicle.flush()

#Send message to drone that guides it to its current location. Pass gps for whether it is a gps (1) or velocity (0) command.
# For GPS command, height can be adjusted by passing desired altitude to  altgiven.
def dummy_yaw_initializer(gps,altgiven):

    #global seekingalt,FiringAlt
    lat=vehicle.location.global_relative_frame.lat
    lon=vehicle.location.global_relative_frame.lon
    alt=vehicle.location.global_relative_frame.alt

    aLocation=LocationGlobalRelative(lat,lon,alt)
    if gps == True:
        aLocation.alt=altgiven
        breakgoto(aLocation)
    if gps == False:
        msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            aLocation.lat*1e7,
            aLocation.lon*1e7,
            aLocation.alt,
            0,
            0,
            0,
            0, 0, 0,
            0, 0)
        vehicle.send_mavlink(msg)
    vehicle.flush()

# track() sends a specific mavlink message given the x and y angle received from subscriber(). This centers the drone on the Aruco
# marker. It also corrects the altitude to firing alt with AltCorrect(). When within certain closeness parameters, it calls fire().
# If not within parameters, it will return to subscriber()
def track(x,y):
	global Fire, tracking_time,time_taken
	if interrupt == True:
		return None
	print("TRACK")
	msg = vehicle.message_factory.landing_target_encode(
	  0,
	  0,
	  mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
	  x,
	  y,
	  0,
	  0,0,)
	if x < 0.2 and x>-0.2 and y < 0.2 and y>-0.2:
		fire()
    #if Fire == False:
        #if vehicle.location.global_relative_frame.alt/FiringAlt < 1.2 and vehicle.location.global_relative_frame.alt/FiringAlt > 0.8:
            #if time_taken==0:
                #tracking_time=time.time()
                #time_taken=1
            #if x < 0.2 and x>-0.2 and y < 0.2 and y>-0.2:
                #Fire = True
                #fire()
                #time.sleep(2)
                #return None
            #print("TIMETAKEN:",tracking_time)
            #if time.time()-tracking_time > 5:
                #print("NUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGE")
                #send_local_ned_velocity(0.3,0.3,0)
                #time_taken=0
                #MARK NOT A TARGET HERE RETURN RESUME SIGNAL
	vehicle.send_mavlink(msg)
	vehicle.flush()

#Correct altitude by measuring altimeter and passing a velocity command to adjust accordingly.
def AltCorrect(FiringAlt):
	global interrupt
	if interrupt == True:
		return None               #NEED TO ADJUST PWM SIGNAL TO WORK WITH VARIOUS ALTITUDES
	print("Correcting ALT")
	if vehicle.location.global_relative_frame.alt < FiringAlt:
		vz = -0.5
		#vehicle.channels.overrides['3']=1650
	if vehicle.location.global_relative_frame.alt > FiringAlt*1.5:
		vz = 0.5
		#vehicle.channels.overrides['3']=1350
	if vehicle.location.global_relative_frame.alt/FiringAlt < 1.3 and vehicle.location.global_relative_frame.alt/FiringAlt > 0.9:
		vz = 0
		#vehicle.channels.overrides['3']=1500

	send_local_ned_velocity(0,0,vz)
	return None

# Handles firing events: logging, trigger pull for set time, maintaining tracking.
def fire(): #TODO: Fire logic
	global fire_time, ids_to_find, targsleft, id_to_find, index, sub, flush, flush_time, Fire, found
	initial_time=time.time()
	timestamp = datetime.now(pytz.utc).isoformat().replace("+00:00","Z")
	Lat = vehicle.location.global_relative_frame.lat
	Lon = vehicle.location.global_relative_frame.lon
	id=id_to_find#:TODO
	print("USF","UAV","WaterBlast!",id_to_find,timestamp,Lat,Lon,sep = "_")
	print("Trigger Pull on ",id_to_find,"!")
	GPIO.output(gpfire, GPIO.HIGH)

	while True:
		if time.time()-initial_time > fire_time:
			print("Trigger Release!")
			GPIO.output(gpfire, GPIO.LOW)
			Fire = False
			f= open("LOG.txt","a")
			f.write("\nUSF_UAV_WaterBlast!_"+str(id_to_find)+"__"+str(timestamp)+"_"+str(Lat)+"_"+str(Lon))
			f.write(" ")
			f.close
			index=0
			targsleft=targsleft-1
			
			ids_to_find.remove(id_to_find)
			found = False
			break
	#goto(wp0)

	vehicle.mode = VehicleMode('GUIDED')
	while vehicle.mode != VehicleMode('GUIDED'):
		time.sleep(1)
	if not ids_to_find:
	  vehicle.mode = VehicleMode('RTL')
	  while vehicle.armed == False:
	    time.sleep(1)
	    
	#goto(0)
	return None

# Given image from subscriber() do math to determine marker location, return rotation and translation vector arrays
def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv.solvePnP(marker_points, c, mtx, distortion, False, cv.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

# Handles image processing. If a target marker is found, detemine location, angles of displacement, switch to LOITER mode and enable tracking.
# If marker not found for given time since last marker seen, update index and search for the next marker every cycle until a marker is again found.
def subscriber():
  while True:
  	if interrupt == True:
  		return None
  	global ugh, notfound_count, found_count, time_last, time_to_wait, id_to_find, ids_to_find, count, FiringAlt, sub, Fire, currentwp, found, notfound, index, time_last_seen, flush, flush_time, tracking, just_flushed
  
  	timetosee=3
  	if just_flushed == 1:
  		timetosee=0

  	if index>targsleft:
  	  index = 0
  	id_to_find=ids_to_find[index]
  	#print("INDEX: ",index)
  	#print("Looking for:",alias[index])
      
  	frame = picam2.capture_array("main")
  	gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
      
  
  	ids = ''
  	marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)
  
  	try:
  		if marker_corners:
  			#vehicle.channels.overrides['8']=2000
  			just_flushed=0
  			#AltCorrect(FiringAlt)
  			rVec, tVec, _ = estimatePoseSingleMarkers(
  			marker_corners, MARKER_SIZE, cam_mat, dist_coef
  			)
  			total_markers = range(0, marker_IDs.size)
  			for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
  				cv.polylines(
  				frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
  				)
  				print("SEEN:", alias[ids[0]-16])
  				if ids[0]==id_to_find:
  				  found = True
  				  vehicle.channels.overrides['8']=2000
  	    
  				  corners = corners.reshape(4, 2)
  				  corners = corners.astype(int)
  				  top_right = corners[0].ravel()
  				  top_left = corners[1].ravel()
  				  bottom_right = corners[2].ravel()
  				  bottom_left = corners[3].ravel()
  				  x = round(tVec[i][0][0],1)
  				  y = round(tVec[i][1][0],1)
  				  y_sum=0
  				  x_sum=0
  					
  				  x_sum = marker_corners[0][0][0][0] + marker_corners[0][0][1][0] + marker_corners[0][0][2][0] +marker_corners[0][0][3][0]
  				  y_sum = marker_corners[0][0][0][1] + marker_corners[0][0][1][1] + marker_corners[0][0][2][1] +marker_corners[0][0][3][1]
  					
  				  x_avg = x_sum / 4
  				  y_avg = y_sum / 4
  				  x_ang = (x_avg - horizontal_res*.5)*horizontal_fov/horizontal_res
  				  y_ang = (y_avg - vertical_res*.5)*vertical_fov/vertical_res
  				  print("X_Ang:",x_ang)
  				  print("Y_Ang:",y_ang)
  				  print("\n\n")
  					
  				  if vehicle.mode !='LOITER':
  				    vehicle.mode = VehicleMode('LOITER')
  				    vehicle.channels.overrides['3']=1500
  				  while vehicle.mode !='LOITER':
  				    time.sleep(1)
  				    track(x_ang,y_ang)
  					#tracking=True
  						#ugh=0
  				  else:
  				    
  				    track(x_ang,y_ang)
  					#tracking=True
  						#ugh=0
  					
  				
  					
  					#point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
  					#cv.putText(
  						#frame,
  						#f"id: {alias[int(str(ids[0]))-16]}",
  						#top_right,
  						#cv.FONT_HERSHEY_PLAIN,
  						#1.3,
  						#(0, 0, 255),
  						#2,
  						#cv.LINE_AA,
  					#)
  					#print("made it HERE")
  					#cv.putText(
  						#frame,
  						#f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][1][0],1)} ",
  						#bottom_right,
  						#cv.FONT_HERSHEY_PLAIN,
  						#1.0,
  						#(0, 0, 255),
  						#2,
  						#cv.LINE_AA,
  					#)
  	        
  					#print("FOUND: ",alias[int(str(ids[0]))-16])
  				  print("X:",x," Y:",y)
  				  print('FOUND COUNT: '+str(found_count)+ ' NOTFOUND COUNT: '+str(notfound_count))
  	                
  				  found_count = found_count + 1
  				  
  	
  				  time_last_seen=time.time()
  			#else:
  				#notfound_count=notfound_count+1
  				#if time.time()- time_last_seen > timetosee:
  				#	index = index+1
  				#	tracking = False
  		else:
  			 notfound_count=notfound_count+1
  			 if time.time()- time_last_seen > timetosee:
  			   index = index+1
  			   tracking = False
  	except Exception as e:
  		print('Target likely not found')
  		print(e)
  		notfound_count=notfound_count+1
  		if time.time()- time_last_seen > 3 and timetosee:
  			index = index+1
  			tracking = False
  			time_last = time.time()
  	if index > targsleft:
  	  index=0
  	#if flush == 1: #and time.time()-flush_time >3:dddd
          #sub.unregister()
  		
  
  	

# Interrupt handler for given keyboard inputs. Threaded to make it always available.
def interrupt():
  global interrupt, Kill_Interrupt, last_refresh_time, vehicle, refresh_time, START
  refresh = 0
  first = True

  while True:
    if first == True:
      print("\nENTER 'S' to START\nENTER 'Q' for Emergency LAND\nENTER 'W' to KILL Motors\nENTER 'E' to exit\nENTER 'R' to Return to Launch\nENTER 'NULL' for Vehicle Status\nENTER 'O' for OPTIONS\n\n")
      first = False
	

    for _ in range(1):
      uinput = input()
      
      if uinput == "o":
        print("\nENTER 'S' to START\nENTER 'Q' for Emergency LAND\nENTER 'W' to KILL Motors\nENTER 'E' to exit\nENTER 'R' to Return to Launch\nENTER 'NULL' for Vehicle Status\nENTER 'O' for OPTIONS\n\n")
	
      if uinput == "s":
        print("Starting in 3...")
        time.sleep(1)
        print("Starting in 2...")
        time.sleep(1)
        print("Starting in 1...")
        time.sleep(1)
        print("Script Started!")
        START = True
      
      if uinput == "q":
        interrupt = True
        print("\n\nEMERGENCY LAND")
        Land()
        
      if uinput == "w":
        interrupt = True
        Kill_Interrupt == True
        print("\n\nKILLING MOTORS")
        msg = vehicle.message_factory.command_long_encode(
        0,0, #target system, target component
        common.MAV_CMD_DO_FLIGHTTERMINATION, #command
        0, #confirmationlast_refresh_time=0
        1, #param 1
        0,0,0, #unused params
        0,0,0)
        vehicle.send_mavlink(msg)
      
      if uinput =="e":
        interrupt = True
        if vehicle.armed == False:
          print("Exiting")
          vehicle.close
          sys.exit()
        if vehicle.armed == True:
          print("Vehicle Still Armed!")
        
      if uinput =="r": #TODO: Ensure motors can still be killed
        interrupt = True
        print("Return to Launch!")
        vehicle.mode = VehicleMode("RTL")
        while vehicle.armed:
          time.sleep(1)
          vehicle.close
          exit()
    
      
      if uinput == "":
        print("\n#####VEHICLE STATUS#####")
        mode=vehicle.mode.name
        if vehicle.armed==True:
          print("Vehicle Armed")
          print("MODE: ",mode)
        if vehicle.armed==False:
          print("Vehicle Disarmed")
          print("MODE:", end = " ")
          print(vehicle.mode.name)
        if vehicle.armed==False and first==False:
          print("ENTER 'E' to exit\n\n")


if __name__=='__main__':
  vehicle = connectMyCopter()
  print("Connected!")
  
  interruptor = Thread(target=interrupt)
  interruptor.start()
  
  subs = Thread(target=subscriber)
  subs.start()
  while True:
    
    if interrupt == True:
      break
#DONT CODE ABOVE HERE
    if START == True:
      arm_and_takeoff(3)
      while True:
        if interrupt == True:
          break
        
        goto(0)
        #if vehicle.location.global_relative_frame.alt < 3:
          #while True:
            #if interrupt == True:
              #break
            #AltCorrect(3)
            #time.sleep(1)
            #if vehicle.location.global_relative_frame.alt > 3:
              #break
      #arm_and_takeoff(3)
      #goto(0)
      #while True:
        #if interrupt == True:
          #break
