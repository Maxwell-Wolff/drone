########IMORTS#########
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
#######VARIABLES########


gpfire = 17
GPIO.setmode(GPIO.BCM)

GPIO.setup(gpfire, GPIO.OUT)
GPIO.output(gpfire, GPIO.LOW)

velocity=-.5 #m/s
seekingalt=2 #m
FiringAlt=2
fire_time = 1
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

wp1 = LocationGlobalRelative(28.8649019,-82.5127223,2)
wp2 = LocationGlobalRelative(28.8649479,-82.5121833,2)
wp3 = LocationGlobalRelative(28.8648316,-82.5127111,2)
wp4 = LocationGlobalRelative(28.8648673,-82.5121852,2)

waypoints=[wp1,wp2,wp3,wp4]

wpinit_time=0
codeinit=0
interrupt = 0
Kill_Interrupt=0
########################
fire_init_time=0
ids_to_find = [16, 17, 18, 19]#72 ##arucoID
targsleft = 3
index = 0
time_last_seen=0

#app = Flask(__name__)

horizontal_res = 1536
vertical_res = 864

horizontal_fov = 102 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 67 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count=0
found=False
notfound_count=0
notfound=0
#############CAMERA INTRINSICS#######
calib_data_path = r"/home/rpi/drone/MultiMatrix.npz"
calib_data = np.load(calib_data_path)
#print(calib_data.files)

# r_vectors = calib_data["rVector"]
# t_vectors = calib_data["tVector"]

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
param_markers =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(marker_dict, param_markers)

############INIT CAMERA#############
picam2=Picamera2()
picam2.preview_configuration.main.size = (1536, 864)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

MARKER_SIZE = 30.48  #CM

alias = ["GMU","GWU","VT","Howard", "USF"]
centerx_array = np.array([0, 0, 0, 0, 0])
centery_array = np.array([0, 0, 0, 0, 0])
tempx= np.array([50, 50, 50, 50, 50])
tempy= np.array([50, 50, 50, 50, 50])
distance = np.array([0, 0, 0, 0, 0, 0])

#####
time_last=0
time_to_wait = .1 ##100 ms
last_refresh_time=0
refresh_time=10
################FUNCTIONS###############

def connectMyCopter():
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect
	baud_rate = 57600

	vehicle = connect(connection_string,baud=baud_rate) #,wait_ready=True)
	vehicle.parameters['PLND_ENABLED']=2
	vehicle.parameters['PLND_TYPE']=1
	vehicle.parameters['PLND_EST_TYPE']=0
	time.sleep(1)
	vehicle.parameters['RC8_OPTION']=39
	vehicle.parameters['RC7_OPTION']=0
	vehicle.parameters['RC9_OPTION']=0
	vehicle.parameters['RC10_OPTION']=0
	vehicle.parameters['RC11_OPTION']=0
	vehicle.channels.overrides['3']=0
	return vehicle


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

def get_distance_meters(targetLocation,currentLocation):
    dLat=targetLocation.lat - currentLocation.lat
    dLon=targetLocation.lon - currentLocation.lon
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
	#TODO: WHEREINEEDTOBE()
    global reached, flush
    if flush==1:
        return None
    if targetLocation==0:
        WHEREINEEDTOBE()
        print("WP=",currentwp)
        targetLocation = waypoints[currentwp]

    clock_start=0
    if interrupt == True:
        return None

    distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

    vehicle.simple_goto(targetLocation)

    #while
    while vehicle.mode.name=="GUIDED":
        currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
    	
        if currentDistance<distanceToTargetLocation*.03:
            print("Reached waypoint.")
            if flush == 0:
                reached=1
            clock_start=0
            time.sleep(2)
            #break
        if currentDistance<3 and clock_start==0:
            proximity_time = time.time()
            clock_start=1

        if currentDistance<1 and clock_start==1:
            if time.time() - proximity_time > 2:
                print("Reached waypoint on proximity time basis")
                if flush ==0:
                    reached=1
                clock_start=0
                time.sleep(2)
                #break
        #if tracking == False:
            #AltCorrect(seekingalt)
        subscriber()
        if reached ==1:
            goto(0)
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
            if interrupt == True:
                return None

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

##Send velocity command to drone
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


def track(x,y):
    if interrupt == True:
        return None
    global Fire, tracking_time,time_taken
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,0,)
    if Fire == False:
        if vehicle.location.global_relative_frame.alt/FiringAlt < 1.2 and vehicle.location.global_relative_frame.alt/FiringAlt > 0.8:
            if time_taken==0:
                tracking_time=time.time()
                time_taken=1
            if x < 0.2 and x>-0.2 and y < 0.2 and y>-0.2:
                Fire = True
                fire()
                time.sleep(2)
                return None
            print("TIMETAKEN:",tracking_time)
            if time.time()-tracking_time > 5:
                print("NUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGENUDGE")
                send_local_ned_velocity(0.3,0.3,0)
                time_taken=0
                #MARK NOT A TARGET HERE RETURN RESUME SIGNAL
    
    vehicle.send_mavlink(msg)
    vehicle.flush()
    subscriber()

def AltCorrect(FiringAlt):
    if interrupt == True:
        return None               #NEED TO ADJUST PWM SIGNAL TO WORK WITH VARIOUS ALTITUDES
    if vehicle.location.global_relative_frame.alt < FiringAlt:
        #vz = -0.3
        vehicle.channels.overrides['3']=1650
    if vehicle.location.global_relative_frame.alt > FiringAlt:
        #vz = 0.3
        vehicle.channels.overrides['3']=1350
    if vehicle.location.global_relative_frame.alt/FiringAlt < 1.2 and vehicle.location.global_relative_frame.alt/FiringAlt > 0.8:
        #vz = 0
        vehicle.channels.overrides['3']=1500
    return None

def fire(): #TODO: Fire logic
	global fire_time, ids_to_find, targsleft, id_to_find, index, sub, flush, flush_time, Fire
	initial_time=time.time()
	timestamp = datetime.now(pytz.utc).isoformat().replace("+00:00","Z")
	Lat = vehicle.location.global_relative_frame.lat
	Lon = vehicle.location.global_relative_frame.lon
	id=72#:TODO
	print("USF","UAV","WaterBlast!",id_to_find,timestamp,Lat,Lon,sep = "_")
	GPIO.output(gpfire, GPIO.HIGH)

	while True:
		subscriber()
		if time.time()-initial_time > fire_time:
			GPIO.output(gpfire, GPIO.LOW)
			Fire = False
			index=0
			targsleft=targsleft-1
			ids_to_find.remove(id_to_find)
			break

	vehicle.mode = VehicleMode('GUIDED')
	while vehicle.mode != VehicleMode('GUIDED'):
		time.sleep(1)
	print("ENDFIRE")
	flush=1
	flush_time=time.time()
	#dummy_yaw_initializer(True,seekingalt)
	time.sleep(1)
	return None


def WHEREINEEDTOBE():
    global currentwp, reached, waypointinit

    if flush==1:
        return(currentwp)

    if waypointinit == 0:
        currentwp=0
        waypointinit = 1
    for i in range(0,2):
        if currentwp==i and reached ==1 and currentwp < 2:

            currentwp = i+1
            reached=0
    if currentwp == 2 and reached ==1:
        currentwp=0

    return(currentwp)

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

def subscriber():
	if interrupt == True:
		return None
	global ugh, notfound_count, found_count, time_last, time_to_wait, id_to_find, ids_to_find, count, FiringAlt, sub, Fire, currentwp, found, notfound, index, time_last_seen, flush, flush_time, tracking, just_flushed

	timetosee=3
	if just_flushed == 1:
		timetosee=0

	id_to_find=ids_to_find[index]
	print("Looking for:",alias[index])
    
	frame = picam2.capture_array("main")
	gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    

	ids = ''
	marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)

	try:
		if marker_corners:
			#TODOvehicle.channels.overrides['8']=2000
			just_flushed=0
			AltCorrect(FiringAlt)
			rVec, tVec, _ = estimatePoseSingleMarkers(
			marker_corners, MARKER_SIZE, cam_mat, dist_coef
			)
			total_markers = range(0, marker_IDs.size)
			for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
				cv.polylines(
				frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
				)

				if ids[0]==id_to_find:
				
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
					
					#if vehicle.mode !='LOITER':
						#vehicle.mode = VehicleMode('LOITER')
						#while vehicle.mode !='LOITER':
						#time.sleep(1)
						#track(x_ang,y_ang)
						#tracking=True
						#ugh=0
					#else:
						#track(x_ang,y_ang)
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
					found = True
	
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
	if flush == 1: #and time.time()-flush_time >3:dddd
        #sub.unregister()
		return None
    #ret,buffer = cv.imencode('.jpg', frame)
   # frame = buffer.tobytes()
    #yield (b'--frame\r\n'
          # b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
	goto(0)
	return None

#@app.route('/video_feed')
#def video_feed():
    	#return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


def interrupt():
	global interrupt, Kill_Interrupt, last_refresh_time, refresh_time
	refresh = 0
	first = True
	while True:

		if first == True:
			print("\nENTER 'Q' for Emergency LAND\nENTER 'W' to KILL Motors\nENTER 'E' to exit\nENTER 'R' to Return to Launch\nENTER 'NULL' for Vehicle Status\nENTER 'O' for OPTIONS\n\n")
			first = False
	
	
		for _ in range(1):
			uinput = input()
	
	
		if uinput == "o":
			print("\nENTER 'Q' for Emergency LAND\nENTER 'W' to KILL Motors\nENTER 'E' to exit\nENTER 'R' to Return to Launch\nENTER 'NULL' for Vehicle Status\nENTER 'O' for OPTIONS\n\n")
	
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
			print("Exiting")
			#vehicle.close
			sys.exit()
	
		if uinput =="r":
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
			if vehicle.armed==False:
				print("Vehicle Disarmed")
				print("MODE:", end = " ")
				print(vehicle.mode.name)
			if vehicle.armed==False and first==False:
				print("ENTER 'E' to exit\n\n")


if __name__=='__main__':
	#app.run(host='0.0.0.0', port=5000)
###Variables
    
	interruptor = Thread(target=interrupt)
	interruptor.start()
	codeinit=1

	vehicle = connectMyCopter()
	#vehicle.airspeed=2.23 #5mph
	#arm_and_takeoff(seekingalt)
	#time.sleep(1)
	#goto(0)

	while True:
		subscriber()
		if interrupt == True:
			break
		if flush == 1:
			flush=0
			dummy_yaw_initializer(True,seekingalt)
			time.sleep(3)
			just_flushed=1
			goto(0)

                # lat_home=-35.3632609#vehicle.location.global_relative_frame.lat
                # lon_home=149.1652352#vehicle.location.global_relative_frame.lon
                # wp_home=LocationGlobalRelative(lat_home,lon_home,5)
                # wp1=LocationGlobalRelative(-35.36303741,149.1652374,5)
                # wp2=LocationGlobalRelative(lat_home-0.00022349,lon_home-0.0000022,5)
