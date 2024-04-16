########IMORTS#########

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from pymavlink.dialects.v20 import common
from array import array
import socket
import sys
from threading import Thread

#######VARIABLES########

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)
vehicle.parameters['PLND_ENABLED']=2
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
time.sleep(1)
vehicle.parameters['CH8_OPT']=39
vehicle.parameters['CH7_OPT']=0
vehicle.parameters['CH9_OPT']=0
vehicle.parameters['CH10_OPT']=0
vehicle.parameters['CH11_OPT']=0
vehicle.channels.overrides['3']=0
velocity=-.5 #m/s
takeoff_height=4 #m
FiringAlt=2

init = 0
sub = ''
Fire = False
currentwp=0
interrupt = 0
Kill_Interrupt=0
########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 2#72 ##arucoID
marker_size = 20 ##CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count=0
found=0
notfound_count=0
notfound=0
#############CAMERA INTRINSICS#######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

#####
time_last=0
time_to_wait = .1 ##100 ms
last_refresh_time=0
refresh_time=10
################FUNCTIONS###############
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

    clock_start=0
    if interrupt == True:
        return None

    distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

    vehicle.simple_goto(targetLocation)

    while vehicle.mode.name=="GUIDED":
        currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
        subscriber()
    		# LOOK FOR MARKER() # TODO
    		# if FOUND && STILL A TARGET:
    		# 	DO TRACK()
    		# 	DETERMINE WHERE I NEED TO BE()
    		# 	GOTO(WHERE I NEED TO BE)
        if currentDistance<distanceToTargetLocation*.03:
            print("Reached waypoint.")
            clock_start=0
            time.sleep(2)
            break
        if currentDistance<3 and clock_start==0:
            proximity_time = time.time()
            clock_start=1

        if currentDistance<1 and clock_start==1:
            if time.time() - proximity_time > 2:
                print("Reached waypoint on proximity time basis")
                clock_start=0
                time.sleep(1)
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
    dummy_yaw_initializer()

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

def dummy_yaw_initializer():
	lat=vehicle.location.global_relative_frame.lat
	lon=vehicle.location.global_relative_frame.lon
	alt=vehicle.location.global_relative_frame.alt

	aLocation=LocationGlobalRelative(lat,lon,alt)

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
    global Fire
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,0,)

    if vehicle.location.global_relative_frame.alt/FiringAlt < 1.1 and vehicle.location.global_relative_frame.alt/FiringAlt > 0.9:
        if x < 0.03 and x>-0.03 and y < 0.03 and y>-0.03:
            print("FIRING!!!!")
            Fire=True
            time.sleep(2)
            return None
                #TODO: MARK NOT A TARGET HERE RETURN RESUME SIGNAL
    vehicle.send_mavlink(msg)
    vehicle.flush()

def AltCorrect(FiringAlt):
    if interrupt == True:
        return None               #TODO: NEED TO ADJUST PWM SIGNAL
    if vehicle.location.global_relative_frame.alt < FiringAlt:
        #vz = -0.3
        vehicle.channels.overrides['3']=1650
    if vehicle.location.global_relative_frame.alt > FiringAlt:
        #vz = 0.3
        vehicle.channels.overrides['3']=1350
    if vehicle.location.global_relative_frame.alt/FiringAlt < 1.1 and vehicle.location.global_relative_frame.alt/FiringAlt > 0.9:
        #vz = 0
        vehicle.channels.overrides['3']=1500
    return None


def msg_receiver(message):
    if interrupt == True:
        return None
    global notfound_count, found_count, time_last, time_to_wait, id_to_find, count, FiringAlt, sub, Fire, currentwp, found, notfound

    if Fire == True:
        sub.unregister()
        return None
    AltCorrect(FiringAlt)

    if Fire == False:
        if time.time() - time_last > time_to_wait:
            np_data = rnp.numpify(message) ##Deserialize image data into array
            gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
            vehicle.channels.overrides['8']=2000
            ids = ''
            (corners, ids, rejected) = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

            try:
                if ids is not None and currentwp==0:
                    if ids[0]==id_to_find:
                        ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=np_camera_matrix,distCoeffs=np_dist_coeff)
                        (rvec, tvec) = (ret[0][0,0,:],ret[1][0,0,:])
                        x = '{:.2f}'.format(tvec[0]) ### Xerror/distance between camera and aruco in CM
                        y = '{:.2f}'.format(tvec[1]) ### Yerror/distance between camera and aruco in CM
                        z = '{:.2f}'.format(tvec[2]) ### Zerror/distance between camera and aruco in CM

                        y_sum=0
                        x_sum=0
                        vy=0
                        vx=0
                        x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] +corners[0][0][3][0]
                        y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] +corners[0][0][3][1]

                        x_avg = x_sum / 4
                        y_avg = y_sum / 4
                        x_ang = (x_avg - horizontal_res*.5)*horizontal_fov/horizontal_res
                        y_ang = (y_avg - vertical_res*.5)*vertical_fov/vertical_res
                        print("X_Ang:",x_ang)
                        print("Y_Ang:",y_ang)


                        if vehicle.mode !='LOITER':
                            vehicle.mode = VehicleMode('LOITER')
                            while vehicle.mode !='LOITER':
                                time.sleep(1)
                            track(x_ang,y_ang)

                        else:
                            track(x_ang,y_ang)



                        marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z

                        aruco.drawDetectedMarkers(np_data,corners)
                        aruco.drawAxis(np_data,np_camera_matrix,np_dist_coeff,rvec,tvec,10)
                        ##putText(image, text_to_draw,position,font_face,fontScale,color,thickness)
                        cv2.putText(np_data,marker_position,(10,50),0,.7,(255,0,0),thickness=2)
                        print(marker_position)
                        print('FOUND COUNT: '+str(found_count)+ ' NOTFOUND COUNT: '+str(notfound_count))

                        found_count = found_count + 1
                        found = True
                    else:
                        notfound_count=notfound_count+1
                else:
                    notfound_count=notfound_count+1
                    found = False
            except Exception as e:
                print('Target likely not found')
                print(e)
                notfound_count=notfound_count+1
            new_msg = rnp.msgify(Image, np_data,encoding='rgb8')
            newimg_pub.publish(new_msg)
            time_last = time.time()
        else:
            return None


def subscriber():
    global sub
    #TODO: since goto() calls this function, logic for switching back to guided should be elsewhere. When the drone reaches final waypoint, subscriber is not called and it will not switch back to guided. So Land() falls on deaf ears.
    rospy.init_node('drone_node',anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)

    return None



def interrupt():
    global interrupt, Kill_Interrupt, last_refresh_time, refresh_time
    refresh = 0
    first = True
    while True:

        if first == True:
            print("\nENTER 'Q' for Emergency LAND\nENTER 'W' to KILL Motors\nENTER 'E' to exit\nENTER 'NULL' for Vehicle Status\nENTER 'O' for OPTIONS\n\n")
            first = False


        for _ in range(1):
            uinput = raw_input()


        if uinput == "o":
            print("\nENTER 'Q' for Emergency LAND\nENTER 'W' to KILL Motors\nENTER 'E' to exit\nENTER 'NULL' for Vehicle Status\nENTER 'O' for OPTIONS")

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
            vehicle.close
            exit()

        if uinput == "":
            print("\n#####VEHICLE STATUS#####")
            mode=vehicle.mode.name
            if vehicle.armed==True:
                print("Vehicle Armed")
            if vehicle.armed==False:
                print("Vehicle Disarmed")
            print "MODE:",
            print(vehicle.mode.name)
            if vehicle.armed==False and first==False:
                print("ENTER 'E' to exit\n\n")


if __name__=='__main__':
###Variables
    vehicle.airspeed=2.23 #5mph
    lat_home=-35.3632609#vehicle.location.global_relative_frame.lat
    lon_home=149.1652352#vehicle.location.global_relative_frame.lon
    wp_home=LocationGlobalRelative(lat_home,lon_home,5)
    wp1=LocationGlobalRelative(-35.36303741,149.1652374,5)
    wp2=LocationGlobalRelative(lat_home-0.00022349,lon_home-0.0000022,5)
    print("\nlat_home:")
    print(lat_home)
    print("\nlon_home:")
    print(lon_home)
    interruptor = Thread(target=interrupt)
    interruptor.start()


    try:
        arm_and_takeoff(5)
        time.sleep(1)
        #currentwp=1
        goto(wp1)
        goto(wp2)
        #currentwp=0
        goto(wp_home)
        Land()
    except rospy.ROSInterruptException:
        pass
