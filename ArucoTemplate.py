
########IMORTS#########

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv2 import aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from pymavlink.dialects.v20 import common
from array import array
from threading import Thread

init = 0
count = 0
#######VARIABLES########
app = Flask(__name__)

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
########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72 ##arucoID
marker_size = 20 ##CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count=0
notfound_count=0

#############CAMERA INTRINSICS#######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

#####
time_last=0
time_to_wait = .1 ##100 ms
################FUNCTIONS###############
def connectMyCopter():
        parser = argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect')
        args = parser.parse_args()

        connection_string = args.connect
        baud_rate = 57600

        vehicle = connect(connection_string,baud=baud_rate) #,wait_ready=True)
        return vehicle

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable !=True:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode !='GUIDED':
        print('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
    print('Vehicle now in GUIDED mode. Have Fun!')

    vehicle.armed = True
    while vehicle.armed ==False:
        print('Waiting for vehicle to become armed.')
        time.sleep(1)
    print('Look out! Virtual props are spinning!')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('Current Altitude: %d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >=.95*targetHeight:
            break
        time.sleep(1)
    print('Target altitude reached!')

    return None

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
            time.sleep(2)
                #MARK NOT A TARGET HERE RETURN RESUME SIGNAL
    vehicle.send_mavlink(msg)
    vehicle.flush()

def AltCorrect(FiringAlt): #NEED TO ADJUST PWM SIGNAL TO WORK WITH VARIOUS ALTITUDES
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
    
    global notfound_count, found_count, time_last, time_to_wait, id_to_find, count, FiringAlt
  
    id_to_find= ###TODO

    
    frame = picam2.capture_array("main")
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    vehicle.channels.overrides['8']=2000
    ids = ''
    marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)

    try:
        print("index:",index)
        if marker_corners:
            AltCorrect(FiringAlt)
            rVec, tVec, _ = estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()
                
                x = round(tVec[i][0][0],1)
                y = round(tVec[i][1][0],1)
                z = np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)

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

                if vehicle.mode !='LOITER':
                    vehicle.mode = VehicleMode('LOITER')
                    while vehicle.mode !='LOITER':
                            time.sleep(1)
                    track(x_ang,y_ang)
              
                else:
                    track(x_ang,y_ang)

                marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z

                point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv.putText(
                    frame,
                    f"id: {alias[int(str(ids[0]))-16]}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
                
                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][1][0],1)} ",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
        
                
                print(marker_position)
                print('FOUND COUNT: '+str(found_count)+ ' NOTFOUND COUNT: '+str(notfound_count))
                

                found_count = found_count + 1
               

               
            else:
                notfound_count=notfound_count+1
              

        else:
            notfound_count=notfound_count+1
            
    except Exception as e:
        print('Target likely not found')
        print(e)
        notfound_count=notfound_count+1
       
    ret,buffer = cv.imencode('.jpg', frame)
    frame = buffer.tobytes()
    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return None

@app.route('/video_feed')
def video_feed():
    	return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def interrupt():
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
            print("MODE:", end = " ")
            print(vehicle.mode.name)
            if vehicle.armed==False and first==False:
                print("ENTER 'E' to exit\n\n")


if __name__=='__main__':
  interruptor = Thread(target=interrupt)
  interruptor.start()
  app.run(host='0.0.0.0', port=5000)

  connectMyCopter()
  arm_and_takeoff(5)
  time.sleep(1)
  send_local_ned_velocity(0.5,0,0)
  time.sleep(4)
  while True:
    subscriber()
    
