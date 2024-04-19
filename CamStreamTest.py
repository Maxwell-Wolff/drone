import cv2 as cv
from cv2 import aruco
import numpy as np
from picamera2 import Picamera2
import math
import RPi.GPIO as GPIO
import time
import threading
from flask import Flask, Response

app = Flask(__name__)

horizontal_res = 1536
vertical_res = 864

horizontal_fov = 102 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 67 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

calib_data_path = r"/home/rpi/drone/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
param_markers =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(marker_dict, param_markers)

picam2=Picamera2()
picam2.preview_configuration.main.size = (1536, 864)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

MARKER_SIZE = 30.48  

alias = ["GMU","GWU","VT","Howard", "USF"]
centerx_array = np.array([0, 0, 0, 0, 0])
centery_array = np.array([0, 0, 0, 0, 0])
tempx= np.array([50, 50, 50, 50, 50])
tempy= np.array([50, 50, 50, 50, 50])
distance = np.array([0, 0, 0, 0, 0, 0])

def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
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

def generate_frames():
    while True:
        frame = picam2.capture_array("main")


        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)
        if marker_corners:
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

                

                # for pose of the marker
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
        ret,buffer = cv.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    	return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
	app.run(host='0.0.0.0', port=5000)

#picam2.stop()
#cv.destroyAllWindows()
