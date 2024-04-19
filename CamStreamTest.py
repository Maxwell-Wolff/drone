import cv2 as cv2
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import numpy as np
from flask import Flask, Response

app = Flask(__name__)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
 
alias = ["GMU", "GWU", "VT", "Howard", "USF"]
 
def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:
 
        ids = ids.flatten()
 
        
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
 
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
 
            cv2.line(image, topLeft, topRight, (255, 0, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
 
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, alias[int(str(markerID))-16] + " ID:" + str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            
 
    return image
 
picam2=Picamera2()
picam2.preview_configuration.main.size = (1280, 720)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

def generate_frames():# 
    while True:
     
        img = picam2.capture_array("main")
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(img)
        detected_markers = aruco_display(markerCorners, markerIds, rejectedCandidates, img)
        #cv2.imshow("Markers", detected_markers)
        
        ret, buffer = cv2.imencode('.jpg', detected_markers)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
        
 #       key = cv2.waitKey(1) & 0xFF
 #       if key == ord("q"):
 #           break

@app.route('/video_feed')
def video_feed():
	return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
	app.run(host='0.0.0.0', port=5000)

#cv2.destroyAllWindows()
#picam2.stop()
