import RPi.GPIO as GPIO
#######GPIO#######
gpfire = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpfire, GPIO.OUT)
GPIO.output(gpfire, GPIO.LOW)

GPIO.output(gpfire, GPIO.HIGH)
time.sleep(1)
GPIO.output(gpfire, GPIO.LOW)