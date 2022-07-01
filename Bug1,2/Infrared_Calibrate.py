import RPi.GPIO as GPIO
import time
import pickle
from TRSensor import TRSensor

CS = 5
Clock = 25
Address = 24
DataOut = 23

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)

# Simple example prints accel/mag data once per second:
if __name__ == '__main__':

    from AlphaBot import AlphaBot

    TR = TRSensor()
    Ab = AlphaBot()
    Ab.stop()
    print("Line follow Example")
    time.sleep(0.5)
    for i in range(0,1000):
        TR.calibrate()
        print(i)
    print(TR.calibratedMin)
    print(TR.calibratedMax)
    time.sleep(0.5)

    with open('ir_calib.pkl','wb') as ir_calib_file:
        pickle.dump(TR,ir_calib_file)