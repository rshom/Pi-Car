#!/usr/bin/python3
from threading import Thread

import numpy as np
import time

import RPi.GPIO as GPIO
import Adafruit_PCA9685
from picamera import PiCamera
from picamera.array import PiRGBArray

#GPIO.setwarnings(False) #??? In original code
GPIO.setmode(GPIO.BCM)  # Set pin numbering system for board


# set up a pulse width modulation function to send to servos
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)


class Sonar:

    def __init__(self, txPin=11, rxPin=8):
        self.soundSpeed = 340

        GPIO.setup(txPin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(rxPin, GPIO.IN)
        
        self.tx = txPin
        self.rx = rxPin


    def __del__(self):
        pass
    
    def __enter__(self):
        return self
    
    def __exit__(self,exception_type, exception_value, traceback):
        self.close()

    def close(self):
        pass

    def ping(self, maxRange=1, pulseWidth=.000015): #TODO: impliment distance
        '''Measure return time of a single ping'''

        # Transmit ping
        GPIO.output(self.tx, GPIO.HIGH)
        time.sleep(pulseWidth)
        GPIO.output(self.tx, GPIO.LOW)

        while not GPIO.input(self.rx):
            #??? waiting for ping to stop. Should already be done though
            # Forces the time to start/end at end of ping. Falling edge
            pass 
        t1 = time.time()
        while GPIO.input(self.rx):
            # Forces the time to start/end at end of ping. Falling edge
            pass
        t2 = time.time()
        return (t2-t1)*self.soundSpeed/2


class LineSensor:
    '''Line sensor consisting of 3 elements in a linear array'''
    
    def __init__(self,pin_middle=16, pin_left=19, pin_right=20, blackLine=True):
        '''Create instance of Line Sensor'''
        
        # Set up pins for linereader
        #GPIO.setwarnings(False) #??? global
        #GPIO.setmode(GPIO.BCM)  #??? global

        self.pin_middle = pin_middle
        self.pin_left = pin_left
        self.pin_right = pin_right

        print("Starting line sensor")
        time.sleep(2)
        GPIO.setup(self.pin_middle,GPIO.IN)
        GPIO.setup(self.pin_left,GPIO.IN)
        GPIO.setup(self.pin_right,GPIO.IN)

    def __del__(self):
        '''Called on destruction of instance'''
        self.close()
    
    def __enter__(self):
        '''Called at start of with statement'''
        return self
    
    def __exit__(self,exception_type, exception_value, traceback):
        '''Called at the end of with statement'''
        self.close()

    def __call__(self):
        '''Return the current state of the line sensor'''
        return np.array([GPIO.input(self.pin_left),
                         GPIO.input(self.pin_middle),
                         GPIO.input(self.pin_right)])

    def close(self):
        pass


class Camera(PiCamera):
    '''Camera class class which can read or stream over a socket'''

    def __init__(self, resolution=(640.480)):
        '''Constructor for camera'''

        print("Initializing picamera")
        PiCamera.__init__(self, resolution = resolution)
        time.sleep(1) # Give camera time to start up
        
        # Set up buffer for capture
        self.rawCapture = PiRGBArray(self)
        self.rawCapture.truncate(0)
        self.rawCapture.seek(0)
        
    def __del__(self):
        '''Destructor'''
        self.close()

    def __enter__(self):
        '''Called on with statement'''
        return self

    def __exit__(self,exception_type, exception_value, traceback):
        '''Called on exit from with statement'''
        self.close()

    def close(self):
        '''Close camera nicely'''
        PiCamera.close(self)

    def read(self):
        '''Return frame same as cv2.read()'''
        self.rawCapture.truncate(0)
        self.rawCapture.seek(0)
        PiCamera.capture(self,self.rawCapture, format="bgr",
                         use_video_port=True)
        return True,self.rawCapture.array

    def stream(self,sock):
        '''Continuous capture and send over network'''

        # Reset buffer
        self.rawCapture.truncate(0)
        self.rawCapture.seek(0)

        # Take pictures and send to client as quickly as client can handle
        for frame in self.capture_continuous(output=self.rawCapture,
                                             format= "bgr",
                                             use_video_port=True):

            # reformat image for sending
            image = frame.array
            encoded,buffer = cv2.imencode('.jpg',image)
            jpg_as_text = base64.b64encode(buffer)

            #TODO: send as numpy array and avoid using opencv
            # Send to client
            sock.send(jpg_as_text)

            # Reset buffer
            self.rawCapture.truncate(0)
            self.rawCapture.seek(0)
    

class Motor:

    def __init__(self, en, pin1, pin2):
        '''Set up pins and start motor'''

        GPIO.setwarnings(False)  #??? is this global
        GPIO.setmode(GPIO.BCM)   #??? is this global
        GPIO.setup(en, GPIO.OUT)
        GPIO.setup(pin1, GPIO.OUT)
        GPIO.setup(pin2, GPIO.OUT)

        self.en = en
        self.pin1 = pin1
        self.pin2 = pin2

        self.speed = 0

        self.pwm = 0
        # this could pop an error and need try: except: pass
        self.pwm = GPIO.PWM(en,1000)


    def __del__(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self,exception_type, exception_value, traceback):
        self.close()

    def close(self):
        self.stop()
        GPIO.cleanup() #???

    def stop(self):
        '''Stops motor after pulling in the opposite direction to hard stop'''
        #TODO: impliment the hard stop
        '''
        if self.speed>0:
            self.pulse(-100,.1)
        elif self.speed<0:
            self.pulse(100,.1)
        '''
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
        GPIO.output(self.en,   GPIO.LOW)
        self.speed = 0

    def coast(self):
        '''Releases motor to coast'''
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
        GPIO.output(self.en,   GPIO.LOW)
        self.speed = 0

    def forward(self, speed=100):
        '''Drives forward'''
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW)
        self.pwm.start(100)
        self.pwm.ChangeDutyCycle(speed)
        self.speed = speed

    def reverse(self, speed=100):
        '''Drives backwards'''
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.HIGH)
        self.pwm.start(100)
        self.pwm.ChangeDutyCycle(speed)
        self.speed = speed

    def drive(self, speed):
        '''checks direction and moves'''
        if speed>0:
            self.forward(speed)
        elif speed<0:
            self.reverse(-speed)
        else:
            self.stop()

    def pulse(self,speed,t):
        self.drive(speed)
        time.sleep(t)
        self.stop()


class Servo:

    '''Class defining a servo'''

    def __init__(self, pin, pwmMin, pwmMax, pwmCenter, angMin, angMax, angCenter):
        '''Create an instance of Servo class'''

        # Hardware pin
        self.pin = pin

        # Underlying PWM settings
        self._pwm_center = pwmCenter
        self._pwm_max = pwmMax
        self._pwm_min = pwmMin
        self._pwm_range = pwmMax-pwmMin

        # External angle options
        self.MIN = angMin
        self.MAX = angMax
        self.CENTER = angCenter
        self.RANGE =angMax-angMin

        # set to initial position
        self.angle = self.CENTER
        self.center()

    def __del__(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self,exception_type, exception_value, traceback):
        self.close()

    def close(self):
        pass

    def _angle_to_pwm(self, angle):
        '''Convert angle to corresponding pwm frequency'''
        return round((angle-self.CENTER)*(self._pwm_range/self.RANGE)+self._pwm_center)

    def goto(self, angle):
        '''Rotate to angle'''
        if angle>self.MAX:
            #print("Warning: request above available positions: using {} instead".format(self.MAX))
            angle=self.MAX
            
        elif angle<self.MIN:
            #print("Warning: request below available positions: using {} instead".format(self.MIN))
            angle=self.MIN
        pwm.set_pwm(self.pin, 0, int(self._angle_to_pwm(angle)))
        self.angle = angle

    def rotate(self, angle):
        '''Rotate an incriment'''
        self.goto(angle+self.angle)

    def center(self):
        '''Go to center'''
        self.goto(self.CENTER)

    def max(self):
        '''Go to max'''
        self.goto(self.MAX)

    def min(self):
        '''Go to min'''
        self.goto(self.MIN)


class PiCar:
    '''
    Provides an interface to control the car.

    Upon construction, this class initializes all controls and sensors.
    For controls, the car has a motor, a turning servo, and 2 servos controlling the head.
    For sensors, the car has a line sensor, a sonar, and a camera.
    The constructor expects all the peripherals to be plugged into the pi in a specific manner which can only be changed by directly changing the code of the constructor.
    '''

    def __init__(self):
        # Controls
        print("Initializing controls")
        self.motor = Motor(17,27,18)
        self.tilt = Servo(0,425,650,515,-30,30,0)
        self.pan = Servo(1,160,650,395,-90,90,0)
        self.turn = Servo(2,180,520,370,-45,45,0)

        # Sensors
        print("Initializing sensors")
        self.camera = Camera( resolution=(640,480) )
        self.sonar = Sonar()
        self.lineSensor = LineSensor(pin_middle=16,
                                             pin_left=19,
                                             pin_right=20,
                                             blackLine=True)
        # Variables
        self._mode = ''
        self._stopped=False


    
    def __del__(self):
        self.close()


    def __enter__(self):
        return self


    def __exit__(self,exception_type, exception_value, traceback):
        self.close()


    def close(self):
        self.all_stop()
        self.safety_thread.running = False
        self.safety_thread.join()
        self.camera.close()
        GPIO.cleanup() #??? originally in sonar?


    def all_stop(self):
        '''Stop the car and face forward. Exit any current mode'''
        self._stopped = True
        self.motor.stop()
        self.all_ahead()

    def all_ahead(self):
        '''Bring all controls to forward'''
        self.turn.center()
        self.tilt.center()
        self.pan.center()

    def ebrake(self, dir=1):
        '''
        Hard brake by reversing for a second
        
        Change dir to -1 to use ebrake when reversed
        '''
        
        self.motor.pulse(dir*100,.2)


    def run_cmd(self, cmd,arg=''):
        '''Run a preset command'''
        # Must match client side commands
        if cmd == 'disconnect':
            self.close()
        elif cmd == 'forward':
            #self.motor.pulse(100,.1)
            self.motor.forward(100)
        elif cmd == 'reverse':
            #self.motor.pulse(-100,.1)
            self.motor.reverse(100)
        elif cmd == 'stop':
            self.motor.stop()
        elif cmd == 'coast':
            self.motor.coast()
        elif cmd == 'all_stop':
            self.all_stop(),
        elif cmd == 'all_ahead':
            self.all_ahead()
        elif cmd == 'left':
            self.turn.rotate(30)
        elif cmd == 'right':
            self.turn.rotate(-30)
        elif cmd== 'straight':
            self.turn.center()
        elif cmd == 'tilt_down':
            self.tilt.rotate(-1)
        elif cmd == 'tilt_up':
            self.tilt.rotate(1)
        elif cmd == 'pan_left':
            self.pan.rotate(1)
        elif cmd == 'pan_right':
            self.pan.rotate(-1)
        else:
            pass

    def sonar_scan(self, distance=1, scanSpeed=1, tiltAngle = 0): #TODO: impliment distance
        '''Measure distances across full range of sonar

        Car will first look all the way to the left. It will then slowly turn all the way to the right, while making a sonar measurement at each angle.
        distance determines how long the ping should wait for a response.
        scanSpeed determines how far apart the pings are angularly. A slower scanSpeed means more pings and denser set of results. scanSpeed must be an integer greater than 0.

        sonar_scan() returns a list of distances and a second list of corresponding angles.
        '''

        self.tilt.goto(tiltAngle)
        angles = [ang for ang in range(self.pan.MIN,
                                       self.pan.MAX+scanSpeed,
                                       scanSpeed)]
        for angle in angles:
            self.pan.goto(angle)
            results.append(self.sonar.ping(distance))

        return results, angles


    def pulse(self,runTime, coastTime):
        '''
        Continuously pulse the motor.

        Generally, this method should be called as a thread to run in the background. Otherwise it will block the program from running.
        '''
        self._stopped = False
        while not self._stopped:
            self.motor.forward(100)
            time.sleep(runTime)
            self.motor.coast()
            time.sleep(coastTime)
        self._stopped = True

    def follow_line(self, darkLine = False, speed=1,
                    gain = (10,60,0), nHist = 100,
                    maxAng = 30):
        '''
        Look for a line and drive along following it

        The follow line is a generator needs to be continuously called in order to continue following the line. 
        '''

        self._mode='follow_line'
        print("Line following mode")
        # Set up a history of observations
        obs = np.zeros((nHist,3))

        P = 0
        turnAng = 0
        while self._mode=='follow_line':
            # If about to run into wall, ebrake and shut off
            # #TODO: unduplicate this code
            # if self.sonar.ping()<.2:
            #     self.ebrake()
            #     self.all_stop()
            #     # this should also break the loop, but just in case
            #     return

            # Take measure with line sensor
            current = self.lineSensor() # Place new observation on the end
            if (current==[1,1,1]).all():
                current = obs[-1] 
            if (current==[1,0,1]).all():
                current = obs[-1] 
            obs[:-1] = obs[1:] # Drop first in observation
            obs[-1] = current
            #print(obs[-1])
            # Determine wheel angle based on control algorithm
            [left, center, right] = obs.sum(0)

            #Pprev = P
            P = current[2]-current[0]
            I = P*abs(right-left)/nHist
            D = (nHist-center)/nHist     # Hi if recently on line
            #D = abs(turnAng/maxAng)
            # D will be 1 or 0
            # 1 means close to line (small turn)
            
            turnAng = (P*gain[0] + gain[1]*I) / (gain[2]*D+1)
            if turnAng>maxAng:
                turnAng = maxAng
            elif turnAng < -maxAng:
                turnAng = -maxAng
                
            print("{}  {} {}".format([left,center,right],[round(P), round(I), round(D)], round(turnAng)))
            
            # Pulse motor and turn wheels according to control algorithm
            self.turn.goto(turnAng)
            #time.sleep(1)

            #TODO: make these values adjustable in the function call
            if abs(turnAng)<12:
                self.motor.forward(70)
            elif abs(turnAng)<30:
                self.motor.forward(95)
            else:
                self.motor.forward(100)

            yield

        self._stopped = True


    def track_object(self):
        '''Keep an object in view and follow it'''
        pass

    def explore(self):
        pass



if __name__=='__main__':
    car = PiCar()
    try:
        
        follow = car.follow_line(darkLine=False,
                        speed=1,
                        gain=(5,50,1),
                        nHist = 50,
                        runTime = 1,
                        coastTime = .5,
                        maxAng = 35
        )
        
        follow = car.follow_line(darkLine=False,
                        speed=1,
                        gain=(10,20,0),
                        nHist = 150,
                        runTime = 1,
                        coastTime = .5,
                        maxAng = 35
        )
        while car._mode=='follow_line':
            try:
                next(follow)
            except:
                car._mode=''
                

    finally:
        car.all_stop()

 
