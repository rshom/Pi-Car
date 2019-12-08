try:
    import keyboard # need to be root on os x to use this
except:
    print("Warning: Keyboard control requires root access")
    print("Check to see if keyboard module is installed and you are root")

import time
import cv2
import numpy as np
import socket
from threading import Thread
import zmq
import base64
import imutils
import speech_recognition as sr


serverAddr = '192.168.0.212'
#serverAddr = 'localhost'

class LiveFeed(Thread):
    '''
    Thread that receives frames from server and makes them available to other client threads.
    '''

    def __init__(self, addr=serverAddr ,port=5555, threaded=False):
        '''
        Live Feed constructor

        Connects to socket and starts thread if threaded=True
        '''
        
        if threaded:
            Thread.__init__(self)

        context = zmq.Context()
        self.sock = context.socket(zmq.SUB)
        self.sock.bind("tcp://*:{1}".format(addr,port)) #??? connect
        self.sock.setsockopt_string(zmq.SUBSCRIBE, np.unicode('')) #???
        self.frame = np.zeros(shape=[640, 480, 3], dtype=np.uint8)
        self.running=False

        print("feed started")
        if threaded:
            self.start()


    def read(self):
        '''Read a single frame from server'''
        frame = self.sock.recv()
        img = base64.b64decode(frame)
        npimg = np.frombuffer(img, dtype=np.uint8)
        self.frame = cv2.imdecode(npimg,1)
        return self.frame
        #self.frame = imutils.resize(frame, width=500)

    def run(self):
        '''Continuously read frames and update in background'''
        self.running=True
        while(self.running==True):
            self.frame = self.read()
            cv2.line(self.frame,(300,240),(340,240),(128,255,128),1)
            cv2.line(self.frame,(320,220),(320,260),(128,255,128),1)
            #print(self.frame)


class Keyboard(Thread):
    '''Keystroke commands that can be sent over the transmitter'''
    #TODO: keyboard should inherant from a remote transmitter class

    def __init__(self, transmitter):
        Thread.__init__(self)

        self.transmitter = transmitter #TODO: put in abstract controller class
        
        self.commands = {
            # First command runs on press
            # Second command runs on release
            'q': ('disconnect',''),
            'w': ('forward','coast'), #TODO: impliment coast
            's': ('reverse','coast'),
            'space': ('stop','coast'),
            '2': ('all_ahead',''),
            'a': ('left','straight'),
            'd': ('right','straight'),
            'i': ('tilt_down',''),
            'j': ('pan_left',''),
            'k': ('tilt_up',''),
            'l': ('pan_right',''),
            '9': ('follow_line','')
            }


        self.start()

    def run_cmd(self,cmd): #TODO: put in abstract controller class
        '''Send a command to picar reciever'''
        self.transmitter.send_cmd(cmd)

    def run(self):
        '''Background thread listening to for keyboard events'''
        while True:
            key = keyboard.read_event(suppress=True)
            if key.name=='esc':
                self.run_cmd('all_stop')

            if key.name=='7':
                cmd = voice_command()
                if cmd:
                    self.run_cmd(cmd)
            
            if key.name in self.commands:
                if key.event_type=='down':
                    # on press
                    self.run_cmd(self.commands[key.name][0])
                elif key.event_type=='up':
                    # on release
                    self.run_cmd(self.commands[key.name][1])


class Transmitter:
    '''
    Class containing a socket and the ability to send commands to the server

    There should only be one instance unless multiple cars are being controlled.
    Every controller must be passed the transmitter in order to work.
    '''
    def __init__(self, addr = serverAddr, port = 5000):

        self.size=1024

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((addr,port))

    
    def send_cmd(self, cmd):
        '''Send command to picar reciever'''
        self.sock.send(cmd.encode())


#TODO: Impliment a color tracker similar to the used in the sample code
    
class ObjectTracker(Thread):
    '''
    Object tracker is a thread that has an opencv tracker

    The tracker is given access to a live feed where gets updated frames and to a transmitter which can send commands to a picar. Object tracking with the PiCar is poor, likely due to the instability of the head, paticularly when moving.
    '''

    def __init__(self, feed, transmitter, tracker="kcf", mode="watch"):
        '''
        Object tracker constructor

        Multiple trackers are available through OpenCV. While each works uniquely, they are all objects with the same function calls for creating and updating the tracker. Each tracker runs on a different base algorithm, but has the same output. Each has advantages and disadvantages. Online research shows that KCF is likely the best possible tracker.

        The user must pass a feed and a transmitter in the contructor and can optionally input a specific tracker to use and a mode. 
        '''
        
        # Initialize underlying thread
        Thread.__init__(self)
        
        self.feed = feed
        self.transmitter = transmitter

        #TODO: impliment modes: track, watch, follow, lockFollow
        # track: track object on still camera
        # watch: move head to follow object
        # follow: attempt to maintain distance to object
        # lockFollow: follow with still head #TODO: better name
        self.mode = mode 

        OPENCV_OBJECT_TRACKERS = {
	    "csrt": cv2.TrackerCSRT_create,
	    "kcf": cv2.TrackerKCF_create,
	    "boosting": cv2.TrackerBoosting_create,
	    "mil": cv2.TrackerMIL_create,
	    "tld": cv2.TrackerTLD_create,
	    "medianflow": cv2.TrackerMedianFlow_create,
	    "mosse": cv2.TrackerMOSSE_create
        }

        # extract the OpenCV version info
        (major, minor) = cv2.__version__.split(".")[:2]
        
        # if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
        # function to create our object tracker
        if int(major) == 3 and int(minor) < 3:
            self.tracker = cv2.Tracker_create(tracker.upper())
        else:
            self.tracker = OPENCV_OBJECT_TRACKERS[tracker]()

        self.tracking = False
        # Thread does not start automatically

    def select(self,frame):
        '''
        Select object to track from a frame

        Gives the user the ability to select an object from a paused frame.
        The selected contains the portion of the image which will be tracked.
        '''
        #TODO: "Stream" replaced with viewname
        self.box = cv2.selectROI("Stream", frame, fromCenter=True, 
                               showCrosshair=True)
        self.tracker.init(frame,self.box)
        self.tracking = True
        #fps = FPS().start() #???
        return self.box

    def deselect(self):
        '''Clear selected object and stop tracking'''
        pass

    def track(self,frame):
        '''
        Get a single update of the tracker.
        '''
        (success,self.box) = self.tracker.update(frame)
        return success,self.box

    def run(self):
        '''
        Track object in background using latest frame

        Updates the box location for the viewer. Also sends command using transmitter so the car will look towards the object thus centering it in the screen.
        '''
        while self.tracking:
            success,self.box = self.tracker.update(feed.frame)
            # Locate offset from center
            #TODO: find midpoints better
            xmid = 640/2
            ymid = 480/2
            
            # find center
            (x,y,w,h) = [int(v) for v in self.box]
            x = x+w/2 # center
            y = y+h/2 # center

            xOffset = x-xmid
            yOffset = y-ymid
            
            # Adjust pan to look towards object
            if not success:
                self.transmitter.send_cmd('none')
            elif x>xmid:
                print("look right")
                self.transmitter.send_cmd('pan_right')
            elif x<xmid:
                print("look left")
                self.transmitter.send_cmd('pan_left')

            # Adjust tilt to look towards object
            if not success:
                self.transmitter.send_cmd('none')
            elif y>ymid:
                print("look down")
                self.transmitter.send_cmd('tilt_down')
            elif y<ymid:
                print("look up")
                self.transmitter.send_cmd('tilt_up')


            #TODO: impliment ability to drive towards object
            
            #print(area)
            time.sleep(1)


        
def voice_command():
    '''
    Get input from the user using voice recognition.

    Voice commands are limited because implimenting a call for each can become complicated. The car has a tendency to get out of control, because the user cannot respond quickly enough.
    '''
    r = sr.Recognizer()
    cmds = ['forward',
            'reverse',
            'turn left',
            'turn right',
            'pan left',
            'pan right',
            'tilt up',
            'tilt down',
            'all stop',
            'all ahead',
            'follow line']

    with sr.Microphone() as source:
        #r.adjust_for_ambient_noise(source)
        print("Listening")
        audio = r.listen(source)
    print("recording finished")
    try:
        a2t = r.recognize_google(audio)
    except:
        a2t=''
    print("Heard: {}".format(a2t))
    
    if a2t in cmds:
        return a2t.replace(' ','_').replace('-','_')
    else:
        return None
    

def main():
    transmitter = Transmitter()
    feed = LiveFeed(threaded=True) # Run feed in background
    remoteControl  = Keyboard(transmitter)
    tracker = None 
    viewer="Stream"
    # Viewer grab latest frame and displays it
    ## Viewer is seperate from the internal livestream which may be faster
    ## Viewer just grabs frame when it can
    #TODO: create start_viewer() function
    while viewer:
        frame = feed.frame
        if tracker:
            # place box on image
            (x,y,w,h) = [int(v) for v in tracker.box]
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          (0, 255, 0), 2)

        
        cv2.imshow(viewer, frame) # imshow must be in main thread
        key = cv2.waitKey(1) # waitKey must be in main thread
    
        if key == ord("q"):
            viewer=False
        
        elif key== ord("8"):
            if tracker:
                tracker.tracking=False
                tracker.join()
                tracker=None
            else:
                tracker = ObjectTracker(feed,transmitter)
                tracker.select(feed.frame)
                tracker.start()
        
        # Display speed doesn't need to keep up
        #time.sleep(.1)

    # Clean up
    #TODO: close sockets (should close out stuff on server nicely too)
    #TODO: close windows
    #TODO: shutdown all threads
    cv2.waitKey(1)
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == '__main__':
    while True:
        try:
            main()
        except:
            print("Restarting Client")
            pass
