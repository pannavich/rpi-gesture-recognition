import cv2
import numpy as np
import mediapipe as mp
import paho.mqtt.publish as publish 
import paho.mqtt.client as mqtt
from threading import Thread
import time
import tensorflow as tf
import os

from tflite_runtime.interpreter import Interpreter
from tflite_runtime.interpreter import load_delegate



actions = ['fan_on','fan_off','air_on','air_off','air_on_fan_on','buzzer_off']

class VideoStream:
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self,resolution=(640,480),framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

    # Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
    # Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
    # Return the most recent frame
        return self.frame

    def stop(self):
    # Indicate that the camera and thread should be stopped
        self.stopped = True

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

imW = 640
imH = 480


mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mpDraw = mp.solutions.drawing_utils

tflite_model_path = "./model.tflite"

# For coral, add load_delegate
# interpreter = Interpreter(model_path=tflite_model_path,experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
interpreter = Interpreter(model_path=tflite_model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
i = 0

# connect to mqtt
port = 1883 # default port
Server_ip = "broker.netpie.io" 
Client_ID = "YOUR CLIENT ID"
Token = "YOUR TOKEN"
Secret = "YOUR SECRET"

MqttUser_Pass = {"username":Token,"password":Secret}

# The callback for when the client receives a connect response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

client = mqtt.Client(protocol=mqtt.MQTTv311, client_id=Client_ID , clean_session=True)
client.on_connect = on_connect
client.username_pw_set(Token,Secret)
client.connect(Server_ip, port)
client.loop_start()

# Initialize video stream
videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
time.sleep(1)

while True:
    # Start timer (for calculating frame rate)
    t1 = cv2.getTickCount()

    # Grab frame from video stream
    frame1 = videostream.read()

    # Acquire frame and resize to expected shape [1xHxWx3]
    img = frame1.copy()
    x,y,c = img.shape
    img = cv2.flip(img,1)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    result = hands.process(img)

    landmarks = []
    if result.multi_hand_landmarks is None:
        i = 0
    if result.multi_hand_landmarks:
        for handslms in result.multi_hand_landmarks:
            joint = np.zeros((21,4))
            for j,lm in enumerate(handslms.landmark):
                joint[j] = [lm.x,lm.y,lm.z, lm.visibility]

            # Compute angles between joints
            parent_joint = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19], :3] # Parent joint
            child_joint = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], :3] # Child joint
            v = child_joint - parent_joint

            # Normalize v
            v = v/ np.linalg.norm(v, axis=1)[:, np.newaxis]

            # Get angle using arcos of dot product
            angle = np.arccos(np.einsum('nt,nt->n',
                v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:], 
                v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:])) # [15,]
            # Convert radian to degree
            angle = np.degrees(angle) 

            angle_label = np.array(angle, dtype=np.float32)
            d = np.concatenate([joint.flatten(), angle_label])
            d = d.reshape(d.shape[0],1)
            d = np.expand_dims(d, axis=0)
            d = np.array(d, dtype=np.float32)
            # Perform the actual detection by running the model with the input
            interpreter.set_tensor(input_details[0]['index'],d)
            interpreter.invoke()

            # Retrieve detection results
            label = interpreter.get_tensor(output_details[0]['index'])[0]
            action = actions[np.argmax(label)]

            # loop for preventing unintentional command
            if(np.max(label)>0.5):
                i+=1
                cv2.putText(img, f'{action.upper()}', org=(10, 30), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=2)
                
                # real intention
                if i == 15:
                    action = action.split('_')
                    if len(action)==4:
                        Publish_Topic1  = '@msg/'+ action[0]
                        message1 = action[1]
                        client.publish(Publish_Topic1, message1)
                        Publish_Topic2  = '@msg/'+ action[2]
                        message2 = action[3]
                        client.publish(Publish_Topic2, message2)
                    else:
                        Publish_Topic  = '@msg/'+ action[0]
                        message = action[1]
                        client.publish(Publish_Topic, message)
                    i = 0
            mpDraw.draw_landmarks(img,handslms, mpHands.HAND_CONNECTIONS)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.putText(img,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
    cv2.imshow('img', img)

    # Calculate framerate
    t2 = cv2.getTickCount()
    time1 = (t2-t1)/freq
    frame_rate_calc= 1/time1

    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
videostream.stop()
