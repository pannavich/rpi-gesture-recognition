import cv2
import mediapipe as mp
import numpy as np
import os

# label of data we will collect
actions = ['motor_on','motor_off','air_on','air_off','air_motor_on','buzzer_off']

# initialize mediapipe hand skeleton detection
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mpDraw = mp.solutions.drawing_utils

# initialize camera
cap = cv2.VideoCapture(0)

# directory for saving collected data
os.makedirs('dataset',exist_ok=True)

while cap.isOpened():
    # iterate through our labels
    for idx, action in enumerate(actions):
        num = 0
        data = []
        ret, img = cap.read()
        img = cv2.flip(img,1)
        cv2.putText(img, f'Waiting for collecting {action.upper()} action...', org=(10, 30), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=2)
        cv2.imshow('img', img)
        cv2.waitKey(3000)

        while True:
            ret, img = cap.read()
            img = cv2.flip(img,1)
            img_show = img
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            x, y, c = img.shape
            result = hands.process(img)
            landmarks = []
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

                    # Get angle using arccos of dot product
                    angle = np.arccos(np.einsum('nt,nt->n',
                        v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:], 
                        v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:])) # [15,]
                    # Convert radian to degree
                    angle = np.degrees(angle) 


                    angle_label = np.array([angle], dtype=np.float32)
                    angle_label = np.append(angle_label, idx)
                    d = np.concatenate([joint.flatten(), angle_label])

                    # draw skeleton and text on frame
                    mpDraw.draw_landmarks(img_show,handslms, mpHands.HAND_CONNECTIONS)
                    cv2.putText(img_show, f'{action.upper()} #{num+1}', org=(10, 30), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=2)
            # show image
            cv2.imshow('img', img_show)
            key = cv2.waitKey(1)
            # press k to save frame 
            if key == ord('k'):
                print('Saved')
                num+=1
                data.append([d,idx])
            # press m to go to next label
            elif key == ord('m'):
                data = np.array(data)
                print(f'Saved to folder raw_{action}')
                # save data for this action in dataset
                np.save(os.path.join('dataset', f'raw_{action}'), data)
                break

