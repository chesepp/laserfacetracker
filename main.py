import cv2
import mediapipe as mp
import time
#from Arduino import Arduino
import serial
import numpy as np

FOV_X = 90
FOV_Y = 70  
RES_X = 1280
RES_Y = 720
#import pyautogui
countdowntime = 5
endoftimer = False
mp_hands = mp.solutions.hands
mp_face = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
cascPath = "hand.xml"
handCascade = cv2.CascadeClassifier(cascPath)

arduino = serial.Serial(port='/dev/cu.usbmodem1101',baudrate=9600, timeout=1)
GREEN_ON = "LED2_ON"
RED_ON = "LED1_ON"
LED_OFF = "LED_OFF"
LASER_ON = "LASER_ON"
LASER_OFF = "LASER_OFF"

video_capture = cv2.VideoCapture(1)
hands = mp_hands.Hands(static_image_mode=False,max_num_hands=2,min_detection_confidence=0.5,)
face = mp_face.FaceDetection(min_detection_confidence=0.5)
start_time = None

def pixel_to_servo_angle(norm_x, norm_y, frame_width, frame_height, fov_x=FOV_X, fov_y=FOV_Y):
    # Calculate normalized offsets from center
    x_angle = 90 + ((norm_x - 0.5) * fov_x)
    y_angle = 90 + ((norm_y - 0.5) * fov_y)

    y_angle -= 20
    #x_offset = (pixel_x - frame_width / 2) / frame_width
    #y_offset = (pixel_y - frame_height / 2) / frame_height

    # Map offsets to servo angles
    #x_angle = 90 + (x_offset * fov_x)
    #y_angle = 90 - (y_offset * fov_y)

    # Constrain angles to valid servo range
    x_angle = max(0, min(180, x_angle))
    y_angle = max(0, min(180, y_angle))

    return int(x_angle), int(y_angle)

if not video_capture.isOpened():
    print("ERROR: Unable to open a video source")
    exit()
while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        arduino.write(f"{LASER_OFF}\n".encode())
        break
    ret, frame = video_capture.read()
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    frame_height, frame_width ,_ =frame.shape

    hand_results = hands.process(rgb_frame)
    face_results = face.process(rgb_frame)
    
    if hand_results.multi_hand_landmarks:
        hand_not_detected = False
        if start_time is None:
            start_time = time.time()
        elapsed_time = time.time() - start_time
        remaining_time = max(0,countdowntime - elapsed_time)
        for hand_landmarks in hand_results.multi_hand_landmarks:
                # Draw landmarks and connections
            mp_drawing.draw_landmarks(
                frame, 
                hand_landmarks, 
                mp_hands.HAND_CONNECTIONS,
                mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)
            )
    else:
        hand_not_detected = True

    if face_results.detections:
        face_not_detected = False
        detection = face_results.detections[0]
        if start_time is None:
            start_time = time.time()
        elapsed_time = time.time() - start_time
        remaining_time = max(0,countdowntime - elapsed_time)
        if remaining_time > 0:
            cv2.putText(frame, f"Countdown: {int(remaining_time)}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        face_point = detection.location_data.relative_keypoints[2]
        normalized_x = face_point.x
        normalized_y = face_point.y
        servo_x, servo_y = pixel_to_servo_angle(normalized_x,normalized_y, frame_width,frame_height)
        arduino.write(f"X:{servo_x}\n".encode())
        arduino.write(f"Y:{servo_y}\n".encode())
        print(f"angles ({servo_x},{servo_y}) sent to servo motor")

        for detection in face_results.detections:
            mp_drawing.draw_detection(frame,detection)
    else: 
        face_not_detected = True
    
    #if face_not_detected and hand_not_detected:
        #start_time = None
        #arduino.write(f"{GREEN_ON}\n".encode())
        #arduino.write(f"{LASER_OFF}\n".encode())
    cv2.imshow('HAND STUFF', frame)
    if remaining_time <= 0:
        #COUNTDOWN FINISHED FIRE THE PEPPER SPRAY
        endoftimer = True
        cv2.putText(frame, "COUNTDOWN FINISHED! PERSON LOITERING DETECTED!", (50,100), cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
        arduino.write(f"{LASER_ON}\n".encode())



video_capture.release()
cv2.destroyAllWindows()