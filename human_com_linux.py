import cv2
cv2.namedWindow('Hand Tracking', cv2.WINDOW_NORMAL)

from djitellopy import Tello
import mediapipe as mp
import numpy as np
from collections import defaultdict, deque
import matplotlib.pyplot as plt
import time
from pynput import keyboard



# Speed of the drone
S = 20

# Init Tello object that interacts with the Tello drone
tello = Tello()

# Drone velocities between -100~100
for_back_velocity = 0
left_right_velocity = 0
up_down_velocity = 0
yaw_velocity = 0
speed = 10

send_rc_control = False

# Store the state of keys (pressed or not)
pressed_keys = set()

def on_press(key):
    global send_rc_control, should_continue
    try:
        if key.char == 't':
            tello.takeoff()
            send_rc_control = True
        elif key.char == 'l':
            tello.land()
            send_rc_control = False
        elif key.char == '\x1b':  # ESC key
            tello.land()
            send_rc_control = False
            should_continue = False
    except AttributeError:
        pass

def on_release(key):
    pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

tello.connect()
tello.set_speed(speed)

FPS = 120
ACTIVE_PLOT = False

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

N = 10
hand_trails = defaultdict(lambda: deque(maxlen=N))
hand_lost_counter = defaultdict(int)
hand_positions = {}
next_hand_id = 0
max_lost_frames = 10
distance_threshold = 200

x_speeds = []
y_speeds = []

should_continue = True

def send_drone_speeds(x_speed, y_speed):
    global for_back_velocity, left_right_velocity, yaw_velocity, send_rc_control, up_down_velocity
    new_x_speed = int(x_speed / 2)
    new_x_speed = max(-100, min(100, new_x_speed))
    new_y_speed = int(-y_speed / 2)
    new_y_speed = max(-100, min(100, new_y_speed))
    left_right_velocity = new_x_speed
    up_down_velocity = new_y_speed
    print(f"Sending x: {new_x_speed} ({x_speed})  y : {y_speed}")
    if send_rc_control:
        tello.send_rc_control(left_right_velocity, for_back_velocity, up_down_velocity, yaw_velocity)

def find_closest_hand(new_center, existing_hands):
    closest_id = None
    min_distance = distance_threshold
    for hand_id, prev_center in existing_hands.items():
        distance = np.linalg.norm(np.array(new_center) - np.array(prev_center))
        if distance < min_distance:
            min_distance = distance
            closest_id = hand_id
    return closest_id

with mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5, max_num_hands=2) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb_frame)
        hands_detected = {}
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                center_x = (int(hand_landmarks.landmark[9].x * frame.shape[1]) + int(hand_landmarks.landmark[0].x * frame.shape[1])) // 2
                center_y = (int(hand_landmarks.landmark[9].y * frame.shape[0]) + int(hand_landmarks.landmark[0].y * frame.shape[0])) // 2
                new_center = (center_x, center_y)
                hand_id = find_closest_hand(new_center, hand_positions)
                if hand_id is None:
                    hand_id = next_hand_id
                    next_hand_id += 1
                    hand_trails[hand_id] = deque(maxlen=N)
                hand_positions[hand_id] = new_center
                hands_detected[hand_id] = new_center
                hand_trails[hand_id].append(new_center)
                hand_lost_counter[hand_id] = 0
                cv2.circle(frame, new_center, 5, (0, 0, 255), -1)
                cv2.putText(frame, f'({center_x}, {center_y})', (center_x - 20, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        for hand_id in list(hand_trails.keys()):
            if hand_id not in hands_detected:
                hand_lost_counter[hand_id] += 1
                if hand_lost_counter[hand_id] > max_lost_frames:
                    del hand_trails[hand_id]
                    del hand_positions[hand_id]
                    del hand_lost_counter[hand_id]
        cv2.imshow('Hand Tracking', frame)
        for trail in hand_trails.values():
            vector = np.array(trail[-1]) - np.array(trail[0])
            cv2.arrowedLine(frame, trail[-1], trail[-1] + vector, (0, 255, 0), 2)
            x_component = 0
            y_component = 0
            x_speeds.append(x_component)
            y_speeds.append(y_component)
            send_drone_speeds(x_component, y_component)
            time.sleep(1/FPS)
        cv2.imshow('Hand Tracking', frame)
        if cv2.getWindowProperty('Hand Tracking', cv2.WND_PROP_VISIBLE) < 1:
            cap.release()
            cv2.destroyAllWindows()
            exit()
        if not should_continue:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
tello.end()
