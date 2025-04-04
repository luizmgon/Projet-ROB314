import cv2
cv2.namedWindow('Hand Tracking', cv2.WINDOW_NORMAL)
from djitellopy import Tello
import mediapipe as mp
import numpy as np
from collections import defaultdict, deque
import matplotlib.pyplot as plt
import time
import keyboard

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

# Connect to Tello drone
tello.connect()
tello.set_speed(speed)


FPS = 120

ACTIVE_PLOT = False

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Open webcam
cap = cv2.VideoCapture(0)

# Buffer to store last N points per hand
N = 10
hand_trails = defaultdict(lambda: deque(maxlen=N))
hand_lost_counter = defaultdict(int)
hand_positions = {}  # Stores last known positions of hands
next_hand_id = 0  # Unique identifier for hands
max_lost_frames = 10  # Frames to tolerate before removing a hand
distance_threshold = 200  # Max distance to associate a new detection to an existing hand

x_speeds = []
y_speeds = []

should_continue = True

def send_drone_speeds(x_speed, y_speed):
    global for_back_velocity
    global left_right_velocity
    global yaw_velocity
    global send_rc_control
    global up_down_velocity

    new_x_speed = int(x_speed / 2)
    if(new_x_speed > 100): new_x_speed = 100
    if(new_x_speed < -100): new_x_speed = -100

    new_y_speed = int(-y_speed / 2)
    if(new_y_speed > 100): new_y_speed = 100
    if(new_y_speed < -100): new_y_speed = -100

    left_right_velocity = new_x_speed
    up_down_velocity = new_y_speed


    print(f"Sending x: {new_x_speed} ({x_speed})  y : {y_speed}")


    global send_rc_control
    global should_continue

    # Check for key presses and handle accordingly
    if keyboard.is_pressed('t'):  # Detect if the key is pressed
        tello.takeoff()
        send_rc_control = True

    elif keyboard.is_pressed('l'):
        tello.land()
        send_rc_control = False

    elif keyboard.is_pressed('esc'):
        tello.land()
        send_rc_control = False
        should_continue = False

    # Send velocities to the drone
    if send_rc_control:
        tello.send_rc_control(left_right_velocity, for_back_velocity,
            up_down_velocity, yaw_velocity)
        

def plot_speed_graphs(x_speeds, y_speeds):
    # Draw real-time speed graphs
    plt.clf()
    plt.subplot(2, 1, 1)
    plt.plot(x_speeds, label='X Speed')
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(y_speeds, label='Y Speed')
    plt.legend()
    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)

# Function to find the closest existing hand
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
        print("Debug 1")
        frame = cv2.flip(frame, 1)         # Flip the frame for a mirror effect
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)      # Convert BGR to RGB

        # Process the frame with MediaPipe Hands
        result = hands.process(rgb_frame)
        
        # Track detected hands
        hands_detected = {}
        
        # If hands are detected
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Calculate the center of the palm
                center_x = (int(hand_landmarks.landmark[9].x * frame.shape[1]) + int(hand_landmarks.landmark[0].x * frame.shape[1])) // 2
                center_y = (int(hand_landmarks.landmark[9].y * frame.shape[0]) + int(hand_landmarks.landmark[0].y * frame.shape[0])) // 2
                new_center = (center_x, center_y)
                
                # Find the closest existing hand
                hand_id = find_closest_hand(new_center, hand_positions)
                
                if hand_id is None:
                    hand_id = next_hand_id
                    next_hand_id += 1
                    hand_trails[hand_id] = deque(maxlen=N)
                
                hand_positions[hand_id] = new_center
                hands_detected[hand_id] = new_center
                hand_trails[hand_id].append(new_center)
                hand_lost_counter[hand_id] = 0  # Reset lost frame counter
                
                # Draw the center point and coordinates
                cv2.circle(frame, new_center, 5, (0, 0, 255), -1)
                cv2.putText(frame, f'({center_x}, {center_y})', (center_x - 20, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Handle lost hands
        for hand_id in list(hand_trails.keys()):
            if hand_id not in hands_detected:
                hand_lost_counter[hand_id] += 1
                if hand_lost_counter[hand_id] > max_lost_frames:
                    del hand_trails[hand_id]
                    del hand_positions[hand_id]
                    del hand_lost_counter[hand_id]

        # show the frame
        cv2.imshow('Hand Tracking', frame)


        print("Debug 2")

        # Draw the tracked points for each hand
        for trail in hand_trails.values():
            # for j in range(1, len(trail)):
            #     cv2.line(frame, trail[j - 1], trail[j], (255, 0, 0), 2)

            # Get vector between first and last point
            vector = np.array(trail[-1]) - np.array(trail[0])
            cv2.arrowedLine(frame, trail[-1], trail[-1] + vector, (0, 255, 0), 2)

            # Get x and y components of the vector
            # x_component = vector[0]
            
            x_component = 0
            y_component = 0

            x_speeds.append(x_component)
            y_speeds.append(y_component)

            if ACTIVE_PLOT:
                # Plot the speed graphs
                plot_speed_graphs(x_speeds, y_speeds)

            send_drone_speeds(x_component, y_component)

            time.sleep(1/FPS)


        # Show the frame
        cv2.imshow('Hand Tracking', frame)

        # Check if the OpenCV window was closed
        if cv2.getWindowProperty('Hand Tracking', cv2.WND_PROP_VISIBLE) < 1:
            print("Hand Tracking window closed. Stopping program.")
            cap.release()
            cv2.destroyAllWindows()
            exit()

        if(not should_continue): break
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cap.release()
cv2.destroyAllWindows()
tello.end()
