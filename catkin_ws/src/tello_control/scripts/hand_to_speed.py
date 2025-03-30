#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pynput import keyboard
import cv2
import mediapipe as mp
import numpy as np
from collections import defaultdict, deque

# Initialize ROS node
rospy.init_node('hand_to_speed_node')

# Setup publisher to send velocity commands to tello_driver
speed_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)

# MediaPipe setup for hand tracking
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Buffer to store hand positions
N_points = 10
hand_trail = deque(maxlen=N_points)
max_lost_frames = 5
lost_hand_count = 0

def send_drone_speeds(x_speed, y_speed):
    # Create a Twist message to publish to the cmd_vel topic
    twist = Twist()
    twist.linear.x = x_speed
    twist.linear.y = y_speed
    speed_pub.publish(twist)

def is_hand_closed(hand_landmarks, frame_shape):

    # Calculate vectors and distances
    vector_0_12 = np.array([
        hand_landmarks.landmark[0].x - hand_landmarks.landmark[12].x,
        hand_landmarks.landmark[0].y - hand_landmarks.landmark[12].y
    ])
    vector_0_9 = np.array([
        hand_landmarks.landmark[0].x - hand_landmarks.landmark[9].x,
        hand_landmarks.landmark[0].y - hand_landmarks.landmark[9].y
    ])
    distance_0_12 = np.linalg.norm(vector_0_12)
    distance_0_9 = np.linalg.norm(vector_0_9)

    # Determine if the hand is closed
    is_closed = distance_0_12 < distance_0_9
    if is_closed:
        print("Hand is closed")
    return is_closed
    
def process_hand_frame(frame):
    global lost_hand_count

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb_frame)

    hand_is_closed = False

    # Process the frame if hands are detected
    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            # Calculate the center of the palm
            center_x = (int(hand_landmarks.landmark[9].x * frame.shape[1]) + int(hand_landmarks.landmark[0].x * frame.shape[1])) // 2
            center_y = (int(hand_landmarks.landmark[9].y * frame.shape[0]) + int(hand_landmarks.landmark[0].y * frame.shape[0])) // 2
            new_center = (center_x, center_y)

            hand_is_closed = is_hand_closed(hand_landmarks, frame.shape)

            if(not hand_is_closed):
                hand_trail.append(new_center)

            # Draw the center point and its coor
            cv2.circle(frame, new_center, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"{center_x}, {center_y}", new_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            print("Hand detected at:", new_center)
    if not result.multi_hand_landmarks or hand_is_closed:
        #Ros print no hands 
        print("No hands detected or hand is closed")
        lost_hand_count += 1
        if lost_hand_count > max_lost_frames:
            hand_trail.clear()
            lost_hand_count = 0

    x_component = 0
    y_component = 0

    if(len(hand_trail) > 1):
        vector = np.array(hand_trail[-1]) - np.array(hand_trail[0])
        cv2.arrowedLine(frame, hand_trail[-1], hand_trail[-1] + vector, (0, 255, 0), 2)

        # Get x and y components of the vector
        x_component = vector[0]
        y_component = vector[1]

    send_drone_speeds(x_component, y_component)

    return frame


def main():
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # Process hand detection
        frame = process_hand_frame(frame)
        cv2.imshow("Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
