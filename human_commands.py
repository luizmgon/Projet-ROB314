import cv2
import mediapipe as mp
import numpy as np
from collections import defaultdict, deque

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
        
        # Flip the frame for a mirror effect
        frame = cv2.flip(frame, 1)
        
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
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
        
        # Draw the tracked points for each hand
        for trail in hand_trails.values():
            for j in range(1, len(trail)):
                cv2.line(frame, trail[j - 1], trail[j], (255, 0, 0), 2)
        
        # Show the frame
        cv2.imshow('Hand Tracking', frame)
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cap.release()
cv2.destroyAllWindows()
