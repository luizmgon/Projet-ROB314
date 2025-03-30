import cv2
cv2.namedWindow('Hand Tracking', cv2.WINDOW_NORMAL)
cv2.namedWindow("Tello Camera", cv2.WINDOW_NORMAL)
import cv2.aruco as aruco
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

# Connect to Tello drone
tello.connect()
tello.set_speed(speed)
tello.set_video_direction(Tello.CAMERA_DOWNWARD)
tello.streamon()
frame_read = tello.get_frame_read()

# Define o dicionário de marcadores ArUco e os parâmetros do detector
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Parâmetros da câmera (ajuste conforme sua câmera)
# Esses valores podem ser calibrados ou ajustados manualmente para sua câmera.
# Exemplo de parâmetros de uma câmera simples:
fx = 100  # Focal length x (em pixels)
fy = 100  # Focal length y (em pixels)
cx = 160  # Centro da imagem x
cy = 120  # Centro da imagem y

# Matriz de calibração da câmera (Apenas um exemplo, ajuste conforme sua câmera)
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])

# Distortion coefficients (ajuste conforme sua câmera)
dist_coeffs = np.zeros((4, 1))  # Supondo que não haja distorção


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
hand_is_closed = False


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

def send_drone_speeds(x_speed, y_speed, z_speed):
    global for_back_velocity
    global left_right_velocity
    global yaw_velocity
    global send_rc_control
    global up_down_velocity

    new_x_speed = int(-x_speed / 2)
    if(new_x_speed > 100): new_x_speed = 100
    if(new_x_speed < -100): new_x_speed = -100

    new_y_speed = int(-y_speed / 2)
    if(new_y_speed > 100): new_y_speed = 100
    if(new_y_speed < -100): new_y_speed = -100

    for_back_velocity = new_x_speed
    up_down_velocity = new_y_speed
    left_right_velocity = int(z_speed)


    print(f"Sending x: {new_x_speed} ({x_speed})  y : {new_y_speed} ({y_speed})  z : {z_speed}")

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

                # Check if the hand is closed by checking if 0-12 distance is smaller than 0-9
                # Vector between 0 and 12   

                vector_0_12 = np.array([hand_landmarks.landmark[0].x - hand_landmarks.landmark[12].x,
                                        hand_landmarks.landmark[0].y - hand_landmarks.landmark[12].y])
                vector_0_9 = np.array([hand_landmarks.landmark[0].x - hand_landmarks.landmark[9].x,
                                        hand_landmarks.landmark[0].y - hand_landmarks.landmark[9].y])
                distance_0_12 = np.linalg.norm(vector_0_12)
                distance_0_9 = np.linalg.norm(vector_0_9)
                if distance_0_12 < distance_0_9:
                    hand_is_closed = True
                    print("Hand is closed")
                else:
                    hand_is_closed = False
                
                
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

        x_component = 0
        y_component = 0
        z_component = 0
        
        # Draw the tracked points for each hand
        for trail in hand_trails.values():
            # for j in range(1, len(trail)):
            #     cv2.line(frame, trail[j - 1], trail[j], (255, 0, 0), 2)

            # Get vector between first and last point
            if not hand_is_closed and len(trail) > 1:
                vector = np.array(trail[-1]) - np.array(trail[0])
                cv2.arrowedLine(frame, trail[-1], trail[-1] + vector, (0, 255, 0), 2)

                # Get x and y components of the vector
                x_component = vector[0]
                y_component = vector[1]

                x_speeds.append(x_component)
                y_speeds.append(y_component)

            # Captura o frame atual
        frame_drone = frame_read.frame[0:240, :, :]

        #flip frame 90 degrees
        # frame_drone = cv2.rotate(frame_drone, cv2.ROTATE_90_CLOCKWISE)
        
        # Converte para tons de cinza
        gray = cv2.cvtColor(frame_drone, cv2.COLOR_BGR2GRAY)
        
        # Detecta os marcadores ArUco
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        
        if ids is not None:
            # Desenha os marcadores detectados
            aruco.drawDetectedMarkers(frame_drone, corners, ids)
            
            # Estima a posição do marcador em relação à câmera
            # Ajuste para 0.2 metros (20 cm) se o tamanho do marcador for 20 cm
            marker_size = 17.5  # Tamanho do marcador em cm
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
            
            # Para o primeiro marcador detectado

            rvec = rvecs[0]
            tvec = tvecs[0]
            
            # Desenha os eixos (3D) do marcador
            cv2.drawFrameAxes(frame_drone, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
            
            # Imprime a posição estimada (translação) com duas casas decimais
            # A translação (tvecs) já está em metros, então, se precisar em centímetros, multiplique por 100
            print(f"Posição do ArUco marker {ids[0]}: x={tvec[0][0]:.1f} cm, y={tvec[0][1]:.1f} cm, z={tvec[0][2]:.1f} cm")

            # x_component = 1*tvec[0][0]
            # y_component = 3*tvec[0][1]

            z_component = -1*tvec[0][1]    
            
        # Exibe o frame em tempo real
        cv2.imshow("Tello Camera", frame_drone)

        if ACTIVE_PLOT:
            # Plot the speed graphs
            plot_speed_graphs(x_speeds, y_speeds)

        send_drone_speeds(x_component, y_component, z_component)

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
