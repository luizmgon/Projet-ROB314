#!/usr/bin/env python

import cv2
cv2.namedWindow("Tello Camera", cv2.WINDOW_NORMAL)
import rospy
from djitellopy import Tello
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import time

# Inicializa o nó do ROS
rospy.init_node('tello_driver_node')

# Inicializa o objeto Tello
tello = Tello()
tello.connect()
tello.set_speed(10)

# Controle proporcional
ARUCO_POSITION_SCALE = 1  # Fator de escala, ajustável conforme necessário

# Posição do ArUco
aruco_position_ref = 0

send_rc_control = False
finish = False

# Inicializa o CvBridge para converter OpenCV para ROS Image
bridge = CvBridge()

# Inicia o fluxo de vídeo
tello.set_video_direction(Tello.CAMERA_DOWNWARD)
tello.streamon()

# Subscriber para a posição do ArUco
def aruco_position_callback(msg):
    global aruco_position_ref
    aruco_position_ref = -msg.y  # Assume que a posição do ArUco é recebida em 'z' (ajuste se necessário)

# Subscriber para controlar o drone com base em velocidade
def velocity_callback(msg):
    x = msg.linear.x
    y = msg.linear.y

    # Controle proporcional para o eixo Z com base na posição do ArUco
    z = ARUCO_POSITION_SCALE * aruco_position_ref  # Proporcional à posição do ArUco

    for_back_velocity = -int(x)
    up_down_velocity = -int(y)
    left_right_velocity = int(z)

    # Envia sinais de controle para o Tello
    if send_rc_control:
        print(f"Sending RC control: ({left_right_velocity}, {for_back_velocity}, {up_down_velocity}, 0)")
        tello.send_rc_control(left_right_velocity, for_back_velocity,
            up_down_velocity, 0)

# Subscriber para comandos de controle (decolar, aterrissar, etc.)
def control_callback(msg):
    global finish
    global send_rc_control
    if msg.data == "takeoff":
        tello.takeoff()
        send_rc_control = True
    elif msg.data == "land":
        tello.land()
        send_rc_control = False
    elif msg.data == "finish":
        tello.streamoff()
        tello.end()
        finish = True

# Função para capturar a imagem da câmera do Tello e publicá-la
def capture_and_publish_image():
    if not finish:
        frame_read = tello.get_frame_read()
        
        # Captura o frame atual da câmera
        frame_drone = frame_read.frame[0:240, :, :]

        # Exibe o frame na tela
        cv2.imshow("Tello Camera", frame_drone)

        # Converte a imagem para o formato ROS Image e publica
        try:
            image_msg = bridge.cv2_to_imgmsg(frame_drone, "bgr8")
            image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting frame: {e}")

# Publisher para enviar a imagem capturada
image_pub = rospy.Publisher('/tello/image', Image, queue_size=10)

# Subscribe aos tópicos
rospy.Subscriber('/tello/cmd_vel', Twist, velocity_callback)
rospy.Subscriber('/tello/control', String, control_callback)
rospy.Subscriber('/tello/aruco_position', Point, aruco_position_callback)  # Assumindo que a posição é do tipo Point

# Loop para capturar e publicar imagens enquanto o nó está rodando
rate = rospy.Rate(100)  # 10 Hz, ajustável conforme necessário
while not rospy.is_shutdown():
    capture_and_publish_image()

    key = cv2.waitKey(10)& 0xFF
    if key == ord('q') or finish:  # Pressione 'q' para sair
        break

    rate.sleep()

cv2.destroyAllWindows()

