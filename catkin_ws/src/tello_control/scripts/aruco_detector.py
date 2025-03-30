#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from cv2 import aruco

# Inicializa o nó do ROS
rospy.init_node('aruco_detector_node')

# Inicializa o CvBridge para conversão entre OpenCV e ROS
bridge = CvBridge()

# Definir o dicionário ArUco e os parâmetros do detector
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Parâmetros da câmera (ajuste conforme sua câmera)
fx = 100  # Focal length x (em pixels)
fy = 100  # Focal length y (em pixels)
cx = 160  # Centro da imagem x
cy = 120  # Centro da imagem y

# Matriz de calibração da câmera
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])

# Coeficientes de distorção (ajuste conforme sua câmera)
dist_coeffs = np.zeros((4, 1))  # Supondo que não haja distorção

# Publisher para a posição (x, y, z) do marcador ArUco
position_pub = rospy.Publisher('/tello/aruco_position', Point, queue_size=10)

frame = None

# Callback para o tópico /tello/image
def image_callback(msg):

    global frame

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"Erro ao converter a imagem: {e}")
        return
    
    # Converte a imagem para escala de cinza
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detecta os marcadores ArUco
    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is not None:
        # Desenha os marcadores detectados
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Estima a posição do marcador em relação à câmera
        marker_size = 17.5  # Tamanho do marcador em cm
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        # Para o primeiro marcador detectado
        tvec = tvecs[0]

        # Desenha os eixos 3D do marcador
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[0], tvec, 0.1)

        # Cria a mensagem de posição
        position_msg = Point()
        position_msg.x = tvec[0][0]  # Coordenada X em cm
        position_msg.y = tvec[0][1]  # Coordenada Y em cm
        position_msg.z = tvec[0][2]  # Coordenada Z em cm

        rospy.loginfo(f"Posição do marcador ArUco: x={position_msg.x:.1f} cm, y={position_msg.y:.1f} cm, z={position_msg.z:.1f} cm")

        # Publica a posição do marcador ArUco
        position_pub.publish(position_msg)

    else:
        # Publishes a default position if no markers are detected 0,0,0
        position_msg = Point()
        position_msg.x = 0
        position_msg.y = 0
        position_msg.z = 0
        position_pub.publish(position_msg)

# Subscribe ao tópico /tello/image
rospy.Subscriber('/tello/image', Image, image_callback)

rate = rospy.Rate(100)  # 10 Hz

while not rospy.is_shutdown():

    # Exibe o frame com os marcadores e eixos desenhados
    if frame is not None:
        cv2.imshow("Tello Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rate.sleep()

# Finaliza a exibição de janelas OpenCV
cv2.destroyAllWindows()
