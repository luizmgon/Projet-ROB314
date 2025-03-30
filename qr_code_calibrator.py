import cv2
import cv2.aruco as aruco
from djitellopy import Tello
import numpy as np

# Inicializa o drone
tello = Tello()
tello.connect()
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

while True:
    # Captura o frame atual
    frame = frame_read.frame[0:240, :, :]
    
    # Converte para tons de cinza
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detecta os marcadores ArUco
    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
    
    if ids is not None:
        # Desenha os marcadores detectados
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Estima a posição do marcador em relação à câmera
        # Ajuste para 0.2 metros (20 cm) se o tamanho do marcador for 20 cm
        marker_size = 17.5  # Tamanho do marcador em cm
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        
        # Para cada marcador detectado
        for i in range(len(ids)):
            rvec = rvecs[i]
            tvec = tvecs[i]
            
            # Desenha os eixos (3D) do marcador
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
            
            # Imprime a posição estimada (translação) com duas casas decimais
            # A translação (tvecs) já está em metros, então, se precisar em centímetros, multiplique por 100
            print(f"Posição do ArUco marker {ids[i]}: x={tvec[0][0]:.1f} cm, y={tvec[0][1]:.1f} cm, z={tvec[0][2]:.1f} cm")
    
    # Exibe o frame em tempo real
    cv2.imshow("Tello Camera", frame)
    
    # Espera por uma tecla para sair
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Salva a imagem
cv2.imwrite("picture.png", frame)

# Finaliza operações
tello.end()
cv2.destroyAllWindows()
