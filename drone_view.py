import cv2
from djitellopy import Tello

tello = Tello()
tello.connect()
tello.set_video_direction(Tello.CAMERA_DOWNWARD)
# tello.set_video_direction(Tello.CAMERA_FORWARD)  
tello.streamon()
frame_read = tello.get_frame_read()
# cv2.resize(frame_read.frame, (640, 480))
# tello.takeoff()

while True:
    # Captura o frame atual
    frame = frame_read.frame[0:240, :, :]

    print(frame.shape)

    # Exibe o frame em tempo real
    cv2.imshow("Tello Camera", frame)

    # Espera por uma tecla para sair
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Salva a imagem
cv2.imwrite("picture.png", frame)

tello.land()
cv2.destroyAllWindows()
