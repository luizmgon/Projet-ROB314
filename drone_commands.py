from djitellopy import Tello
import keyboard
import time

# Inicializa o drone
tello = Tello()
tello.connect()

tello.set_speed(50)

# Aguarda o usuário pressionar 's' para decolagem
print("Pressione 's' para iniciar a decolagem...")
while True:
    if keyboard.is_pressed('s'):
        print("Decolando...")
        tello.takeoff()
        break
    time.sleep(0.1)  # Pequena pausa para evitar alto uso da CPU

print("Use W, A, S, D para mover, E/Q para girar, R/F para subir/descer, ESC para pousar.")

while True:
    event = keyboard.read_event()

    if event.event_type == keyboard.KEY_DOWN:  # Captura apenas o pressionamento da tecla
        key = event.name  # Obtém o nome da tecla pressionada

        if key == 'esc':  # ESC -> pousar e sair
            print("Pousando...")
            tello.land()
            break
        elif key == 'w':
            print("Movendo para frente")
            # tello.move_forward(30)
            tello.go_xyz_speed(30, 0, 0, 100)
        elif key == 's':
            print("Movendo para trás")
            # tello.move_back(30)
            tello.go_xyz_speed(-30, 0, 0, 50)

        elif key == 'a':
            print("Movendo para a esquerda")
            tello.move_left(30)
        elif key == 'd':
            print("Movendo para a direita")
            tello.move_right(30)
        elif key == 'e':
            print("Girando no sentido horário")
            tello.rotate_clockwise(30)
        elif key == 'q':
            print("Girando no sentido anti-horário")
            tello.rotate_counter_clockwise(30)
        elif key == 'r':
            print("Subindo")
            tello.move_up(30)
        elif key == 'f':
            print("Descendo")
            tello.move_down(30)

        print(f"X:{tello.get_speed_x()} Y:{tello.get_speed_x()} Z:{tello.get_speed_x()}")

print("Programa encerrado.")
