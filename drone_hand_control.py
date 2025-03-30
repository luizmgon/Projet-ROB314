from djitellopy import Tello
import time
import keyboard  # Biblioteca para capturar pressionamento de teclas

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

def run():

    global send_rc_control

    should_stop = False
    while not should_stop:
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
            should_stop = True

        # Send velocities to the drone
        if send_rc_control:
            tello.send_rc_control(left_right_velocity, for_back_velocity,
                up_down_velocity, yaw_velocity)

        time.sleep(1 / 120)  # Sleep to simulate FPS delay

    # Deallocate resources before finishing
    tello.end()

def keydown(key):

    global S
    global for_back_velocity
    global left_right_velocity
    global yaw_velocity
    global send_rc_control
    global up_down_velocity

    if key == 'up':  # set forward velocity
        for_back_velocity = S
    elif key == 'down':  # set backward velocity
        for_back_velocity = -S
    elif key == 'left':  # set left velocity
        left_right_velocity = -S
    elif key == 'right':  # set right velocity
        left_right_velocity = S
    elif key == 'w':  # set up velocity
        up_down_velocity = S
    elif key == 's':  # set down velocity
        up_down_velocity = -S
    elif key == 'a':  # set yaw counter clockwise velocity
        yaw_velocity = -S
    elif key == 'd':  # set yaw clockwise velocity
        yaw_velocity = S
    elif key == 't':  # takeoff
        tello.takeoff()
        send_rc_control = True
    elif key == 'l':  # land
        tello.land()
        send_rc_control = False
    elif key == '1':  
        S -= 10
        if S < 0:
            S = 0
    elif key == '2':
        S += 10
        if S > 100:
            S = 100

    # Send velocities to the drone
    if send_rc_control:
        tello.send_rc_control(left_right_velocity, for_back_velocity,
            up_down_velocity, yaw_velocity)

def keyup(key):
    """ Reset velocities when key is released
    Arguments:
        key: The key released
    """

    global S
    global for_back_velocity
    global left_right_velocity
    global yaw_velocity
    global send_rc_control
    global up_down_velocity

    if key == 'up' or key == 'down':  # set zero forward/backward velocity
        for_back_velocity = 0
    elif key == 'left' or key == 'right':  # set zero left/right velocity
        left_right_velocity = 0
    elif key == 'w' or key == 's':  # set zero up/down velocity
        up_down_velocity = 0
    elif key == 'a' or key == 'd':  # set zero yaw velocity
        yaw_velocity = 0

    # Send velocities to the drone
    if send_rc_control:
        tello.send_rc_control(left_right_velocity, for_back_velocity,
            up_down_velocity, yaw_velocity)

def main():
    run()

if __name__ == '__main__':
    main()
