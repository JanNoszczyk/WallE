from time import sleep
from approxeng.input.selectbinder import ControllerResource
from robot import WallE

class RobotStopException(Exception):
    pass


def mixer(yaw, throttle, max_power=100):
    """
    Mix a pair of joystick axes, returning a pair of wheel speeds. This is where the mapping from
    joystick positions to wheel powers is defined, so any changes to how the robot drives should
    be made here, everything else is really just plumbing.
    
    :param yaw: 
        Yaw axis value, ranges from -1.0 to 1.0
    :param throttle: 
        Throttle axis value, ranges from -1.0 to 1.0
    :param max_power: 
        Maximum speed that should be returned from the mixer, defaults to 100
    :return: 
        A pair of power_left, power_right integer values to send to the motor driver
    """
    yaw = yaw/1.5
    left = throttle + yaw
    right = throttle - yaw
    scale = float(max_power) / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


walle = WallE()

try:
    while True:
        try:
            with ControllerResource(dead_zone=0.1, hot_zone=0.2) as joystick:
                print('Controller found, press HOME button to exit, use left stick to drive.')
                print(joystick.controls)
                while joystick.connected:
                    # Get joystick values from the left analogue stick
                    x_axis, y_axis = joystick['lx', 'ly']
                    # Get power from mixer function
                    power_left, power_right = mixer(yaw=x_axis, throttle=y_axis)
                    print(power_left, power_right)
                    sleep(0.1)
                    walle.set_speeds(power_left, power_right)

                    joystick.check_presses()
                    if joystick.has_presses:
                        print(joystick.presses)
                    if 'home' in joystick.presses:
                        raise RobotStopException()
        except IOError:
            print('No controller found yet')
            sleep(1)

except RobotStopException:
    walle.stop_motors()
