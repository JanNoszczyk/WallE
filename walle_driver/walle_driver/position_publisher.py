import rclpy
from rclpy.node import Node
from time import sleep
import math
from approxeng.input.selectbinder import ControllerResource
from walle_msgs.msg import MotorPower


class PositionPublisher(Node):

    def __init__(self, joystick):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(MotorPower, 'position', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joystick = joystick

    def timer_callback(self):
        msg = MotorPower()

        # Get joystick values from the left analogue stick
        x_axis, y_axis = self.joystick['lx', 'ry']
        # Get power from mixer function
        power_left, power_right = self.steering(x=y_axis, y=x_axis)
        msg.left = power_left
        msg.right = power_right
        self.publisher_.publish(msg)
        
        self.joystick.check_presses()
        if 'home' in self.joystick.presses:
            raise RobotStopException()
    
        self.get_logger().info("Publishing: {} {}".format(msg.left, msg.right))

    def mixer(self, yaw, throttle, max_power=100):
        left = throttle + yaw
        right = throttle - yaw
        scale = float(max_power) / max(1, abs(left), abs(right))

        return float(left * scale), float(right * scale)
    
    def steering(self, x, y, max_power=100):
        # convert to polar
        r = math.hypot(x, -y)
        t = math.atan2(-y, x)

        # rotate by 45 degrees
        t += math.pi / 4

        # back to cartesian
        left = r * math.cos(t)
        right = r * math.sin(t)

        # rescale the new coords
        left = left * math.sqrt(2)
        right = right * math.sqrt(2)

        # clamp to -1/+1
        left = max(-1, min(left, 1))
        right = max(-1, min(right, 1))

        return float(max_power*left), float(max_power*right)


def main(args=None):
    rclpy.init(args=args)

    try:
        with ControllerResource(dead_zone=0.1, hot_zone=0.2) as joystick:

            position_publisher = PositionPublisher(joystick)
            rclpy.spin(position_publisher)

    except IOError:
        print('No controller found yet')
        sleep(1)


    position_publisher.destroy_node()
    rclpy.shutdown()

class RobotStopException(Exception):
    pass

if __name__ == '__main__':
    main()
