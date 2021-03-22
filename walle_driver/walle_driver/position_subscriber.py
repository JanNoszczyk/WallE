import rclpy
from rclpy.node import Node
from walle_driver.motor_controller import MotorController
from walle_msgs.msg import MotorPower


class PositionSubscriber(Node):

    def __init__(self, motors):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            MotorPower,
            'position',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.motors = motors

    def listener_callback(self, msg):

        self.motors.set_speeds(msg.left, msg.right)

        self.get_logger().info("I heard: {} {}".format(msg.left, msg.right))


def main(args=None):
    rclpy.init(args=args)
    
    motors = MotorController()
    position_subscriber = PositionSubscriber(motors)

    rclpy.spin(position_subscriber)


    position_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
