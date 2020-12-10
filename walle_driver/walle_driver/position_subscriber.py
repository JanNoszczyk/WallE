import rclpy
from rclpy.node import Node
from robot import WallE
from walle_msgs.msg import MotorPower


class PositionSubscriber(Node):

    def __init__(self, walle):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            MotorPower,
            'position',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.walle = walle

    def listener_callback(self, msg):

        self.walle.set_speeds(msg.left, msg.right)

        self.get_logger().info("I heard: {} {}".format(msg.left, msg.right))


def main(args=None):
    rclpy.init(args=args)
    
    walle = WallE()
    position_subscriber = PositionSubscriber(walle)

    rclpy.spin(position_subscriber)


    position_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
