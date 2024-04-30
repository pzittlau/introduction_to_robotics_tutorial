import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WatchdogNode(Node):

    def __init__(self):
        super().__init__('watchdog')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'input_cmd', self.cmd_callback, 10)
        self.create_subscription(String, 'controller_cmd', self.controller_callback, 10)
        self.get_logger().info('Watchdog node started')
        self.state = '' # By default the turtle should not move

    def cmd_callback(self, msg):
        if self.state == 'start':
            # Normal operation: turtle moves based on the input command
            self.publisher.publish(msg)
            # self.get_logger().info(f'cmd_vel published with linear.x: {msg.linear.x}, angular.z: {msg.angular.z}')
        elif self.state == 'stop':
            # Stop all movements: set all velocities to zero
            msg.linear.x = float(0)
            msg.linear.y = float(0)
            msg.linear.z = float(0)
            msg.angular.x = float(0)
            msg.angular.y = float(0)
            msg.angular.z = float(0)

            self.publisher.publish(msg)
            # self.get_logger().info(f'cmd_vel published with all velocities set to zero {type(msg.linear)}')
        else:
            # Before 'start': only allow turning
            msg.linear.x = float(0)
            msg.linear.y = float(0)
            msg.linear.z = float(0)
            self.publisher.publish(msg)
            # self.get_logger().info(f'cmd_vel published with linear.x set to zero, angular.z: {msg.angular.z}')

    def controller_callback(self, msg):
        self.state = msg.data.lower()  # Update the state based on the controller command
        self.get_logger().warn(f'The controller says I should {msg.data} the turtle ...')



def main(args=None):
    rclpy.init(args=args)

    node = WatchdogNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
