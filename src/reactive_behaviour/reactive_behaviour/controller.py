import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn

PI = 3.1415

def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

def clamp(x, min, max=None):
    if x < min:
        return min
    elif x > max:
        return max
    else:
        return x

class VelocityController(DrivingSwarmNode):

    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()
        
    def timer_cb(self):
        if not self.started:
            return
        msg = Twist()
        fw_distance = self.distances[0]
        r_distance = self.distances[270]
        l_distance = self.distances[90]
        fr_distance = self.distances[340]
        fl_distance = self.distances[20]

        x = 0
        # x = sign(fw_distance) * max(fw_distance, 0.1)
        # clamp x between 0 and 0.1
        x = clamp(fw_distance - 0.3, -0.1, 0.1)
        msg.linear.x = x

        # rotate if x == 0
        if (self.close_to_wall(0.3, 0.1)):
            msg.angular.z = 10*PI/360

        if (self.close_to_wall(0.3, 0.05)):
            msg.angular.z = 25*PI/360

        if (l_distance < 0.4 or r_distance < 0.4):
            if (l_distance < r_distance):
                msg.angular.z = -msg.angular.z

        self.publisher.publish(msg)

    def close_to_wall(self, offset, threshold):
        o = offset + threshold
        for d in self.distances:
            if d < o:
                return True

        return False

    def laser_cb(self, msg):
        self.distances = msg.ranges

def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
