import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn

PI = 3.1415

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
        if not self.started or not hasattr(self, 'distances'):
            return

        msg = Twist()
        f_distance = self.distances[0]
        r_distance = self.distances[270]
        fr_distance = self.distances[340]
        fl_distance = self.distances[20]

        msg.linear.x = clamp(f_distance - 0.3, -0.1, 0.3)

        # Wall-following behavior (right wall)
        desired_distance = 0.2  # Desired distance from the right wall
        max_turn_rate = PI/2
        if f_distance < 0.3:  # Obstacle in front
            msg.angular.z = max_turn_rate  # Turn left
        else:
            # Turn left to increase distance and right to decrease distance
            msg.angular.z = 1.5*clamp(desired_distance - r_distance, -max_turn_rate, max_turn_rate)
            # if r_distance < desired_distance:
            #     msg.angular.z = clamp(desired_distance - r_distance, 0, max_turn_rate)  # Turn left to increase distance
            # elif r_distance > desired_distance:
            #     msg.angular.z = clamp(desired_distance - r_distance, -max_turn_rate, 0)  # Turn right to decrease distance
            # else:
            #     msg.angular.z = 0.0  # Go straight

        # Additional adjustment based on front-left and front-right sensors
        if fr_distance < 0.4:
            msg.angular.z = max_turn_rate
        elif fl_distance < 0.4:
            msg.angular.z = -max_turn_rate

        # Publish the command
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

# [robotA.scoring_node]: score: 27.11 at t=120.0s
# [robotB.scoring_node]: score: 26.44 at t=120.0s
