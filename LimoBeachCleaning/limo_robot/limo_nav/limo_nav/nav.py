import rclpy
import sys
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from builtin_interfaces.msg import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import ReliabilityPolicy, QoSProfile

PI = 3.14159265

class PotentialField(Node):
    def __init__(self, x_goal, y_goal):
        super().__init__('potential_field_node')
        self.goal_x = float(x_goal)
        self.goal_y = float(y_goal)
        self.get_logger().info(f'x = {self.goal_x} and y = {self.goal_y}')

        #self.reentrant_group_1 = ReentrantCallbackGroup()
        self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_3 = MutuallyExclusiveCallbackGroup()
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.att_pub = self.create_publisher(PoseStamped, 'attraction_vector', 10)
        self.rep_pub = self.create_publisher(PoseStamped, 'repulsion_vector', 10)
        self.fin_pub = self.create_publisher(PoseStamped, 'final_vector', 10)
        
        # Create Subscriber to the odometry and LiDAR
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=self.mutuallyexclusive_group_1)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.mutuallyexclusive_group_2)
        
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.controller)

        
        # Controller and other methods
        self.theta = 0.0
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.V_attraction = [0.0, 0.0]
        self.V_repulsion = [0.0, 0.0]

    def controller(self):
        x_final = self.V_attraction[0] + self.V_repulsion[0]
        y_final = self.V_attraction[1] + self.V_repulsion[1]

        final_vector = self.create_vector_pose(x_final, y_final)
        self.fin_pub.publish(final_vector)

        direction = Twist()
        tolerance = 1.0
        angle = math.atan2(y_final, x_final)
        delta = PI - abs((abs(angle - self.theta) % (2 * PI)) - PI)

        self.get_logger().info(f'angle to goal: {delta}')

        if delta < -tolerance:
            direction.angular.z = -0.2
            direction.linear.x = 0.0
        elif delta > tolerance:
            direction.angular.z = 0.2
            direction.linear.x = 0.0
        else:
            #direction.linear.x = 0.1
            direction.linear.x = 0.05
            direction.angular.z = 0.0

        self.cmd_pub.publish(direction)

    def create_vector_pose(self, x, y):
        vector = PoseStamped()
        vector.header.frame_id = "base_link"
        now = self.get_clock().now() 
        vector.header.stamp = Time(sec=int(now.nanoseconds // 1e9), nanosec=int(now.nanoseconds % 1e9))
        vector.pose.position.x = self.x_odom
        vector.pose.position.y = self.y_odom
        vector.pose.position.z = 0.0

        angle = math.atan2(y, x)
        q = quaternion_from_euler(0, 0, angle)
        quaternion_msg = Quaternion()
        quaternion_msg.x = q[0]
        quaternion_msg.y = q[1]
        quaternion_msg.z = q[2]
        quaternion_msg.w = q[3]
        vector.pose.orientation = quaternion_msg
        
        return vector

    def compute_attraction(self, x_a, y_a):
        distance = math.sqrt((x_a - self.x_odom)**2 + (y_a - self.y_odom)**2)
        x_a -= self.x_odom
        y_a -= self.y_odom
        Q_attraction = 100
        F_attraction = (Q_attraction) / (4 * PI * distance**2)
        self.V_attraction = [F_attraction * x_a, F_attraction * y_a]

        attraction_vector = self.create_vector_pose(self.V_attraction[0], self.V_attraction[1])
        self.att_pub.publish(attraction_vector)

    def odom_callback(self, msg):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.theta = yaw

        self.compute_attraction(self.goal_x, self.goal_y)

    def scan_callback(self, _msg):
        angle_min = _msg.angle_min
        angle_increment = _msg.angle_increment
        scan = _msg.ranges
        x_r = 0.0
        y_r = 0.0

        for i in range(len(scan)):
            if scan[i] < 100.0 and scan[i] > 0.1:
                Q_repulsion = 1
                Current_Q = (Q_repulsion) / (4 * PI * scan[i]**2)
                x_r -= Current_Q * math.cos(angle_min + self.theta + angle_increment * i)
                y_r -= Current_Q * math.sin(angle_min + self.theta + angle_increment * i)

        if len(scan) == 360:
            self.V_repulsion = [0.0001, 0.000000000001]
        else:
            self.V_repulsion = [x_r, y_r]

        repulsion_vector = self.create_vector_pose(self.V_repulsion[0], self.V_repulsion[1])
        self.rep_pub.publish(repulsion_vector)
        self.get_logger().info('laser callback')
        #self.controller()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print('Usage: ros2 run limo_nav nav x y')
        return
    
    x_goal = sys.argv[1]
    y_goal = sys.argv[2]

    potential_field = PotentialField(x_goal=x_goal, y_goal=y_goal)

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(potential_field)
    
    try:
        #rclpy.spin(potential_field)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        potential_field.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
