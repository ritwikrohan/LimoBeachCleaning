import rclpy
import sys
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, Vector3Stamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from builtin_interfaces.msg import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import ReliabilityPolicy, QoSProfile

PI = 3.14159265

class PotentialField(Node):
    def __init__(self):
        super().__init__('potential_field_node')

        self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()
        #self.mutuallyexclusive_group_3 = MutuallyExclusiveCallbackGroup()
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.att_pub = self.create_publisher(PoseStamped, 'attraction_vector', 10)
        self.rep_pub = self.create_publisher(PoseStamped, 'repulsion_vector', 10)
        self.fin_pub = self.create_publisher(PoseStamped, 'final_vector', 10)
        
        # Create Subscriber to the odometry and LiDAR
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.mutuallyexclusive_group_2)
        
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.controller)

        
        # Controller and other methods
        self.V_attraction = [30.0, 0.0]
        self.attraction_vector = self.create_vector_pose(self.V_attraction[0], self.V_attraction[1])
        self.V_repulsion = [0.0, 0.0]
        self.twist = Twist()
        self.repulsion_vector = PoseStamped()
        

    def controller(self):
        
        self.x_final = self.V_attraction[0] + self.V_repulsion[0]
        self.y_final = self.V_attraction[1] + self.V_repulsion[1]
        self.final_vector = self.create_vector_pose(self.x_final, self.y_final)
        
        self.att_pub.publish(self.attraction_vector)
        self.rep_pub.publish(self.repulsion_vector)
        self.fin_pub.publish(self.final_vector)
        #self.get_logger().info('Final: ' + str(math.sqrt(math.pow(self.x_final,2) + math.pow(self.y_final,2))))
        #self.get_logger().info('Attraction: ' + str(math.sqrt(math.pow(self.V_attraction[0],2) + math.pow(self.V_attraction[1],2))))
        #self.get_logger().info('Repulsion: ' + str(math.sqrt(math.pow(self.V_repulsion[0],2) + math.pow(self.V_repulsion[1],2))))

        if self.x_final < 0.0:
            v_lin = 0.0
        else:
            v_lin = math.sqrt(math.pow(self.x_final,2) + math.pow(self.y_final,2))

        v_ang = math.atan2(self.y_final, self.x_final)

        self.twist.linear.x = v_lin / 250
        self.twist.angular.z = v_ang / 4 * PI

        self.cmd_pub.publish(self.twist)

    def create_vector_pose(self, x, y):
        vector = PoseStamped()
        vector.header.frame_id = "base_link"
        now = self.get_clock().now() 
        vector.header.stamp = Time(sec=int(now.nanoseconds // 1e9), nanosec=int(now.nanoseconds % 1e9))
        vector.pose.position.x = 0.0
        vector.pose.position.y = 0.0
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

    def scan_callback(self, _msg):
        angle_min = _msg.angle_min
        angle_increment = _msg.angle_increment
        scan = _msg.ranges
        x_r = 0.0
        y_r = 0.0

        for i in range(len(scan)):
            if scan[i] < 100.0 and scan[i] > 0.1:
                x_r -= (1/scan[i])*math.cos(angle_min + angle_increment * i)
                y_r -= (1/scan[i])*math.sin(angle_min + angle_increment * i)

        self.V_repulsion = [x_r, y_r]

        self.repulsion_vector = self.create_vector_pose(self.V_repulsion[0], self.V_repulsion[1])

def main(args=None):
    rclpy.init(args=args)

    potential_field = PotentialField()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(potential_field)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        potential_field.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
