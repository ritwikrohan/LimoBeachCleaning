import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Polygon, Point32, TransformStamped
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from math import cos, sin, pi
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
# from std_msgs.msg import Empty
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs 
import math
import numpy as np

class RobotStateMachine(Node):


    def __init__(self):
        super().__init__('statemachine_node')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        client_cb_group2 = MutuallyExclusiveCallbackGroup()
        client_cb_group3 = MutuallyExclusiveCallbackGroup()
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(Empty, "/execute_motion", callback_group=client_cb_group)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.client2 = self.create_client(Empty, "/execute_motion_2", callback_group=client_cb_group3)
        while not self.client2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self._action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd', callback_group=client_cb_group2)
        self.req = Empty.Request()
        # self.publisher_ = self.create_publisher(Twist, "/diffbot_base_controller/cmd_vel_unstamped", 10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.local_table_footprint_publisher = self.create_publisher(Polygon,"/local_costmap/footprint", 10)
        self.global_table_footprint_publisher = self.create_publisher(Polygon,"/global_costmap/footprint", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )
        self.nav = BasicNavigator()
        self.speed_msg = Twist() 
        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.footprint = Polygon()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.robot_stage = {
            "initial_stage": [-1.0, -4.0, 0.0 , 1.0],
            "loading_stage_1": [0.0, -5.0, 0.0, 1.0],
            "loading_stage_2": [0.99, -6.88, -0.707,0.707],
            "door_stage": [5.5, -0.55, 0.0, 1.0],
            "last_stage_1": [10.0, -0.55, 0.0, 1.0],
            "last_stage_2": [10.0, -2.55, 0.0, 1.0],
            "back_to_initial": [-0.190,0.142 ,-0.1246747,0.9921977]}
        self.stage_number = 1
        self.goal_reached = False
        self.target_linear_x = 0.0
        self.target_linear_y = -5.0
        self.target_angular_z = 0.0  # Assuming we are using simple orientation in 2D
        self.target_angular_w = 1.0
        # Frame:map, Position(1.02507, -0.261939, 0), Orientation(0, 0, 0.708792, 0.705417) = Angle: 1.57557
    def odom_callback(self, msg):
        # Get current position from the message
        self.current_pose = msg.pose.pose
    def send_gripper_command(self, position=0.0, max_effort=0.0):
        # Create the goal message for the GripperCommand action
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        # Send the goal asynchronously
        future = self._action_client.send_goal_async(goal_msg)
        
        # Add a callback to handle the result once available
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback to handle the response when the goal is sent."""
        goal_handle = future.result()  # Get the goal handle from the future

        if not goal_handle:
            self.get_logger().error("Goal rejected!")
            return
        
        # Now, we need to get the result
        self.get_logger().info("Goal accepted!")

        # Wait for the result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback to handle the result once it is available."""
        try:
            result = future.result().result  # Result accessible from the future
            self.get_logger().info(f"Received result: {result}")
        except Exception as exc:
            self.get_logger().error(f"Exception while getting result: {exc}")
    def target_loop(self):
        if self.current_pose is None:
            return
    
        # Current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Calculate the error
        error_x = self.target_linear_x - current_x
        error_y = self.target_linear_y - current_y
        
        distance = np.sqrt(error_x**2 + error_y**2)

        linear_speed = 0.2  # Set a constant linear speed
        angular_speed = 0.0  # Start with zero angular speed

        if distance > 0.1:  # Threshold to reduce oscillation
            # Calculate the angular velocity based on the desired heading
            angle_to_target = np.arctan2(error_y, error_x)
            current_yaw = self.euler_from_quaternion(self.current_pose.orientation)[2]
            angle_error = angle_to_target - current_yaw
            
            # Normalize angle error to the range [-pi, pi]
            angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
            
            # Proportional controller for angular speed
            angular_speed = 2.0 * angle_error  # Adjust gain as necessary

        # Create and publish the command
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_speed
        cmd_msg.linear.y = 0.0
        cmd_msg.linear.z = 0.0
        cmd_msg.angular.z = angular_speed
        
        self.publisher_.publish(cmd_msg)
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # this function publishes a new footprint after the robot picks the table up according to table size.
    def publish_footprint_table(self):

        footprint = Polygon()
        point1 = Point32()
        point1.x = 0.35
        point1.y = 0.35

        point2 = Point32()
        point2.x = 0.35
        point2.y = -0.35

        point3 = Point32()
        point3.x = -0.35
        point3.y = -0.35

        point4 = Point32()
        point4.x = -0.35
        point4.y = 0.35

        footprint.points = [point1, point2, point3, point4]
            
        self.local_table_footprint_publisher.publish(footprint)
        self.global_table_footprint_publisher.publish(footprint)


    #this function publishes a circle shape as footprint when the robot drops the table and gets out of it
    def publish_footprint_robot(self):
        footprint = Polygon()
        points = []
        for angle in range(0, 360, 10):
            point = Point32()
            point.x = 0.25 * cos(angle * pi / 180)  
            point.y = 0.25 * sin(angle * pi / 180)  
            points.append(point)

            footprint.points = points
            
            self.local_table_footprint_publisher.publish(footprint)
            self.global_table_footprint_publisher.publish(footprint)

    # elevator down function
    def down_table(self):
        print('Down table')
        msg_pub = String()
        msg_pub.data = ""
        self.pub_table_down.publish(msg_pub)
        
        duration = Duration(seconds=5)
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            rate.sleep

    # getting out of table after dropping it
    # def turn_right(self,sec,nano=0):
    #     msg_vel = Twist()
    #     msg_vel.angular.z = -0.2
    #     duration = Duration(seconds=sec,nanoseconds=nano)
    #     rate = self.create_rate(10, self.get_clock())
    #     start_time = self.get_clock().now()
    #     while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
    #         msg = Twist()
    #         msg.linear.x = 0.0
    #         msg.linear.y = 0.0
    #         msg.linear.z = 0.0
    #         msg.angular.x = 0.0
    #         msg.angular.y = 0.0
    #         msg.angular.z = -0.2
    #         self.publisher_.publish(msg)
    #         # print('moving forward')
    #         rate.sleep
    #     # print('stop')
    #     msg.angular.z = 0.0
    #     self.publisher_.publish(msg)

    def move_forward(self,sec,nano=0):
        msg_vel = Twist()
        msg_vel.linear.x = 0.2
        duration = Duration(seconds=sec,nanoseconds=nano)
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg = Twist()
            msg.linear.x = 0.1
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            print('moving forward')
            rate.sleep
        # print('stop')
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
    
    def move_backward(self,sec,nano=0):
        msg_vel = Twist()
        msg_vel.linear.x = -0.2
        duration = Duration(seconds=sec,nanoseconds=nano)
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg = Twist()
            msg.linear.x = -0.2
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            # print('moving forward')
            rate.sleep
        # print('stop')
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    # def turn_left(self,sec,nano=0):
    #     msg_vel = Twist()
    #     msg_vel.angular.z = 0.2
    #     duration = Duration(seconds=sec,nanoseconds=nano)
    #     rate = self.create_rate(10, self.get_clock())
    #     start_time = self.get_clock().now()
    #     while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
    #         msg = Twist()
    #         msg.linear.x = 0.0
    #         msg.linear.y = 0.0
    #         msg.linear.z = 0.0
    #         msg.angular.x = 0.0
    #         msg.angular.y = 0.0
    #         msg.angular.z = 0.2
    #         self.publisher_.publish(msg)
    #         # print('moving forward')
    #         rate.sleep
    #     # print('stop')
    #     msg.angular.z = 0.0
    #     self.publisher_.publish(msg)

    def call_execute_motion_service(self):
        
        # Call the service
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            self.get_logger().info('Service call successful')
        else:
            self.get_logger().error('Service call failed')

    def call_execute_motion_service2(self):
        
        # Call the service
        self.future = self.client2.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            self.get_logger().info('Service call successful')
        else:
            self.get_logger().error('Service call failed')

    # attach table service request
    def send_request(self, a, table_number):
        self.req.attach_to_table = a
        self.req.table_number = table_number
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    def call_attach_link_service_1(self):
        print("call_attach_link_service called")
        
        self.req1 = AttachLink.Request()
        self.req1.model1_name = 'rb1_robot'
        self.req1.link1_name = 'robot_evelator_platform_link'
        # self.req1.link1_name = 'robot_base_footprint'
        self.req1.model2_name = 'rubish_table_1'
        self.req1.link2_name = 'rubish_table_1::rubish_table_1::link'

        self.future1 = self.attach_link_client.call_async(self.req1)
        rclpy.spin_until_future_complete(self, self.future1)
        if self.future1.result() is not None:
            self.get_logger().info('Service call completed.')
        else:
            self.get_logger().info('Service call failed.')
        return self.future1.result()

    def call_detach_link_service_1(self):
        self.req2 = DetachLink.Request()
        self.req2.model1_name = 'rb1_robot'
        self.req2.link1_name = 'robot_evelator_platform_link'
        # self.req2.link1_name = 'robot_base_footprint'
        self.req2.model2_name = 'rubish_table_1'
        self.req2.link2_name = 'rubish_table_1::rubish_table_1::link'
        self.future2 = self.detach_link_client.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future2)
        if self.future2.result() is not None:
            self.get_logger().info('Service call completed.')
        else:
            self.get_logger().info('Service call failed.')
        return self.future2.result()

    def call_attach_link_service_2(self):
        print("call_attach_link_service called")
        
        self.req1 = AttachLink.Request()
        self.req1.model1_name = 'rb1_robot'
        self.req1.link1_name = 'robot_evelator_platform_link'
        # self.req1.link1_name = 'robot_base_footprint'
        self.req1.model2_name = 'rubish_table_2'
        self.req1.link2_name = 'rubish_table_2::rubish_table_2::link'

        self.future1 = self.attach_link_client.call_async(self.req1)
        rclpy.spin_until_future_complete(self, self.future1)
        if self.future1.result() is not None:
            self.get_logger().info('Service call completed.')
        else:
            self.get_logger().info('Service call failed.')
        return self.future1.result()

    def call_detach_link_service_2(self):
        self.req2 = DetachLink.Request()
        self.req2.model1_name = 'rb1_robot'
        self.req2.link1_name = 'robot_evelator_platform_link'
        # self.req2.link1_name = 'robot_base_footprint'
        self.req2.model2_name = 'rubish_table_2'
        self.req2.link2_name = 'rubish_table_2::rubish_table_2::link'
        self.future2 = self.detach_link_client.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future2)
        if self.future2.result() is not None:
            self.get_logger().info('Service call completed.')
        else:
            self.get_logger().info('Service call failed.')
        return self.future2.result()

    #setting initial position of robot for localisation.
    def set_init_pose(self):
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = self.robot_stage["initial_stage"][0]
        self.initial_pose.pose.position.y = self.robot_stage["initial_stage"][1]
        self.initial_pose.pose.orientation.z = self.robot_stage["initial_stage"][2]
        self.initial_pose.pose.orientation.w = self.robot_stage["initial_stage"][3]
        self.nav.setInitialPose(self.initial_pose)
        print('Initial Position Set')
        # Wait for navigation to activate fully
        self.nav.waitUntilNav2Active()

    # Goes to the position of the given stage
    def go_to_pose(self, stages, stage_name):
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = stages[stage_name][0]
        self.goal_pose.pose.position.y = stages[stage_name][1]
        self.goal_pose.pose.orientation.z = stages[stage_name][2]
        self.goal_pose.pose.orientation.w = stages[stage_name][3]
        self.nav.goToPose(self.goal_pose)

        # Do something during your route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Print information for workers on the robot's ETA for the demonstration
        i = 0
        while not self.nav.isTaskComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 10 == 0:
                print('Time left to reach ' + stage_name + ' : ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
        
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached (' + stage_name + ')...')


        elif result == TaskResult.CANCELED:
            print('Task at ' + stage_name  +
                ' was canceled. Returning to staging point...')
            exit(-1)

        elif result == TaskResult.FAILED:
            print('Task at ' + stage_name + ' failed!')
            exit(-1)

        while not self.nav.isTaskComplete():
            pass
    

    def with_table_backup_1(self, turn=True):
        print("in with_table_backup")
        msg = Twist()
        duration = Duration(seconds=15) # setting the time decides the backup distance
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg.linear.x = -0.25
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            # print('moving backward with table')
            rate.sleep
        print('stop')
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        if turn:
            #turn the table to thte direction of back room
            duration = Duration(seconds=2,nanoseconds=670000000)
            start_time = self.get_clock().now()
            while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = -1.0
                self.publisher_.publish(msg)
                print('turn with table')
                rate.sleep
        print('stop')
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    

    def with_table_backup_2(self, turn=True):
        print("in with_table_backup")
        msg = Twist()
        duration = Duration(seconds=15) # setting the time decides the backup distance
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg.linear.x = -0.25
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            # print('moving backward with table')
            rate.sleep
        print('stop')
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        if turn:
            #turn the table to thte direction of back room
            duration = Duration(seconds=2,nanoseconds=670000000)
            start_time = self.get_clock().now()
            while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = -1.0
                self.publisher_.publish(msg)
                print('turn with table')
                rate.sleep
        print('stop')
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def table_forward(self):
        msg = Twist()
        duration = Duration(seconds=15) # setting the time decides the backup distance
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg.linear.x = 0.25
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            rate.sleep

    def table_backward(self):
        print("Backing up")
        msg = Twist()
        duration = Duration(seconds=7) # setting the time decides the backup distance
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            msg.linear.x = -0.25
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            rate.sleep
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def timer_callback(self):
        self.timer.cancel()
        self.control_loop()

    def door_transform(self, x, y):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "door_frame"
        static_transformStamped.transform.translation.x = x
        static_transformStamped.transform.translation.y = y
        static_transformStamped.transform.translation.z = 0.0
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0 
        static_transformStamped.transform.rotation.z = 0.0  
        static_transformStamped.transform.rotation.w = 1.0  

        self.static_tf_broadcaster.sendTransform(static_transformStamped)

    

    def get_current_yaw(self):
        """Get the current yaw angle of the robot from its orientation."""
        quaternion = self.current_pose.orientation  # Assume this is updated with /odom or another source
        # Convert quaternion to yaw
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Roll (x-axis rotation)
        sin_r_cosp = +2.0 * (w * x + y * z)
        cos_r_cosp = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sin_r_cosp, cos_r_cosp)

        # Pitch (y-axis rotation)
        sin_p = +2.0 * (w * y - z * x)
        sin_p = +1.0 if sin_p > +1.0 else sin_p
        sin_p = -1.0 if sin_p < -1.0 else sin_p
        pitch = np.arcsin(sin_p)

        # Yaw (z-axis rotation)
        sin_y_cosp = +2.0 * (w * z + x * y)
        cos_y_cosp = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(sin_y_cosp, cos_y_cosp)

        return yaw  # Return yaw in radians
    def send_velocity_command(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """
        Sends velocity commands to the robot.

        :param linear_x: Linear velocity in the x direction (forward/backward).
        :param linear_y: Linear velocity in the y direction (left/right).
        :param angular_z: Angular velocity around the z axis (rotation).
        """
        # Create a Twist message
        cmd_msg = Twist()
        
        # Set linear and angular velocities
        cmd_msg.linear.x = linear_x
        cmd_msg.linear.y = linear_y
        cmd_msg.linear.z = 0.0  # Not used in 2D navigation
        cmd_msg.angular.x = 0.0  # Not used
        cmd_msg.angular.y = 0.0  # Not used
        cmd_msg.angular.z = angular_z
        
        # Publish the command
        self.publisher_.publish(cmd_msg)

        # Optional: Log the command for debugging
        self.get_logger().info(f"Sent velocity command: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}")
    def turn_right(self, target_angle_degrees=90):
        target_angle = np.pi / 2  # Target angle in radians (90 degrees)

        # Calculate the target yaw to reach
        target_yaw = self.get_current_yaw() + target_angle
        rate = self.create_rate(10, self.get_clock())
        while True:
            current_yaw = self.get_current_yaw()
            error = target_yaw - current_yaw

            # Normalize the error to the range [-pi, pi]
            error = (error + np.pi) % (2 * np.pi) - np.pi

            # Check if we have reached the target angle
            if abs(error) < 0.05:  # 3-degree tolerance threshold (0.05 radians)
                break

            # Command to turn
            self.send_velocity_command(0.0, angular_z=-0.5)  # Rotate right
            # self.node.create_rate(10).sleep()  # Control loop frequency
            rate.sleep

        # Stop rotating
        self.send_velocity_command(0.0, angular_z=0.0)

    def turn_left(self, target_angle_degrees=90):
        target_angle = -np.pi / 2  # Target angle in radians (0 degrees)

        # Calculate the target yaw to reach
        target_yaw = self.get_current_yaw() + target_angle
        rate = self.create_rate(10, self.get_clock())
        while True:
            current_yaw = self.get_current_yaw()
            error = target_yaw - current_yaw

            # Normalize the error to the range [-pi, pi]
            error = (error + np.pi) % (2 * np.pi) - np.pi

            # Check if we have reached the target angle
            if abs(error) < 0.05:  # 3-degree tolerance threshold (0.05 radians)
                break

            # Command to turn
            self.send_velocity_command(0.0, angular_z=0.5)  # Rotate left
            rate.sleep

        # Stop rotating
        self.send_velocity_command(0.0, angular_z=0.0)

    def door_controller(self, gain_p=0.1, gain_yaw=0.1):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "robot_base_link", "door_frame", rclpy.time.Time()
            )
            self.get_logger().info(f'I HAVE ENTERED')

            distance_error = math.sqrt(
                transform_stamped.transform.translation.x ** 2 +
                transform_stamped.transform.translation.y ** 2
            )
            _, _, error_yaw = self.euler_from_quaternion(transform_stamped.transform.rotation)
            self.get_logger().info(f'Dist Error {distance_error}')
            self.get_logger().info(f'Yaw Error {error_yaw}')
            cmd_vel = Twist()
            while distance_error > 0.05 and error_yaw > 0.05:
                self.get_logger().info(f'Dist Error {distance_error}')
                self.get_logger().info(f'Yaw Error {error_yaw}')
                cmd_vel.linear.x = gain_p * distance_error
                cmd_vel.angular.z = gain_yaw * error_yaw
                self.publisher_.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f"Error in P controller: {str(e)}")
    def turn_to_yaw(self, target_angle_degrees):
        """Turn the robot to an exact target yaw in degrees."""
        target_yaw = np.radians(target_angle_degrees)  # Convert target angle to radians
        rate = self.create_rate(10, self.get_clock())
        while True:
            current_yaw = self.get_current_yaw()  # Current yaw in radians
            error = target_yaw - current_yaw

            # Normalize the error to the range [-pi, pi]
            error = (error + np.pi) % (2 * np.pi) - np.pi

            # Check if we have reached the target angle
            if abs(error) < 0.05:  # 3-degree tolerance threshold (0.05 radians)
                break

            # Command to turn
            if error > 0:
                self.send_velocity_command(0.0, angular_z=0.5)  # Rotate right if the error is positive
            else:
                self.send_velocity_command(0.0, angular_z=-0.5)  # Rotate left if the error is negative

            rate.sleep  # Control loop frequency

        # Stop rotating
        self.send_velocity_command(0.0, angular_z=0.0)
    def get_current_position(self):
        """Get the current position of the robot (x, y)."""
        # Here, we'll just return the current pose.
        # Replace this with your actual method for getting the robot's pose.
        return self.current_pose
    
    def move_to_x_position(self, target_x, k_p=0.5):
        """Move the robot towards the target x position using a P controller."""
        rate = self.create_rate(10, self.get_clock())
        while True:
            # current_x, current_y = self.get_current_position()  # Get current position
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            error_x = target_x - current_x  # Calculate error in x
            self.get_logger().info(f'Current X: {current_x}, Target X: {target_x}, Error: {error_x}')
            
            # Send velocity command only if error is significant
            if abs(error_x) < 0.01:  # Stop tolerance (5 cm)
                break
            
            # Set speed based on error
            linear_velocity_x = k_p * error_x  # P controller output
            self.send_velocity_command(linear_x=linear_velocity_x)  # Move along x
            
            rate.sleep  # Control loop frequency

            # Stop the robot
        self.send_velocity_command(0.0)

    def move_to_y_position(self, target_y, k_p=0.5):
        """Move the robot towards the target y position using a P controller."""
        rate = self.create_rate(10, self.get_clock())
        while True:
            # current_x, current_y = self.get_current_position()  # Get current position
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            error_y = target_y - current_y  # Calculate error in y
            
            self.get_logger().info(f'Current Y: {current_y}, Target Y: {target_y}, Error: {error_y}')
            
            # Send velocity command only if error is significant
            if abs(error_y) < 0.01:  # Stop tolerance (5 cm)
                break
            
            # Set speed based on error
            linear_velocity_y = k_p * error_y  # P controller output
            self.send_velocity_command(linear_x=-linear_velocity_y, linear_y=0.0)  # Move along y
            
            rate.sleep  # Control loop frequency

        # Stop the robot
        self.send_velocity_command(0.0)
    # Heart of the state machine which decides the flow of robot work
    def control_loop(self):
        # rate = self.node.create_rate(1)
        while rclpy.ok() and not self.goal_reached:
            # response = self.send_request(True)
            # if (response):
            #     self.goal_reached = True
            
            if self.stage_number == 1:
                self.go_to_pose(self.robot_stage, "loading_stage_1")
                self.turn_to_yaw(-90)
                self.move_to_y_position(-5.0)
                self.turn_to_yaw(0)
                self.move_to_x_position(0.0)
                self.call_execute_motion_service()
                time.sleep(60)
                self.stage_number = 2
            elif self.stage_number == 2:
                self.move_backward(10)
                self.go_to_pose(self.robot_stage, "loading_stage_2")
                self.move_forward(8)
                # self.send_gripper_command()
                self.call_execute_motion_service2()
                self.goal_reached = True
                self.stage_number = 3
            elif self.stage_number==3:
                print("Loading stage reached and proceeding to attach the table slowly...")
                response = self.send_request(True,1)
                # print(response)
                self.publish_footprint_table()
                # self.goal_reached = True
                self.stage_number=4
            elif self.stage_number==4:
                print("table attached, Backing up slowly and Proceeding to final stage ...")
                response2 = self.call_attach_link_service_1()
                self.with_table_backup_1(turn=False)
                time.sleep(2)
                # self.go_to_pose(self.robot_stage, "new_stage")
                self.stage_number=5
                # self.goal_reached = True
            elif self.stage_number==5:
                print("Proceeding to final stage...")
                time.sleep(5)
                self.go_to_pose(self.robot_stage, "door_stage")
                self.stage_number=6
            elif self.stage_number==6:
                self.door_controller()
                print("going forward")
                # self.table_forward()
                self.stage_number=7
            elif self.stage_number==7:
                print("Stage 7")
                self.go_to_pose(self.robot_stage, "last_stage_1")
                self.down_table()
                time.sleep(2)
                self.call_detach_link_service_1()
                # self.call_detach_link_service()
                time.sleep(2)
                self.table_backward()
                self.publish_footprint_robot()
                self.stage_number=8
                # self.goal_reached = True  
            #############################################################################################################    
            elif self.stage_number == 8:
                self.go_to_pose(self.robot_stage, "loading_stage_2")
                # self.goal_reached = True
                self.stage_number = 9
            elif self.stage_number==9:
                print("Loading stage reached and proceeding to attach the table slowly...")
                response = self.send_request(True,2)
                # print(response)
                self.publish_footprint_table()
                # self.goal_reached = True
                self.stage_number=10
            elif self.stage_number==10:
                print("table attached, Backing up slowly and Proceeding to final stage ...")
                response2 = self.call_attach_link_service_2()
                self.with_table_backup_2(turn=False)
                time.sleep(2)
                # self.go_to_pose(self.robot_stage, "new_stage")
                self.stage_number=11
                # self.goal_reached = True
            elif self.stage_number==11:
                print("Proceeding to final stage...")
                time.sleep(5)
                self.go_to_pose(self.robot_stage, "door_stage")
                self.stage_number=12
                # self.goal_reached = True
            elif self.stage_number==12:
                self.door_controller()
                print("going forward")
                # self.table_forward()
                self.stage_number=13
                # self.goal_reached = True
            elif self.stage_number==13:
                print("Stage 8")
                self.go_to_pose(self.robot_stage, "last_stage_2")
                self.down_table()
                time.sleep(2)
                self.call_detach_link_service_2()
                # self.call_detach_link_service()
                time.sleep(2)
                self.table_backward()
                self.publish_footprint_robot()
                self.stage_number=14
                self.goal_reached = True            
            #     print("table detached and waiting for a few seconds...")
            #     self.down_table()
            #     self.publish_footprint_robot()
            #     print("Proceeding back to initial stage ...")
            #     self.exit_under_the_table()
            #     self.go_to_pose(self.robot_stage,"back_to_initial")
            #     print("Waiting for next pickup. Robot on standby...")
            #     self.goal_reached=True

            
def main(args=None):
    rclpy.init(args=args)
    state_machine = RobotStateMachine()
    executor = MultiThreadedExecutor()
    executor.add_node(state_machine)
    executor.spin()
    # rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




#############################################################################
