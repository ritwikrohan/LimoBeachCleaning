from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class ClientAsync(Node):

    def __init__(self):
        
        super().__init__('service_client')
        
        self.reentrant_group_1 = ReentrantCallbackGroup()
        self.client = self.create_client(SetEntityState , '/demo/set_entity_state')
        self.timer = self.create_timer(0.5, self.timer_callback, callback_group=self.reentrant_group_1)
        self.subscriber_up = self.create_subscription(
            String,
            '/elevator_up',
            self.up_callback,
            10,
            callback_group=self.reentrant_group_1)
        self.subscriber_down = self.create_subscription(
            String,
            '/elevator_down',
            self.down_callback,
            10,
            callback_group=self.reentrant_group_1)
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        
        self.req = SetEntityState.Request()
        self.req.state.name = "rb2_simple_cart"
        self.req.state.reference_frame = "rb1_robot"
        self.req.state.pose.position.z  = 0.1
        self.attach = 0

    def up_callback(self, msg):
        self.attach = 1
    
    def down_callback(self, msg):
        self.attach = 2

    def timer_callback(self):
        if self.attach is 1:
            self.future = self.client.call_async(self.req)
    
        elif self.attach is 2:
            self.req = SetEntityState.Request()
            self.req.state.name = "rb2_simple_cart"
            self.req.state.reference_frame = "rb2_simple_cart"
            self.req.state.pose.position.z  = 0.0
            self.attach = 0
            time.sleep(2)
            self.future = self.client.call_async(self.req)

        else:
            pass


def main(args=None):
    
    rclpy.init(args=args)
    client = ClientAsync()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(client)
    
    try:
        executor.spin()
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()