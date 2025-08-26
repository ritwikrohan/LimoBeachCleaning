import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class ModelSpawner(Node):
    def __init__(self):
        super().__init__('model_spawner')
        self.client = self.create_client(SpawnEntity, 'spawn_entity')

    def send_request(self, model_name, package_name):
        model_path = f'package://{package_name}/models/{model_name}/{model_name}.sdf'

        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = open(model_path, 'r').read()
        request.robot_namespace = ''
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.5  # Adjust height if necessary
        request.reference_frame = 'world'
        
        self.client.wait_for_service()
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    model_spawner = ModelSpawner()
    
    model_name = 'your_model_name'  # Replace with your model's name
    package_name = 'your_package_name'  # Replace with your package's name

    model_spawner.send_request(model_name, package_name)
    while rclpy.ok():
        rclpy.spin_once(model_spawner)
        if model_spawner.future.done():
            try:
                response = model_spawner.future.result()
            except Exception as e:
                model_spawner.get_logger().error('Service call failed %r' % (e,))
            else:
                model_spawner.get_logger().info('Model %s spawned successfully' % model_name)
            break

    model_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()