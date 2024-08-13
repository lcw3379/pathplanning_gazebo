import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from ament_index_python.packages import get_package_share_directory
import os


class SpawnAndDeleteCircleNode(Node):
    def __init__(self):
        super().__init__('spawn_and_delete_circle_node')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        while not self.delete_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting again...')

        self.spawn_request()
        self.timer = self.create_timer(2.0, self.timer_callback)

    def spawn_request(self):
        self.get_logger().info('Spawning circle...')
        request = SpawnEntity.Request()

        # SDF 파일의 경로 설정  
        pkg_share = get_package_share_directory('nav_project')
        sdf_file_path = os.path.join(pkg_share, 'models/circle.sdf')

        with open(sdf_file_path, 'r') as f:
            sdf_content = f.read()

        request.name = 'circle'
        request.xml = sdf_content
        request.robot_namespace = ''
        request.initial_pose.position.x = 1.0
        request.initial_pose.position.y = 1.0
        request.initial_pose.position.z = 0.0
        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = 0.0
        request.initial_pose.orientation.w = 1.0

        self.spawn_future = self.spawn_cli.call_async(request)

    def delete_request(self):
        self.get_logger().info('Deleting circle...')
        request = DeleteEntity.Request()
        request.name = 'circle'
        self.delete_future = self.delete_cli.call_async(request)

    def timer_callback(self):
        if hasattr(self, 'spawn_future') and self.spawn_future.done():
            try:
                response = self.spawn_future.result()
            except Exception as e:
                self.get_logger().info('Spawn service call failed %r' % (e,))
            else:
                self.get_logger().info('Circle spawned successfully')

            self.delete_request()
            self.timer.cancel()  # 타이머를 멈춥니다

        if hasattr(self, 'delete_future') and self.delete_future.done():
            try:
                response = self.delete_future.result()
            except Exception as e:
                self.get_logger().info('Delete service call failed %r' % (e,))
            else:
                self.get_logger().info('Circle deleted successfully')
            
            self.spawn_request()

def main(args=None):
    rclpy.init(args=args)
    node = SpawnAndDeleteCircleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
