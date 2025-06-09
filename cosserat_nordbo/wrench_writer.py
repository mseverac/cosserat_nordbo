import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class WrenchListener(Node):
    def __init__(self):
        super().__init__('wrench_listener')
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/nordbo/wrench_data',
            self.wrench_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.timer = self.create_timer(2, self.timer_callback)

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.wrench = None

    def wrench_callback(self, msg):
        self.wrench = msg.wrench
        #self.get_logger().info(f'Wrench: {self.wrench}')

        
    def timer_callback(self):
        if self.wrench is not None:
            try:
                transform_right = self.tf_buffer.lookup_transform(
                    'cam_bassa_base_frame',
                    'ur_right_cable',
                    rclpy.time.Time()
                )
                transform_left = self.tf_buffer.lookup_transform(
                    'cam_bassa_base_frame',
                    'ur_left_cable',
                    rclpy.time.Time()
                )
                with open('/home/lar95/ros/visual_servoing/src/cosserat_nordbo/cosserat_nordbo/wrench_poses.txt', 'a') as file:
                    file.write(f'{transform_left.transform}, {transform_right.transform}, {self.wrench}\n')
                self.get_logger().info(f'Wrench data written to file: {transform_left.transform.translation}, {transform_right.transform.translation}, {self.wrench}')
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(f'Failed to write to file: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WrenchListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()