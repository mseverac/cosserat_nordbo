from cosserat_nordbo.cosserat_rod_estimation.both_ends_fixed import cosserat_get_cable_state
from cosserat_nordbo.cosserat_rod_estimation.pyelastica_jacobian import compute_pyelastica_jacobian

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Float32MultiArray,Float64MultiArray
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion,Twist,Point
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
import time
from threading import Condition
from tf2_ros import TransformException
from rclpy.wait_for_message import wait_for_message
from geometry_msgs.msg import TransformStamped

from visualization_msgs.msg import Marker


def time_msg_to_float(t):
    return t.sec + t.nanosec * 1e-9


def vector3_to_np(vector3_msg):
    return np.array([vector3_msg.x, vector3_msg.y, vector3_msg.z])

def quaternion_to_rotmat(quaternion_msg):
    q = [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]
    return R.from_quat(q).as_matrix()

class PyelControl(Node):
    def __init__(self):
        super().__init__('pyel_control')
        self.A_pub = self.create_publisher(Float64MultiArray, '/jacobian', 10)

        self.J = None



        self.create_subscription(
            TransformStamped,
            '/tcp_left',
            self.tcp_left_callback,
            10
        )

        self.create_subscription(
            TransformStamped,
            '/tcp_right',
            self.tcp_right_callback,
            10
        )

        self.create_timer(0.1, self.publish_A)  


        
    def tcp_right_callback(self, msg : TransformStamped):
        self.tcp_right = msg

    def tcp_left_callback(self, msg : TransformStamped):
        self.tcp_left = msg


    def publish_A(self):
        if hasattr(self, 'tcp_left') and hasattr(self, 'tcp_right'):

            transform_right = self.tcp_right
            transform_left = self.tcp_left

            before = self.get_clock().now().to_msg()

            if self.J is None :
                Jac,curr_curve = compute_pyelastica_jacobian(
                vector3_to_np(transform_right.transform.translation),
                vector3_to_np(transform_left.transform.translation),
                quaternion_to_rotmat(transform_right.transform.rotation),
                quaternion_to_rotmat(transform_left.transform.rotation),
                plot_cables=False,
                n_elem = 50,
                L=0.5,
                )

            else :
                Jac,curr_curve = compute_pyelastica_jacobian(
                vector3_to_np(transform_right.transform.translation),
                vector3_to_np(transform_left.transform.translation),
                quaternion_to_rotmat(transform_right.transform.rotation),
                quaternion_to_rotmat(transform_left.transform.rotation),
                plot_cables=False,
                plot_all=True,
                n_elem = 50,
                L=0.5,
                last_jac = self.J
                ) 


            after = self.get_clock().now().to_msg()
            self.get_logger().info(f"Computation time of the jacobian: {time_msg_to_float(after) - time_msg_to_float(before):.4f}s")


            self.get_logger().info(f"shape of the jacobian: {Jac.shape}")


            msg = Float64MultiArray()

            msg.data = Jac.flatten().astype(np.float32).tolist()

            self.A_pub.publish(msg)

            self.J = Jac


        else:
            self.get_logger().info("Waiting for tcp_left and tcp_right transforms to compute A matrix.")
            time.sleep(1)








def main(args=None):
    rclpy.init(args=args)
    node = PyelControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

