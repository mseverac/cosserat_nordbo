from cosserat_nordbo.cosserat_rod_estimation.both_ends_fixed import cosserat_get_cable_state
from cosserat_nordbo.cosserat_rod_estimation.pyelastica_jacobian import compute_pyelastica_jacobian

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Float32MultiArray
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
import time
from threading import Condition
from tf2_ros import TransformException
from rclpy.wait_for_message import wait_for_message
from geometry_msgs.msg import TransformStamped






def vector3_to_np(vector3_msg):
    return np.array([vector3_msg.x, vector3_msg.y, vector3_msg.z])

def quaternion_to_rotmat(quaternion_msg):
    q = [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]
    return R.from_quat(q).as_matrix()

class PyelJac(Node):
    def __init__(self):
        super().__init__('pyel_jac')


        self.jac_pub = self.create_publisher(
            Float32MultiArray,
            'pyel_jacobian',
            10
        )

        _, msg = wait_for_message(TransformStamped, self, "tcp_right")
        self.tr = msg

        _, msg = wait_for_message(TransformStamped, self, "tcp_left")
        self.tl = msg

        

        self.get_logger().info("node initialized")

        self.create_timer(4,self.callback)

        


    def callback(self):
        self.get_logger().info("inside callback")

        transform_right = self.tr
        transform_left = self.tl


        def time_msg_to_float(t):
            return t.sec + t.nanosec * 1e-9
        
        now = self.get_clock().now().to_msg()


        age_right = time_msg_to_float(now) - time_msg_to_float(transform_right.header.stamp)
        age_left = time_msg_to_float(now) - time_msg_to_float(transform_left.header.stamp)

        MAX_TF_AGE = 0.5
        while age_right > MAX_TF_AGE or age_left > MAX_TF_AGE:


            if age_left > 10 or age_right > 10 :
                self.get_logger().warn(f"Skipping: TF too old (right: {age_right:.2f}s, left: {age_left:.2f}s)")
            #self.get_logger().warn(f"now : {time_msg_to_float(now):.2f}s)")
            #self.get_logger().warn(f"time left: {time_msg_to_float(transform_left.header.stamp):.2f}s)")


            _, msg = wait_for_message(TransformStamped, self, "tcp_right")
            transform_right = msg

            _, msg = wait_for_message(TransformStamped, self, "tcp_left")
            transform_left = msg

            #self.get_logger().warn(f"init tcp received")

            now = self.get_clock().now().to_msg()

            age_right = time_msg_to_float(now) - time_msg_to_float(transform_right.header.stamp)
            age_left = time_msg_to_float(now) - time_msg_to_float(transform_left.header.stamp)

        self.tr = transform_right
        self.tl = transform_left

        before = self.get_clock().now().to_msg()

        Jac,curr_curve = compute_pyelastica_jacobian(
            vector3_to_np(transform_right.transform.translation),
            vector3_to_np(transform_left.transform.translation),
            quaternion_to_rotmat(transform_right.transform.rotation),
            quaternion_to_rotmat(transform_left.transform.rotation),
            plot_cables=False,
            n_elem = 50,
            L=0.5,
        )

        after = self.get_clock().now().to_msg()
        self.get_logger().info(f"Computation time of the jacobian: {time_msg_to_float(after) - time_msg_to_float(before):.4f}s")


        Jp = np.linalg.pinv(Jac)
        self.get_logger().info(f"shape of the jacobian: {Jp.shape}")


        msg = Float32MultiArray()

        msg.data = Jp.flatten().astype(np.float32).tolist()

        self.jac_pub.publish(msg)





        









def main(args=None):
    rclpy.init(args=args)
    node = PyelJac()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

