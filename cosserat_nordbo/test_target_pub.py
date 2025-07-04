from cosserat_nordbo.cosserat_rod_estimation.both_ends_fixed import cosserat_get_cable_state

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

class TestTargetPub(Node):
    def __init__(self):
        super().__init__('test_target_pub')

        #self.create_subscription(Bool, "call_init_shape", self.callback, 10)

        self.create_timer(5,self.callback2)

        self.cosserat_shape_pub = self.create_publisher(
            Float32MultiArray,
            'curve_target_6dof',
            10
        )




        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ready = False  # Flag pour savoir si les TF ont été récupérées une première fois

        _, msg = wait_for_message(TransformStamped, self, "tcp_right")
        self.tr = msg

        _, msg = wait_for_message(TransformStamped, self, "tcp_left")
        self.tl = msg

        self.get_logger().info("node initialized")

        self.callback(Bool)

    

    def callback(self, msg):
        self.get_logger().info("inside callback")


        # Première fois : attendre que les TF soient disponibles
        if not self.ready:
            self.ready = True

        while self.ready == True :

            try:
                transform_right = self.tr
                transform_left = self.tl


                def time_msg_to_float(t):
                    return t.sec + t.nanosec * 1e-9
                
                now = self.get_clock().now().to_msg()


                age_right = time_msg_to_float(now) - time_msg_to_float(transform_right.header.stamp)
                age_left = time_msg_to_float(now) - time_msg_to_float(transform_left.header.stamp)

                MAX_TF_AGE = 0.5
                while age_right > MAX_TF_AGE or age_left > MAX_TF_AGE:
                    self.get_logger().warn(f"Skipping: TF too old (right: {age_right:.2f}s, left: {age_left:.2f}s)")
                    self.get_logger().warn(f"now : {time_msg_to_float(now):.2f}s)")
                    self.get_logger().warn(f"time left: {time_msg_to_float(transform_left.header.stamp):.2f}s)")


                    _, msg = wait_for_message(TransformStamped, self, "tcp_right")
                    transform_right = msg

                    _, msg = wait_for_message(TransformStamped, self, "tcp_left")
                    transform_left = msg

                    self.get_logger().warn(f"init tcp received")




                    now = self.get_clock().now().to_msg()


                    age_right = time_msg_to_float(now) - time_msg_to_float(transform_right.header.stamp)
                    age_left = time_msg_to_float(now) - time_msg_to_float(transform_left.header.stamp)



                else :
                    before = self.get_clock().now().to_msg()

                    pp_list = cosserat_get_cable_state(
                        np.array([-0.15,0.6,0.0]),
                        np.array([0.05,0.6,0.15]),
                        rod_length=0.5,
                        plot=False,
                        E=1e6
                    )

                    after = self.get_clock().now().to_msg()
                    self.get_logger().info(f"Computation time: {time_msg_to_float(after) - time_msg_to_float(before):.4f}s")

                    positions = np.array(pp_list["position"][-1]).T.reshape(-1, 3)

                    msg_out = Float32MultiArray()
                    msg_out.data = positions.flatten().tolist()
                    self.cosserat_shape_pub.publish(msg_out)
                    self.ready = False 

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(f'Failed to publish cosserat shape: {e}')




    def callback2(self):
        self.get_logger().info("inside callback")


        # Première fois : attendre que les TF soient disponibles
        if not self.ready:
            self.ready = True

        while self.ready == True :

            try:
                transform_right = self.tr
                transform_left = self.tl


                def time_msg_to_float(t):
                    return t.sec + t.nanosec * 1e-9
                
                now = self.get_clock().now().to_msg()


                age_right = time_msg_to_float(now) - time_msg_to_float(transform_right.header.stamp)
                age_left = time_msg_to_float(now) - time_msg_to_float(transform_left.header.stamp)

                MAX_TF_AGE = 0.5
                while age_right > MAX_TF_AGE or age_left > MAX_TF_AGE:
                    self.get_logger().warn(f"Skipping: TF too old (right: {age_right:.2f}s, left: {age_left:.2f}s)")
                    self.get_logger().warn(f"now : {time_msg_to_float(now):.2f}s)")
                    self.get_logger().warn(f"time left: {time_msg_to_float(transform_left.header.stamp):.2f}s)")


                    _, msg = wait_for_message(TransformStamped, self, "tcp_right")
                    transform_right = msg

                    _, msg = wait_for_message(TransformStamped, self, "tcp_left")
                    transform_left = msg

                    self.get_logger().warn(f"init tcp received")




                    now = self.get_clock().now().to_msg()


                    age_right = time_msg_to_float(now) - time_msg_to_float(transform_right.header.stamp)
                    age_left = time_msg_to_float(now) - time_msg_to_float(transform_left.header.stamp)



                else :
                    before = self.get_clock().now().to_msg()

                    pp_list = cosserat_get_cable_state(
                        np.array([-0.15,0.6,0.0]),
                        np.array([0.10,0.6,0.25]),
                        rod_length=0.5,
                        plot=False,
                        E=1e6
                    )

                    after = self.get_clock().now().to_msg()
                    self.get_logger().info(f"Computation time: {time_msg_to_float(after) - time_msg_to_float(before):.4f}s")

                    positions = np.array(pp_list["position"][-1]).T.reshape(-1, 3)

                    msg_out = Float32MultiArray()
                    msg_out.data = positions.flatten().tolist()
                    self.cosserat_shape_pub.publish(msg_out)
                    self.ready = False 

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(f'Failed to publish cosserat shape: {e}')




def main(args=None):
    rclpy.init(args=args)
    node = TestTargetPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()