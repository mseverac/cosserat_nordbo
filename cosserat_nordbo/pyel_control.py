from cosserat_nordbo.cosserat_rod_estimation.both_ends_fixed import cosserat_get_cable_state
from cosserat_nordbo.cosserat_rod_estimation.pyelastica_jacobian import compute_pyelastica_jacobian

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Float32MultiArray
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





def vector3_to_np(vector3_msg):
    return np.array([vector3_msg.x, vector3_msg.y, vector3_msg.z])

def quaternion_to_rotmat(quaternion_msg):
    q = [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]
    return R.from_quat(q).as_matrix()

class PyelControl(Node):
    def __init__(self):
        super().__init__('pyel_control')


        #gains for controller 
        self.k = 0.01
        self.ka = 0.2
        self.vmax = 0.05


        self.cmd_vel_left_pub = self.create_publisher(
            Twist, "/left/vis_vel_cmd_6dof", 1
        )
        self.cmd_vel_right_pub = self.create_publisher(
            Twist, "/right/vis_vel_cmd_6dof", 1
        )


        self.publisher_points_rviz_target = self.create_publisher(Marker, "pyel_target_marker", 1)
        self.publisher_points_rviz = self.create_publisher(Marker, "pyel_curve_marker", 1)

        self.points_pub = self.create_publisher(Float32MultiArray,"pyel_points",1)


        self.create_subscription(
            Float32MultiArray,
            'pyel_jacobian',
            self.jac_callback,
            10,
        )

        _, msg = wait_for_message(TransformStamped, self, "tcp_right")
        self.tr = msg

        _, msg = wait_for_message(TransformStamped, self, "tcp_left")
        self.tl = msg

        
        
        self.get_logger().info("waiting for target")

        
        _,target = wait_for_message(Float32MultiArray,self,"/curve_target_6dof")

        self.target_curve = np.array(target.data).reshape(-1,3)

        self.get_logger().info("node initialized")

        self.create_timer(0.5,self.callback)

        self.create_subscription(
            Float32MultiArray, 
            '/curve_target_6dof', 
            self.target_callback,
            10)
        
        self.Jp = None

    def publish_marker_points_rviz(self, points3d, pub, color=(0.0, 0.0, 1.0), id=0):
        marker = Marker()
        marker.header.frame_id = "cam_bassa_base_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "points"
        marker.id = id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Set marker properties
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points3d]

        pub.publish(marker)

    def pub_cmd_pyel(self,dr):
        """Publishes the command velocities to the robot for the pyel dr"""

        self.get_logger().info(f"------ dr -----")


        self.get_logger().info(f"dr 1-3:{dr[:3]}")
        self.get_logger().info(f"dr 4-6:{dr[3:6]}")
        self.get_logger().info(f"dr 7-9:{dr[6:9]}")
        self.get_logger().info(f"dr 10-12:{dr[9:]}")



        self.get_logger().info(f"------ Command velocities -----")

        self.get_logger().info(f"norm right linear dr : {np.linalg.norm([dr[0], dr[2], dr[1]])}")
        self.get_logger().info(f"norm left linear dr : {np.linalg.norm([dr[6], dr[8], dr[7]])}")

        self.get_logger().info(f"norm right angular dr : {np.linalg.norm([dr[3], dr[5], dr[4]])}")
        self.get_logger().info(f"norm left angular dr : {np.linalg.norm([dr[9], dr[11], dr[10]])}")

        self.get_logger().info(f"***************")

        dr = self.vmax * np.tanh(self.k * dr.astype(float).flatten())

        cmd_right = Twist()
        cmd_right.linear.x = dr[0]
        cmd_right.linear.y = -dr[2]
        cmd_right.linear.z = dr[1]

        cmd_right.angular.x = dr[3] * self.ka
        cmd_right.angular.y = -dr[5] * self.ka
        cmd_right.angular.z = dr[4] * self.ka

        cmd_left = Twist()
        cmd_left.linear.x = dr[6]
        cmd_left.linear.y = -dr[8]
        cmd_left.linear.z = dr[7]
        cmd_left.angular.x = dr[9] * self.ka
        cmd_left.angular.y = -dr[11] * self.ka
        cmd_left.angular.z = dr[10] * self.ka

        # self.get_logger().info(f"Right command: {cmd_right}")
        self.cmd_vel_right_pub.publish(cmd_right)
        # self.cmd_vel_right_pubs.publish(TwistStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="fixed_right_gripper"), twist=cmd_right))
        # self.get_logger().info(f"Left command: {cmd_left}")
        self.cmd_vel_left_pub.publish(cmd_left)
        # self.cmd_vel_left_pubs.publish(TwistStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="fixed_left_gripper"), twist=cmd_left))

        self.get_logger().info(f"norm left linear cmd : {np.linalg.norm([cmd_left.linear.x, cmd_left.linear.y, cmd_left.linear.z])}")
        self.get_logger().info(f"norm right linear cmd : {np.linalg.norm([cmd_right.linear.x, cmd_right.linear.y, cmd_right.linear.z])}")
        self.get_logger().info(f"norm left angular cmd : {np.linalg.norm([cmd_left.angular.x, cmd_left.angular.y, cmd_left.angular.z])}")
        self.get_logger().info(f"norm right angular cmd : {np.linalg.norm([cmd_right.angular.x, cmd_right.angular.y, cmd_right.angular.z])}")
        self.get_logger().info(f"***************")

        def format_vector(vec):
            return [format(v, ".6g") for v in vec]

        self.get_logger().info(f"left linear cmd : {format_vector([cmd_left.linear.x, cmd_left.linear.y, cmd_left.linear.z])}")
        self.get_logger().info(f"right linear cmd : {format_vector([cmd_right.linear.x, cmd_right.linear.y, cmd_right.linear.z])}")
        self.get_logger().info(f"left angular cmd : {format_vector([cmd_left.angular.x, cmd_left.angular.y, cmd_left.angular.z])}")
        self.get_logger().info(f"right angular cmd : {format_vector([cmd_right.angular.x, cmd_right.angular.y, cmd_right.angular.z])}")
        self.get_logger().info(f"***************")



    def target_callback(self,target):

        self.target_curve = np.array(target.data).reshape(-1,3)
        self.get_logger().info("target updated")


    def jac_callback(self,msg):

        self.Jp = np.array(msg.data).reshape(12,-1)
        self.get_logger().info("\n                                                ----->>>>  Jac updated  <<<<----\n")



    def callback(self):

        self.publish_marker_points_rviz(self.target_curve.reshape(-1, 3), self.publisher_points_rviz_target,color = (0.0, 0.5, 0.0))




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


        pp_list = cosserat_get_cable_state(
            vector3_to_np(transform_right.transform.translation),
            vector3_to_np(transform_left.transform.translation),
            quaternion_to_rotmat(transform_right.transform.rotation),
            quaternion_to_rotmat(transform_left.transform.rotation),
            n_elem = 50,
            final_time=0.05,
            rod_length=0.5,
        )


        s = np.array(pp_list["position"][-1]).flatten()


        self.publish_marker_points_rviz(s.reshape(3,-1).transpose(), self.publisher_points_rviz, color=(1.0, 0.0, 0.0))

        points = Float32MultiArray()
        points.data = s.reshape(3,-1).transpose().flatten().astype(np.float32).tolist()

        self.points_pub.publish(points)




        sstar = self.target_curve.transpose().flatten()

        ds = sstar - s 
        """self.get_logger().info(f"s :{s}")
        self.get_logger().info(f"s star :{sstar}")
        self.get_logger().info(f"ds :{ds}")"""


        if self.Jp is None :
            self.get_logger().info(f"waiting for jac")

        else :
            dr = self.Jp @ ds


            self.pub_cmd_pyel(dr)

















        
       





        




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

