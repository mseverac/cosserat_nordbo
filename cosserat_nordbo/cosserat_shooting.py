import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from scipy.spatial.transform import Rotation as R
import numpy as np
from cosserat_nordbo.cosserat_rod_estimation.init_gamma0 import get_cosserat_gamma0
from std_msgs.msg import Float32MultiArray,Float64MultiArray

import time
#!/usr/bin/env python3

def trans_to_matrix(trans: TransformStamped):
  
    pos = trans.transform.translation
    ori = trans.transform.rotation

    r_curr = R.from_quat([ori.x, ori.y, ori.z, ori.w])

    return r_curr.as_matrix(),np.array([pos.x, pos.y, pos.z])


class CosseratShootingNode(Node):
    def __init__(self):
        super().__init__('cosserat_shooting')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.base_frame = 'cam_bassa_base_frame'
        self.frames = ['ur_right_cable', 'ur_left_cable']

        self.shape_pub = self.create_publisher(
            Float32MultiArray, 'cosserat_shooting_shape', 10)
        
        self.gamma_pub = self.create_publisher(
            Float64MultiArray, 'cosserat_shooting_gamma0', 10)

        self.transes = [None, None]
        self.get_logger().info("CosseratShootingNode started.")

        self.rate = 10.0  # Hz

    def timer_callback(self):
        for i,frame in enumerate(self.frames):
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame,
                    rclpy.time.Time())
                self.get_logger().info(
                    f"Transform {frame} w.r.t {self.base_frame}: "
                    f"Position: ({trans.transform.translation.x:.3f}, "
                    f"{trans.transform.translation.y:.3f}, "
                    f"{trans.transform.translation.z:.3f})")
                
                self.transes[i]=trans


            except Exception as e:
                self.get_logger().warn(f"Could not get transform for {frame}: {e}")

       

        if self.transes[0] is not None and self.transes[1] is not None:

            R1,start = trans_to_matrix(self.transes[0])
            R2,end = trans_to_matrix(self.transes[1])

            self.get_logger().info(f"Right Rotation Matrix:\n{R1}")
            self.get_logger().info(f"Left Rotation Matrix:\n{R2}")
            self.get_logger().info(f"Right Start Position: {start}")
            self.get_logger().info(f"Left End Position: {end}")

            p1,r1,n0,m0, sol = get_cosserat_gamma0(
                start, end,
                R1=R1, R2=R2,
                n_elem=49, E=30e6, plot=False, print_=True,
                save=True,
                triple_opt=False,
                xtol=1e-6,
                maxiter=400,
                initial_guess=np.array([-0.38,-0.259, -0.219,
                                        -0.023,0.067,-0.036]),
            )

            shapex = sol.y[0]
            shapey = sol.y[1]
            shapez = sol.y[2]

            self.get_logger().info("--------------------------------")

            self.get_logger().info(f"Cosserat Shooting Result:\n"
                                   f"Start Position: {p1}\n"
                                   f"Start Rotation Matrix:\n{r1}\n"
                                   f"n0: {n0}\n"
                                   f"m0: {m0}\n"
                                   f"Shapex: {shapex}\n"
                                   f"Shapey: {shapey}\n"
                                   f"Shapez: {shapez}\n")
            
            shape = np.vstack((shapex, shapey, shapez)).T

            while True : 
            
                self.shape_pub.publish(
                    Float32MultiArray(data=shape.flatten().tolist())
                )


                gamma0 = np.concatenate([p1, r1.flatten(), n0, m0])
                self.gamma_pub.publish(
                    Float64MultiArray(data=gamma0.flatten().tolist())
                )
                
                time.sleep(1.0 / self.rate)


                



            
            

            




def main(args=None):
    rclpy.init(args=args)
    node = CosseratShootingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()