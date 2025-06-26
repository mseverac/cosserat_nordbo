import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray,Float32MultiArray
from cosserat_nordbo.cosserat_rod_estimation.test_solve_ivp import solve_cosserat_ivp
from cosserat_nordbo.cosserat_rod_estimation.cosserat_jacobian import (
    compute_jacobian,
    gamma_angle_axis_from_gamma,
    gamma_from_gamma_angle_axis,
    add_gammas,
    get_dgamma0s,
    solve_p_from_gamma,
    )

import numpy as np
from scipy.spatial.transform import Rotation as R
from cosserat_nordbo.cosserat_rod_estimation.txt_reader import *
from rclpy.wait_for_message import wait_for_message

from rclpy.executors import MultiThreadedExecutor
import threading

from geometry_msgs.msg import Point, Twist,TwistStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped


#!/usr/bin/env python3
def trans_to_matrix(trans: TransformStamped):
  
    pos = trans.transform.translation
    ori = trans.transform.rotation

    r_curr = R.from_quat([ori.x, ori.y, ori.z, ori.w])

    return r_curr.as_matrix(),np.array([pos.x, pos.y, pos.z])



def publish_marker_points_rviz( points3d, pub, color=(0.0, 0.0, 1.0), id=0):
    marker = Marker()
    marker.header.frame_id = "cam_bassa_base_frame"
    marker.ns = "points"
    marker.id = id
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    # Set marker properties
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.color.a = 1.0
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points3d]

    pub.publish(marker)



class CosseratJacobianComputing(Node):
    def __init__(self):
        super().__init__('cosserat_jacobian_computing')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/curve_target_6dof',
            self.target_cb,
            10)
        

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/points3d',
            self.listener_callback,
            10)


        self.left_cmd_pub = self.create_publisher(Twist,"/left/vis_vel_cmd_6dof",1)
        self.right_cmd_pub = self.create_publisher(Twist,"/right/vis_vel_cmd_6dof",1)


        self.publisher_points_rviz = self.create_publisher(Marker, "gamma0_curve_marker", 1)
        self.publisher_points_rviz_kp1 = self.create_publisher(Marker, "gamma0_kp1_curve_marker", 1)


        self.get_logger().info(f'Waiting for gamma0 from shooting')

        _, msg = wait_for_message(Float64MultiArray, self, "cosserat_shooting_gamma0")

        gamma0 = np.array(msg.data)
        self.get_logger().info(f'Gamma0 length: {len(gamma0)}')
        if len(gamma0) != 18:
            self.get_logger().error('Received gamma0 does not have the correct length (24).')
            return
        gamma0 = np.concatenate([gamma0, np.zeros(6)])
        self.get_logger().info(f'Gamma0: {gamma0}')

        self.gamma0 = gamma0
        self.last_good_gamma = gamma0
        self.bad_iter = 0
        self.max_bad_iter = 150
        rate = 10

        self.r = self.create_rate(rate)




        self.get_logger().info('Node cosserat_jacobian_computing started, listening to /cosserat_shooting_gamma0')



        self.dx = 0.015
        self.dtheta = 0.1 # radians
        self.dn = 0.02
        self.dm = 0.01

        self.L = 0.5

        self.target = None
        self.Jp = None
        self.pvis = None
        self.p_cmd = None


                # Variables pour stocker les données TCP
        self.tcp_left = None
        self.tcp_right = None
        
        # Subscribers pour les topics TCP
        self.tcp_left_sub = self.create_subscription(
            TransformStamped,
            '/tcp_left',
            self.tcp_left_callback,
            10
        )
        self.tcp_right_sub = self.create_subscription(
            TransformStamped,
            '/tcp_right',
            self.tcp_right_callback,
            10
        )
        
   
    def tcp_left_callback(self, msg):
        """Callback pour recevoir les données du topic /tcp_left"""
        self.tcp_left = msg
        self.get_logger().debug(f"Received TCP left: {msg.transform.translation.x:.3f}, "
                              f"{msg.transform.translation.y:.3f}, "
                              f"{msg.transform.translation.z:.3f}")

    def tcp_right_callback(self, msg):
        """Callback pour recevoir les données du topic /tcp_right"""
        self.tcp_right = msg
        self.get_logger().debug(f"Received TCP right: {msg.transform.translation.x:.3f}, "
                               f"{msg.transform.translation.y:.3f}, "
                               f"{msg.transform.translation.z:.3f}")
        


    def target_cb(self,msg):

        self.target = np.array(msg.data).reshape(-1,3).transpose().reshape(-1,1)




    def listener_callback(self, msg):

        self.pvis = np.array(msg.data).reshape(-1,3).transpose()


    def  compute_jac(self):
        while rclpy.ok():

            if self.gamma0 is None:
                self.get_logger().info("missing gamma0 for jac")
                self.r.sleep()

            else :
                J = compute_jacobian(self.gamma0)
                self.Jp = np.linalg.pinv(J)



    def compute_cmd(self):


        while rclpy.ok():


            missing_params = []
            if self.Jp is None:
                missing_params.append("Jp")
            if self.gamma0 is None:
                missing_params.append("gamma0")
            if self.pvis is None:
                missing_params.append("pvis")
            if self.target is None:
                missing_params.append("target")
            if self.tcp_right is None:
                missing_params.append("tcp_right")
            if self.tcp_left is None:
                missing_params.append("tcp_left")

            if missing_params:
                self.get_logger().info(f"missing : {', '.join(missing_params)} for cmd")
                
                self.r.sleep()

            else :

                k_pred = 0.1
                k_cmd = 0.4

                k = 0.5
                ka = 0.005

                pvis = self.pvis

                L=self.L
                Jp = self.Jp

                plot = False
                print_ = False 

                if plot : 

                    fig,ax,ax2,ax3 = initialize_plot()

                gamma0_init = self.gamma0
                p0,_  = solve_p_from_gamma(gamma0_init)


                pvis = pvis.reshape(-1,1)
                dp = (pvis - p0 ) * k_pred


                dgamma_angle_axis = Jp @ dp 
                dgamma = gamma_from_gamma_angle_axis(dgamma_angle_axis)

                gamma = add_gammas(gamma0_init , dgamma)
                p1 , R1 = solve_p_from_gamma(gamma)

                self.gamma0 = gamma

                dp1 = p1 - pvis
                

                publish_marker_points_rviz(p0.reshape(3,-1).transpose(),self.publisher_points_rviz,(1,1,0))

                if print_ :
                    self.get_logger().info(f'Jp shape : {Jp.shape}')
                    self.get_logger().info(f'dp shape : {dp.shape}')
                    self.get_logger().info(f'dp : {dp}')
                    self.get_logger().info(f'dgamma_angle_axis : {dgamma_angle_axis}')
                    self.get_logger().info(f'dgamma : {dgamma}')
                    self.get_logger().info(f'gamma0 init  : {gamma0_init}')
                    self.get_logger().info(f'gamma : {gamma}')
                    self.get_logger().info(f"R1 shape : {R1.shape}")
                    self.get_logger().info(f"norm dp after (red) :{np.linalg.norm(dp1)}")


                if np.linalg.norm(dp1) < 0.08 and self.target is not None:

                    self.max_bad_iter = 20 

                    self.last_good_gamma = gamma

                    if print_ :self.get_logger().info(f"computing cmd")

                    dp_target = (self.target - p1 ) * k_cmd

                    p_target_i = p1 + dp_target

                    dgamma_angle_axis = Jp @ dp_target
                    dg = gamma_from_gamma_angle_axis(dgamma_angle_axis)
                    dg = dg
                    g_cmd = add_gammas(gamma,dg)
                    self.p_cmd , self.Rcmd = solve_p_from_gamma(g_cmd)

                else :

                    self.bad_iter += 1

                    self.get_logger().info(f"too far  for the {self.bad_iter} th time ")

                    if self.bad_iter > self.max_bad_iter :

                        self.bad_iter = 0
                        
                        self.gamma0 = self.last_good_gamma
                        self.get_logger().info(f"too many bad iterations using last good gamma")


                if self.p_cmd is not None:
                        
                    def compute_cmd(pc,Rc,lt,rt,plot = False):

                        

                        pcl = pc[:,-1]
                        pcr = pc[:,0]

                        Rcl = (Rc[:,-1].reshape(3,3))
                        Rcr = (Rc[:,0].reshape(3,3))


                        R1l,pl = trans_to_matrix(lt)
                        R1r,pr = trans_to_matrix(rt)

                        dRl = R.from_matrix(Rcl @ R1l.transpose()).as_rotvec()
                        dRr = R.from_matrix(Rcr @ R1r.transpose()).as_rotvec()

                        dpl = pcl-pl
                        dpr = pcr-pr

                        if plot : 
                            plot_vector(dpr,pr,"red",ax)
                            plot_vector(dpl,pl,"red",ax)
                            plot_vector(dRr,pr,"yellow",ax,scale=0.2)
                            plot_vector(dRl,pl,"yellow",ax,scale=0.2)

                            if print_ :self.get_logger().info(f"dRr :{dRr}")
                            if print_ :self.get_logger().info(f"dRl :{dRl}")

                        return dpl,dpr,dRl,dRr
                    

                    publish_marker_points_rviz(self.p_cmd.reshape(3,-1).transpose(),self.publisher_points_rviz_kp1 ,(1,0.6,0))

                    dpl,dpr,dRl,dRr = compute_cmd(self.p_cmd.reshape(3,-1),self.Rcmd,self.tcp_left,self.tcp_right,plot=plot)


                    left_cmd = Twist()
                    left_cmd.linear.x = dpl[0]*k
                    left_cmd.linear.y = -dpl[2]*k
                    left_cmd.linear.z = dpl[1]*k
                    left_cmd.angular.x = dRl[0]*ka
                    left_cmd.angular.y = -dRl[2]*ka
                    left_cmd.angular.z = dRl[1]*ka

                    right_cmd = Twist()
                    right_cmd.linear.x = dpr[0]*k
                    right_cmd.linear.y = -dpr[2]*k
                    right_cmd.linear.z = dpr[1]*k
                    right_cmd.angular.x = dRr[0]*ka
                    right_cmd.angular.y = -dRr[2]*ka
                    right_cmd.angular.z = dRr[1]*ka

                    self.left_cmd_pub.publish(left_cmd)
                    self.right_cmd_pub.publish(right_cmd)

                    self.get_logger().info(f"norm left linear cmd : {np.linalg.norm([left_cmd.linear.x, left_cmd.linear.y, left_cmd.linear.z])}")
                    self.get_logger().info(f"norm right linear cmd : {np.linalg.norm([right_cmd.linear.x, right_cmd.linear.y, right_cmd.linear.z])}")
                    self.get_logger().info(f"norm left angular cmd : {np.linalg.norm([left_cmd.angular.x, left_cmd.angular.y, left_cmd.angular.z])}")
                    self.get_logger().info(f"norm right angular cmd : {np.linalg.norm([right_cmd.angular.x, right_cmd.angular.y, right_cmd.angular.z])}")
                    self.get_logger().info(f"***************")

                    def format_vector(vec):
                        return [format(v, ".6g") for v in vec]

                    self.get_logger().info(f"left linear cmd : {format_vector([left_cmd.linear.x, left_cmd.linear.y, left_cmd.linear.z])}")
                    self.get_logger().info(f"right linear cmd : {format_vector([right_cmd.linear.x, right_cmd.linear.y, right_cmd.linear.z])}")
                    self.get_logger().info(f"left angular cmd : {format_vector([left_cmd.angular.x, left_cmd.angular.y, left_cmd.angular.z])}")
                    self.get_logger().info(f"right angular cmd : {format_vector([right_cmd.angular.x, right_cmd.angular.y, right_cmd.angular.z])}")
                    self.get_logger().info(f"***************")

                    if print_ :

                        self.get_logger().info(f"left cmd : {left_cmd}")
                        self.get_logger().info(f"right cmd : {right_cmd}")

                    
                        

                    if plot : 
                        plot_cable(pvis,color="green" ,ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1])
                        plot_cable(p0,"yellow",ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1])
                        plot_cable(p1,"red",ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1],points_tilles=True)

                        if self.target is not None :
                            plot_cable(self.target,"purple",ax,ax2,ax3)

                            if self.p_cmd is not None :
                                plot_cable(self.p_cmd,"orange",ax,ax2,ax3)
                                plot_cable(p_target_i,"pink",ax,ax2,ax3)

                    
                        show_plot()


        




def main(args=None):
    rclpy.init(args=args)
    node = CosseratJacobianComputing()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)


    thread_Jac = threading.Thread(target=node.compute_jac, daemon=True)
    thread_Jac.start()

    thread_cmd = threading.Thread(target=node.compute_cmd, daemon=True)
    thread_cmd.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
        thread_Jac.join()
        thread_cmd.join()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()