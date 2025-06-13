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


#!/usr/bin/env python3



class CosseratJacobianComputing(Node):
    def __init__(self):
        super().__init__('cosserat_jacobian_computing')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_shape',
            self.target_cb,
            10)
        

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cosserat_shape',
            self.listener_callback,
            10)
        


        _, msg = wait_for_message(Float64MultiArray, self, "cosserat_shooting_gamma0")

        gamma0 = np.array(msg.data)
        self.get_logger().info(f'Gamma0 length: {len(gamma0)}')
        if len(gamma0) != 18:
            self.get_logger().error('Received gamma0 does not have the correct length (24).')
            return
        gamma0 = np.concatenate([gamma0, np.zeros(6)])
        self.get_logger().info(f'Gamma0: {gamma0}')

        self.gamma0 = gamma0



        self.get_logger().info('Node cosserat_jacobian_computing started, listening to /cosserat_shooting_gamma0')



        self.dx = 0.015
        self.dtheta = 0.1 # radians
        self.dn = 0.02
        self.dm = 0.01

        self.L = 0.5

        self.target = None


    def target_cb(self,msg):

        self.target = np.array(msg.data).reshape(-1,3).transpose().reshape(-1,1)




    def listener_callback(self, msg):

        k_pred = 0.6
        k_cmd = 0.2

        L=self.L
        plot = True

        if plot : 

            fig,ax,ax2,ax3 = initialize_plot()

        pvis = np.array(msg.data).reshape(-1,3).transpose()
        self.get_logger().info(f"pvis : {pvis}")




        gamma0_init = self.gamma0
        J = compute_jacobian(gamma0_init)
        p0,_  = solve_p_from_gamma(gamma0_init)

        Jp = np.linalg.pinv(J)
        self.get_logger().info(f'Jp shape : {Jp.shape}')


        pvis = pvis.reshape(-1,1)
        dp = (pvis - p0 ) * k_pred
        self.get_logger().info(f'dp shape : {dp.shape}')

        self.get_logger().info(f'dp : {dp}')

        dgamma_angle_axis = Jp @ dp 

        """self.get_logger().info(f'dgamma_angle_axis avant gains : {dgamma_angle_axis}')



        dgamma_angle_axis[0:3] = self.dx * dgamma_angle_axis[0:3]
        dgamma_angle_axis[3:6] = self.dtheta * dgamma_angle_axis[3:6]
        dgamma_angle_axis[6:9] = self.dn * dgamma_angle_axis[6:9]
        dgamma_angle_axis[9:12] = self.dm * dgamma_angle_axis[9:12]

        k = -0.03

        dgamma_angle_axis = k*dgamma_angle_axis"""

        
        
        dgamma = gamma_from_gamma_angle_axis(dgamma_angle_axis)


        self.get_logger().info(f'dgamma_angle_axis : {dgamma_angle_axis}')
        


        self.get_logger().info(f'dgamma : {dgamma}')
        self.get_logger().info(f'gamma0 init  : {gamma0_init}')

        
        gamma = add_gammas(gamma0_init , dgamma)

        self.get_logger().info(f'gamma : {gamma}')

        """p = gamma[0:3]
        Rot = gamma[3:12].reshape((3, 3))
        n = gamma[12:15]
        m = gamma[15:18]

        sol1 = solve_cosserat_ivp(d=0.01, L=L, E=3e7, poisson=0.5, rho=1400,
                                position=p,
                                rotation=Rot,
                                n0=n,
                                m0=m,
                                print_=False,
                                )"""
        
        p1 , R1 = solve_p_from_gamma(gamma)
        
        self.get_logger().info(f"R1 shape : {R1.shape}")
        
        self.gamma0 = gamma

        dp1 = p1 - pvis

        self.get_logger().info(f"norm dp after (red) :{np.linalg.norm(dp1)}")

        p_cmd = None

        if np.linalg.norm(dp1) < 0.02 and self.target is not None:
            self.get_logger().info(f"computing cmd")

            dp_target = (self.target - p1 ) * k_cmd

            p_target_i = p1 + dp_target



            dgamma_angle_axis = Jp @ dp_target
            dg = gamma_from_gamma_angle_axis(dgamma_angle_axis)
            dg = dg
            g_cmd = add_gammas(gamma,dg)
            p_cmd , Rcmd = solve_p_from_gamma(g_cmd)

            dp_cmd = p_cmd - p1
            dp_cmd = dp.reshape(3,-1)

            def compute_cmd(pc,p1,Rc,R1,plot = False):

                dp = (pc-p1).reshape(3,-1)
                dpl = dp[:,0]
                dpr = dp[:,-1]

                pl = p1.reshape(3,-1)[:,0]
                pr = p1.reshape(3,-1)[:,-1]


                Rcl = (Rc[:,0].reshape(3,3))
                Rcr = (Rc[:,-1].reshape(3,3))

                R1l = (R1[:,0].reshape(3,3))
                R1r = (R1[:,-1].reshape(3,3))

                dRl = R.from_matrix(Rcl @ R1l.transpose()).as_rotvec()
                dRr = R.from_matrix(Rcr @ R1r.transpose()).as_rotvec()

                if plot : 
                    plot_vector(dpr,pr,"red",ax)
                    plot_vector(dpl,pl,"red",ax)
                    plot_vector(dRr,pr,"yellow",ax,scale=0.2)
                    plot_vector(dRl,pl,"yellow",ax,scale=0.2)

                    self.get_logger().info(f"dRr :{dRr}")
                    self.get_logger().info(f"dRl :{dRl}")

                return dpl,dpr,dRl,dRr


            dpl,dpr,dRl,dRr = compute_cmd(p_cmd,p1,Rcmd,R1,plot=True)




        if plot : 


            plot_cable(pvis,color="green" ,ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1])
            plot_cable(p0,"yellow",ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1])
            plot_cable(p1,"red",ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1],points_tilles=True)

            if self.target is not None :
                plot_cable(self.target,"purple",ax,ax2,ax3)

                if p_cmd is not None :


                    plot_cable(p_cmd,"orange",ax,ax2,ax3)
                    plot_cable(p_target_i,"pink",ax,ax2,ax3)

        
            show_plot()


        




def main(args=None):
    rclpy.init(args=args)
    node = CosseratJacobianComputing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()