import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray,Float32MultiArray
from cosserat_nordbo.cosserat_rod_estimation.test_solve_ivp import solve_cosserat_ivp
import numpy as np
from scipy.spatial.transform import Rotation as R
from cosserat_nordbo.cosserat_rod_estimation.txt_reader import *
from rclpy.wait_for_message import wait_for_message


#!/usr/bin/env python3

def get_dgamma0s(gamma0, dx, dtheta, dn, dm):

    p = gamma0[0:3]
    Rot = gamma0[3:12].reshape((3, 3))
    n = gamma0[12:15]
    m = gamma0[15:18]

    last_v = gamma0[18:21]
    last_u = gamma0[21:24]


    dgamma0s = []

    dps = []
    dRs = []
    
    dms = []
    dns = []


    for vec in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
            
            dp = np.array(vec)*dx
            dps.append(dp)

            # Create rotation matrix from axis-angle (vec * dtheta)
            axis = np.array(vec)

            rotvec = axis * dtheta
            dR = R.from_rotvec(rotvec).as_matrix()
            dRs.append(dR)

            dm0 = np.array(vec)*dm
            dms.append(dm0)
            dn0 = np.array(vec)*dn
            dns.append(dn0)

    dv = np.array([0, 0, 0])
    du = np.array([0, 0, 0])

    for dp in dps:

        dgamma0 = np.zeros_like(gamma0)
        dgamma0[0:3] = p + dp
        dgamma0[3:12] = Rot.flatten()
        dgamma0[12:15] = n
        dgamma0[15:18] = m
        dgamma0[18:21] = last_v 
        dgamma0[21:24] = last_u 
        
        dgamma0s.append(dgamma0)

    for dR in dRs:
        dgamma0 = np.zeros_like(gamma0)
        dgamma0[0:3] = p
        dgamma0[3:12] = (Rot @ dR).flatten()
        dgamma0[12:15] = n
        dgamma0[15:18] = m
        dgamma0[18:21] = last_v 
        dgamma0[21:24] = last_u 
        
        dgamma0s.append(dgamma0)

    for dm0 in dms:
        dgamma0 = np.zeros_like(gamma0)
        dgamma0[0:3] = p
        dgamma0[3:12] = Rot.flatten()
        dgamma0[12:15] = n + dm0
        dgamma0[15:18] = m
        dgamma0[18:21] = last_v 
        dgamma0[21:24] = last_u 
        
        dgamma0s.append(dgamma0)

    for dn0 in dns:
        dgamma0 = np.zeros_like(gamma0)
        dgamma0[0:3] = p
        dgamma0[3:12] = Rot.flatten()
        dgamma0[12:15] = n + dn0
        dgamma0[15:18] = m
        dgamma0[18:21] = last_v 
        dgamma0[21:24] = last_u 
        
        dgamma0s.append(dgamma0)

    return np.array(dgamma0s)




class CosseratJacobianComputing(Node):
    def __init__(self):
        super().__init__('cosserat_jacobian_computing')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/points3d',
            self.listener_callback,
            10)
        


        _, msg = wait_for_message(Float64MultiArray, self, "cosserat_shooting_gamma0")

        gamma0 = np.array(msg.data)
        self.get_logger().info(f'Gamma0 length: {len(gamma0)}')
        if len(gamma0) != 18:
            self.get_logger().error('Received gamma0 does not have the correct length (24).')
            return
        self.get_logger().info(f'Gamma0: {gamma0}')

        self.gamma0 = gamma0



        self.get_logger().info('Node cosserat_jacobian_computing started, listening to /cosserat_shooting_gamma0')



        self.dx = 0.015
        self.dtheta = 0.1 # radians
        self.dn = 0.02
        self.dm = 0.01



    def listener_callback(self, msg):

        pvis = np.array(msg.data).reshape(-1,3).transpose()
        self.get_logger().info(f"pvis : {pvis}")

        plot = True



        gamma0_init = self.gamma0
        gamma0_init = np.concatenate([gamma0_init, np.zeros(6)])  # Ensure gamma0 has 24 elements
        
        dgamma0s = get_dgamma0s(gamma0_init, self.dx, self.dtheta, self.dn, self.dm)
        self.get_logger().info(f'Computed dgamma0s: {dgamma0s}')
        self.get_logger().info(f'shape of dgamma0s: {dgamma0s.shape}')


        cmap = plt.get_cmap('coolwarm')
        colors = [cmap(i / 11) for i in range(12)]

        J = None 

        p = gamma0_init[0:3]
        Rot = gamma0_init[3:12].reshape((3, 3))
        n = gamma0_init[12:15]
        m = gamma0_init[15:18]

        sol_0 = solve_cosserat_ivp(d=0.01, L=0.60, E=3e7, poisson=0.5, rho=1400,
                                    position=p,
                                    rotation=Rot,
                                    n0=n,
                                    m0=m,
                                    print_=False,
                                    )
        
        p0 = np.array(sol_0.y[:3]).reshape(-1,1)

        self.get_logger().info(f'p0: {p0}')
       


        for gamma0,c in zip(dgamma0s, colors):
            #self.get_logger().info(f'dgamma0: {gamma0}')

            p = gamma0[0:3]
            Rot = gamma0[3:12].reshape((3, 3))
            n = gamma0[12:15]
            m = gamma0[15:18]

            sol = solve_cosserat_ivp(d=0.01, L=0.60, E=3e7, poisson=0.5, rho=1400,
                                     position=p,
                                    rotation=Rot,
                                    n0=n,
                                    m0=m,
                                    print_=False,
                                    )
            
            p = np.array(sol.y[:3]).reshape(-1,1)

            dp = p-p0
            
            self.get_logger().info(f'p: {p}')

            #self.get_logger().info(f'Solution for dgamma0: {sol}')

            if J is None : 
                J=dp
            else :
                J=np.hstack([J,dp])


            if plot :

                fig,ax,ax2,ax3 = initialize_plot()
                self.get_logger().info(f'solution : {sol.y[:3]}')
                plot_cable(sol, color=c, ax=ax, ax2=ax2, ax3=ax3, T3=np.array([0.6, 0, 0]), n0=n, m0=m, E=3e7)

        

        Jp = np.linalg.pinv(J)
        self.get_logger().info(f'Jp shape : {Jp.shape}')



        pvis = pvis.reshape(-1,1)
        dp = pvis - p0 
        self.get_logger().info(f'dp : {dp}')

        dgamma = Jp @ dp 
        self.get_logger().info(f'dgamma : {dgamma}')

        gamma = gamma0_init + dgamma

        self.get_logger().info(f'gamma : {gamma}')

        p = gamma[0:3]
        Rot = gamma[3:12].reshape((3, 3))
        n = gamma[12:15]
        m = gamma[15:18]

        sol1 = solve_cosserat_ivp(d=0.01, L=0.60, E=3e7, poisson=0.5, rho=1400,
                                position=p,
                                rotation=Rot,
                                n0=n,
                                m0=m,
                                print_=False,
                                )




        if plot : 
            plot_cable(pvis,color="yellow" ,ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1])
            plot_cable(sol_0,"green",ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1])
            plot_cable(sol1,"purple",ax=ax, ax2=ax2, ax3=ax3,T3=pvis[:,-1])
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