from cosserat_nordbo.cosserat_rod_estimation.test_solve_ivp import solve_cosserat_ivp
import numpy as np
from scipy.spatial.transform import Rotation as R
from cosserat_nordbo.cosserat_rod_estimation.txt_reader import *



def get_dgamma0s(gamma0, dx, dtheta, dn, dm):

    p = gamma0[0:3]
    Rot = gamma0[3:12].reshape((3, 3))
    n = gamma0[12:15]
    m = gamma0[15:18]

    last_v = gamma0[18:21]
    last_u = gamma0[21:24]

    drs = []


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
        drs.append(dx)

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
        drs.append(dtheta)


        dgamma0[12:15] = n
        dgamma0[15:18] = m
        dgamma0[18:21] = last_v 
        dgamma0[21:24] = last_u 
        
        dgamma0s.append(dgamma0)

    for dn0 in dns:
        dgamma0 = np.zeros_like(gamma0)
        dgamma0[0:3] = p
        dgamma0[3:12] = Rot.flatten()
        dgamma0[12:15] = n + dn0
        drs.append(dn)

        dgamma0[15:18] = m
        dgamma0[18:21] = last_v 
        dgamma0[21:24] = last_u 
        
        dgamma0s.append(dgamma0)

    for dm0 in dms:
        dgamma0 = np.zeros_like(gamma0)
        dgamma0[0:3] = p
        dgamma0[3:12] = Rot.flatten()
        dgamma0[12:15] = n 
        dgamma0[15:18] = m + dm0
        drs.append(dm)

        dgamma0[18:21] = last_v 
        dgamma0[21:24] = last_u 
        
        dgamma0s.append(dgamma0)

    return np.array(dgamma0s),np.array(drs)


def gamma_from_gamma_angle_axis(dgamma_angle_axis):
    
    angle_axis = (dgamma_angle_axis[3:6]).flatten()

    rMat = R.from_rotvec(angle_axis).as_matrix()

    dgamma = np.zeros(18)

    dgamma[0:3] = dgamma_angle_axis[0:3].flatten()
    dgamma[3:12] = rMat.flatten()
    dgamma[12:18] = dgamma_angle_axis[6:12].flatten()

    return dgamma

def gamma_angle_axis_from_gamma(gamma):
     
    rMat = R.from_matrix(gamma[3:12].reshape(3,3))

    g = np.zeros(12)

    g[0:3] = gamma[0:3].flatten()
    g[3:6] = rMat.as_rotvec().flatten()
    g[6:12] = gamma[12:18].flatten()

    return g.reshape(12,1)

     

     

def add_gammas(g1,g2):

    g1 = g1[:18]
    g2 = g2[:18]
    g = g1+g2
    R1 = g1[3:12].reshape(3,3)
    R2 = g2[3:12].reshape(3,3)

    R = R1 @ R2
    g[3:12] = R.flatten()

    return g

def solve_p_from_gamma(g0,L=0.5):
    p = g0[0:3]
    Rot = g0[3:12].reshape((3, 3))
    n = g0[12:15]
    m = g0[15:18]

    sol_0 = solve_cosserat_ivp(d=0.01, L=L, E=3e7, poisson=0.5, rho=1400,
                                position=p,
                                rotation=Rot,
                                n0=n,
                                m0=m,
                                print_=False,
                                )
    
    p0 = np.array(sol_0.y[:3]).reshape(-1,1)

    R0 = np.array(sol_0.y[3:12])

    return p0,R0


def compute_jacobian(g0,dx = 0.015,
        dtheta = 0.1 ,
        dn = 0.02,
        dm = 0.01,
        L=0.5,
        plot=False,
        ):
    

    
     
    
    dgs,drs = get_dgamma0s(g0, dx, dtheta, dn, dm)

    print(f"drs : {drs}")
    


    cmap = plt.get_cmap('coolwarm')
    colors = [cmap(i / 11) for i in range(12)]

    J = None 

    p0,_ = solve_p_from_gamma(g0)

    

    


    for gamma0,dr,c in zip(dgs,drs, colors):
        #self.get_logger().info(f'dgamma0: {gamma0}')

        p ,_ = solve_p_from_gamma(gamma0,L=L)

        dp = p-p0
        dp = dp/dr
        

        #self.get_logger().info(f'Solution for dgamma0: {sol}')

        if J is None : 
            J=dp
        else :
            J=np.hstack([J,dp])

        

        if plot :

            plot_cable(p, color=c, ax=ax, ax2=ax2, ax3=ax3)

    

    return J




testing = False

if testing :

    fig,ax,ax2,ax3 = initialize_plot()

     

    g0 = np.array([-0.1,0.7,0.3, 
                    0.0,0.0,1.0, 0.0,1.0,0.0 ,-1.0,0.0,0.0, 
                    -0.44942234, -0.14862693, -0.30385165,
                    -0.0111957 ,  0.06466507,-0.02059487])
    
    J = compute_jacobian(g0,plot=False)


    dgaa = np.zeros(12)
    dgaa[10] = 0.01

    dg = gamma_from_gamma_angle_axis(dgaa)


    
    p0 ,_ = solve_p_from_gamma(g0)
    plot_cable(p0,"blue",ax,ax2,ax3)
    print(f"g0 :{g0}")
    print(f"p0,0 :{p0[0]}")
    print(f"p0,1 :{p0[1]}")
    print(f"p0,1 :{p0[2]}")

    
    g1 = add_gammas(g0,dg)

    
    pc = p0 + J @ gamma_angle_axis_from_gamma(dg)
    plot_cable(pc,"yellow",ax,ax2,ax3,points_tilles=False)

    print(f"g0 :{g0}")
    print(f"g1 :{g1}")


    

    print(f"pc,0 :{pc[0]}")
    
    pr,_  = solve_p_from_gamma(g1)
    plot_cable(pr,"green",ax,ax2,ax3,points_tilles=True)


    show_plot()



     

