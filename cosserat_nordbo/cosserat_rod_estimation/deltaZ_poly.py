import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

x = sp.symbols('x')

X = sp.symbols('X')

P = x*X**4 - 2*(x+1)*X**3 + (x+3)*X**2


# Création des valeurs de x

# Création des points pour X entre 0 et 1
X_points = np.linspace(0, 1, 500)

x_values = -np.logspace(3, 0, 300)  

longueur = np.array([125.01336067, 122.15888335, 119.36960865, 116.64404768, 113.98074562,
 111.37828084, 108.83526418, 106.35033823, 103.9221766 , 101.54948313,
  99.23099135,  96.96546366,  94.75169077,  92.58849099,  90.47470962,
  88.40921835,  86.39091463,  84.41872113,  82.49158508,  80.60847779,
  78.76839408,  76.9703517 ,  75.21339087,  73.49657372,  71.81898381,
  70.17972561,  68.57792411,  67.01272425,  65.48329049,  63.98880643,
  62.52847427,  61.10151447,  59.70716529,  58.3446824 ,  57.01333847,
  55.7124228 ,  54.44124092,  53.19911424,  51.98537965,  50.79938923,
  49.64050983,  48.50812281,  47.40162363,  46.32042158,  45.26393946,
  44.23161325,  43.22289183,  42.23723667,  41.27412155,  40.33303228,
  39.41346643,  38.51493305,  37.63695241,  36.77905576,  35.94078504,
  35.12169269,  34.32134139,  33.53930379,  32.77516233,  32.02850901,
  31.29894513,  30.58608113,  29.88953637,  29.20893889,  28.54392526,
  27.89414035,  27.25923717,  26.63887665,  26.0327275 ,  25.440466  ,
  24.86177584,  24.29634794,  23.74388031,  23.20407787,  22.67665229,
  22.16132184,  21.65781125,  21.16585155,  20.68517994,  20.21553961,
  19.75667966,  19.30835494,  18.8703259 ,  18.4423585 ,  18.02422404,
  17.61569911,  17.21656532,  16.82660943,  16.44562298,  16.07340234,
  15.70974854,  15.35446718,  15.00736832,  14.66826638,  14.33698004,
  14.01333215,  13.69714961,  13.38826333,  13.08650808,  12.79172243,
  12.50374868,  12.22243273,  11.94762406,  11.67917557,  11.41694359,
  11.16078772,  10.91057083,  10.66615892,  10.42742108,  10.19422945,
   9.96645907,   9.74398791,   9.52669673,   9.31446909,   9.10719108,
   8.90475166,   8.70704219,   8.5139566 ,   8.32539126,   8.14124496,
   7.9614188 ,   7.7858162 ,   7.6143428 ,   7.44690646,   7.28341714,
   7.12378691,   6.96792989,   6.81576218,   6.66720184,   6.52216884,
   6.38058501,   6.242374  ,   6.10746125,   5.97577393,   5.84724091,
   5.72179273,   5.59936154,   5.4798811 ,   5.36328669,   5.24951514,
   5.13850473,   5.0301952 ,   4.92452772,   4.82144481,   4.72089035,
   4.62280957,   4.52714893,   4.43385621,   4.34288037,   4.25417159,
   4.16768124,   4.0833618 ,   4.0011669 ,   3.92105123,   3.84297057,
   3.76688174,   3.69274255,   3.62051182,   3.55014934,   3.48161582,
   3.41487289,   3.34988311,   3.28660985,   3.22501738,   3.16507077,
   3.1067359 ,   3.04997942,   2.99476874,   2.94107203,   2.88885815,
   2.83809664,   2.78875774,   2.74081233,   2.6942319 ,   2.64898857,
   2.60505501,   2.56240449,   2.52101078,   2.4808482 ,   2.44189153,
   2.40411606,   2.36749749,   2.33201197,   2.29763605,   2.26434665,
   2.23212106,   2.20093691,   2.17077211,   2.1416049 ,   2.11341376,
   2.08617742,   2.05987484,   2.03448519,   2.00998779,   1.98636216,
   1.96358796,   1.94164496,   1.92051306,   1.90017225,   1.88060264,
   1.86178439,   1.84369774,   1.82632302,   1.80964062,   1.79363099,
   1.77827468,   1.7635523 ,   1.74944458,   1.73593234,   1.72299651,
   1.7106182 ,   1.69877865,   1.68745928,   1.67664173,   1.66630786,
   1.6564398 ,   1.64701993,   1.63803096,   1.6294559 ,   1.62127815,
   1.61348143,   1.6060499 ,   1.59896808,   1.59222096,   1.58579392,
   1.57967284,   1.573844  ,   1.56829418,   1.5630106 ,   1.55798098,
   1.55319346,   1.54863667,   1.54429969,   1.54017205,   1.53624373,
   1.53250512,   1.52894705,   1.52556076,   1.52233788,   1.51927044,
   1.51635083,   1.5135718 ,   1.51092646,   1.50840824,   1.50601089,
   1.50372847,   1.50155533,   1.49948609,   1.49751566,   1.49563918,
   1.49385204,   1.49214988,   1.49052852,   1.48898402,   1.48751264,
   1.48611081,   1.48477514,   1.48350243,   1.48228962,   1.48113381,
   1.48003225,   1.47898232,   1.47798155,   1.47702756,   1.47611812,
   1.47525109,   1.47442444,   1.47363626,   1.47288471,   1.47216804,
   1.47148461,   1.47083284,   1.47021124,   1.46961837,   1.46905288,
   1.4685135 ,   1.46799898,   1.46750817,   1.46703995,   1.46659326,
   1.46616711,   1.46576053,   1.4653726 ,   1.46500247,   1.46464931,
   1.46431232,   1.46399076,   1.46368391,   1.46339109,   1.46311165,
   1.46284497,   1.46259048,   1.46234759,   1.46211579,   1.46189455,
   1.46168339,   1.46148186,   1.4612895 ,   1.4611059 ,   1.46093065])


if longueur is None :




    P_prime = sp.diff(P, X)

    # Expression de 1 + (P')^2
    expr = 1 + P_prime**2

    # Affichage
    sp.pprint(expr, use_unicode=True)


    expr_developpee = sp.simplify(sp.expand(expr))
    sp.pprint(expr_developpee, use_unicode=True)

    expr_developpee = sp.simplify(sp.expand(expr_developpee))
    # Affichage
    sp.pprint(expr_developpee, use_unicode=True)


    sqrt_expr = sp.simplify(sp.sqrt(expr_developpee))

    # Affichage lisible
    sp.pprint(sqrt_expr, use_unicode=True)

    integral = sp.integrate(sqrt_expr, (X, 0,1))
    sp.pprint(integral, use_unicode=True)


    print("Expression intégrale simplifiée :")
    simplified_integral = sp.simplify(integral)
    # Affichage de l'intégrale simplifiée
    sp.pprint(simplified_integral, use_unicode=True)




    longueur = np.zeros(len(x_values))


    # Tracer les courbes
    plt.figure(figsize=(10, 6))
    for i,x_val in enumerate(x_values) :
        P_func = sp.lambdify(X, P.subs(x, x_val), 'numpy')  # Convertir l'expression sympy en fonction numpy
        Y_points = P_func(X_points)

        # Calculer la valeur de l'intégrale simplifiée pour x = x_val

        
        sqrt_val = sqrt_expr.evalf(subs={x: x_val})
        f = sp.lambdify(X,sqrt_val, 'numpy')  # Convertir l'expression sympy en fonction numpy

        res,_ = quad(f, 0, 1)  # Intégration numérique de l'expression simplifiée
        
        print(f"Valeur de l'intégrale simplifiée pour x = {x_val} : {res}")

        plt.plot(X_points, Y_points, label=f'x = {x_val} , val = {res}')

        longueur[i] = res



    plt.show()

    plt.plot(x_values, longueur, label="Longueur en fonction de x", color="red")
    plt.title("Longueur en fonction de x")
    plt.xlabel("x")
    plt.ylabel("Longueur")
    plt.legend()
    plt.grid(True)

    # Affichage du graphique
    plt.tight_layout()
    plt.show()


def numpy_array_to_string(array):
    array_str = np.array2string(array, separator=', ', precision=8, suppress_small=True)
    return f"np.array({array_str})"

"""
print("Valeurs de longueur :")
print(numpy_array_to_string(longueur))
"""
from scipy.interpolate import interp1d
from mpl_toolkits.mplot3d import Axes3D

if np.all(np.diff(longueur) > 0) or np.all(np.diff(longueur) < 0):
    # Création de l'interpolateur inverse
    interp_x_from_l = interp1d(longueur, x_values, kind='linear', bounds_error=False, fill_value='extrapolate')



def deltaZ_poly(p1,p2,L=0.6,plot=False,nb_points=50,print_distance=False):

    """
    p1 : point 1
    p2 : point 2
    L : longueur du câble
    """
    z1 = p1[2]
    z2 = p2[2]

    dz= np.abs(z2-z1)


    if dz < 1e-4 :
        dz = 1e-4
        print("dz is too small, setting to 1e-4 to avoid division by zero.")

    d = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 )
    l = L*np.sqrt(2)/np.sqrt(d*dz)

    x_estime = interp_x_from_l(l)

    
    X_points = np.linspace(0, 1, nb_points)

    P_func = sp.lambdify(X, P.subs(x, x_estime), 'numpy')  # Convertir l'expression sympy en fonction numpy
    Z_points = P_func(X_points)


    inv = z1 > z2

    zm = np.min([z1,z2])
    zM = np.max([z1,z2])

    Z_points = Z_points*dz + zm*np.ones_like(Z_points)

    if inv :
        Z_points = Z_points[::-1]

    #print("y_points",Z_points)

    points = np.linspace(p1,p2,nb_points)

    for i,p in enumerate(points) :
        p[2] = Z_points[i]

    if print_distance :
        dist = sum(np.linalg.norm(points[i+1]-points[i]) for i in range(len(points)-1))
        print(f"Distance entre les points : {dist}")

        print(f"Longueur horizontale du câble : {d}")
        print(f"Longueur verticale du câble : {dz}")
        

        print(f"x correspondant à longueur={l} : {x_estime}")

        print(f"p1 : {p1}")
        print(f"p2 : {p2}")




    if plot :
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        points = np.array(points)
        ax.plot(points[:, 0], points[:, 1], points[:, 2], label="Trajectory")
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='red', label="Points")

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()

    

    return points

"""


points = deltaZ_poly([0,0,0],[0.5,0.0,1.0],plot=True,print_distance=True,L=2)
"""