from scipy.optimize import minimize
import numpy as np

def minimize_with_early_stop(fun, x0, threshold=1e-3, method='L-BFGS-B', bounds=None, options=None):
    """
    Minimisation avec arrêt anticipé si la fonction objectif devient inférieure à `threshold`.

    Paramètres :
        - fun : fonction à minimiser (doit accepter un vecteur x)
        - x0 : point de départ (np.array)
        - threshold : valeur de f(x) en dessous de laquelle l'optimisation s'arrête
        - method : méthode d'optimisation (défaut 'L-BFGS-B')
        - bounds : bornes sur les variables (liste de tuples)
        - options : dictionnaire d'options scipy.optimize.minimize

    Retour :
        Un objet contenant .x, .fun, .success, .message
    """

    # Pour stocker la meilleure solution rencontrée
    best_solution = {'x': None, 'fun': None}

    # Exception personnalisée pour arrêt anticipé
    class EarlyStoppingException(Exception):
        pass

    # Callback qui vérifie la valeur de f(x)
    def early_stop_callback(xk):
        fx = fun(xk)
        best_solution['x'] = xk.copy()
        best_solution['fun'] = fx
        #print(f"[Callback] f(x) = {fx:.6f}")
        if fx < threshold:
            raise EarlyStoppingException()

    try:
        result = minimize(
            fun=fun,
            x0=x0,
            method=method,
            bounds=bounds,
            options=options,
            callback=early_stop_callback,
        )
    except EarlyStoppingException:
        print("seuil atteint")
        class ResultMock:
            def __init__(self, x, fun):
                self.x = x
                self.fun = fun
                self.success = False
                self.message = "Stopped early due to function value < threshold."
        result = ResultMock(best_solution['x'], best_solution['fun'])

    return result
