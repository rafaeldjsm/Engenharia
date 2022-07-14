from scipy.stats import norm
import numpy as np

def cvsn(n):
    '''
    Gera uma curva S de desembolso considerando uma S padrão normal
    '''
    ynac = np.array([norm.cdf((t+1)/n,0.5,0.2) for t in range(n)])
    y = ynac.copy()
    y[1:] -= ynac[:-1].copy()
    # Garantindo que o acumulado total seja 100%
    y = y/sum(y)
    ynac = np.cumsum(y)
    return ynac, y
