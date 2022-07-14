def cvsn(n):
    '''
    Gera uma curva S de desembolso considerando uma S padrão normal
    '''
    x = np.arange(n)
    ynac = [norm.cdf((t+1)/n,0.5,0.2) for t in x]
    y = [ynac[0]] + [ynac[t+1]-ynac[t] for t in x[:-1]]
    # Garantindo que o acumulado total seja 100%
    y = y/sum(y)
    ynac = np.cumsum(y)
    return ynac, y
