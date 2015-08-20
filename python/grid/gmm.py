import numpy as np
from pypr.mixture.clustering import gmm

'''
Wraps some PyPr functions for easy grouping of different GMMs.
Uses a couple different functions.
'''
class GMM:
    
    def __init__(k=1,data=None):
        self.k = 1
        if not data == None:
            self.fit(data)
        self.mu = None
        self.sigma = None
        self.pi = None

    def fit(data):
        self.mu,self.sigma,self.pi,ll = gmm.em_gm(data,k)

    def score(data):

