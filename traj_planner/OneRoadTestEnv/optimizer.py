import matplotlib.pyplot as plot
import numpy as np
#import nlopt as nl
from scipy.optimize import minimize
import pickle

# class Optimizer:
#     """
#     wrapper class for scipy optimize
#     """
#     def __init__(self, method, constraints):
#         """
#         inititialize optimizer
#         @attribute algorithm : algos available in nlopt
#         @attribute n : number of optimization parameters 
#         """
#         self.metdod = method
#         self.n = n
#         self.optimizer = nl.opt(algorithm, n)

#     def addconstraint(self,):
#         pass

# class OptimizerConstrains(Optimizer):
#     def __init__(self):
#         pass

pickle_in = open('trajectory.p','rb')
trajectory = pickle.load(pickle_in)
pickle_in.close()
states = np.array([s['position'] for s in trajectory], dtype='float64')


h = 1
vdes = 28


def euclid_dist(a,b):
    dist = (a - b)**2
    dist = np.sum(dist, axis=1)
    dist = np.sqrt(dist)
    return dist

def x_d(x):
    x_dash = np.zeros(np.shape(x), dtype='float64')
    for i in range(len(x)):
        if i == 0:
            x_dash[i] = 0.5 * x[i+1] / h
        elif i == len(x)-1:
            x_dash[i] = - 0.5 * x[i-1] / h
        else:
            x_dash[i] = 0.5 * (x[i+1]-x[i-1]) / h
    return x_dash

def x_dd(x):
    x_dash2 = np.zeros(np.shape(x), dtype='float64')
    for i in range(len(x)):
        if i == 0:
            x_dash2[i] = (x[i+1] + 2*x[i]) / (h*h)
        elif i == len(x)-1:
            x_dash2[i] = (x[i-1] - 2*x[i]) / (h*h)
        else:
            x_dash2[i] = (x[i-1] - 2*x[i] + x[i+1]) / (h*h)
    return x_dash2

def x_ddd(x):
    x_dash3 = np.zeros(np.shape(x), dtype='float64')
    return x_dash3

def calculatevdes(x, x_dash):
    left = x_dash[:,:]
    right = x_dash[:,:]
    left[:,0] = 2.5
    right[:,0] = 0
    return vdes * np.array([[0,-1], [1,0]], dtype='float64') @ (0.5 * (euclid_dist(x_dash, left) + euclid_dist(x_dash,right)))

def psi(x):
    return np.arctan2(np.divide(x[:,1],x[:,0]))

def joffs(x):
    left = x[:,:]
    right = x[:,:]
    left[:,0] = 2.5
    right[:,0] = 0
    dleft = euclid_dist(x,left)
    dright = euclid_dist(x, right)
    print(np.shape(dleft))
    return np.abs(0.5 * (dleft + dright))**2

def jvel(x):
    x_dash = x_d(x)
    return np.abs(calculatevdes(x,x_dash) - x_dash)**2

def jacc(x):
    return np.abs(x_dd(x))**2

def jjerk(x):
    x_dash3 = x_ddd(x)
    return np.abs(x_dash3)**2

def jyaw(x):
    x_dash = x_d(x)
    return psi(x_dash)**2

def objective(x):
    w = np.array([1,1,1,0,1], dtype='float64')
    j = np.array([joffs(x), jvel(x), jacc(x), jjerk(x), jyaw(x)])
    return np.dot(w.t,j)

# print(objective(states))

def objective1(x):
    return (x[0]*x[3]*(x[0]+x[1]+x[2])+x[3])

def constraint1(x):
    return x[0]*x[1]*x[2]*x[3] - 25.0
def constraint2(x):
    return 40.0 - np.sum(x**2)
x = [20.,1.,1.,50.]
bnd = (1.0,5.0)
bnds= (bnd,bnd,bnd,bnd)
con1 = {'type':'ineq', 'fun': constraint1}
con2 = {'type':'eq', 'fun': constraint2}
cons =[con1, con2]
sol = minimize(objective1, x, method='SLSQP', bounds=bnds, constraints=cons)
print(sol.x)