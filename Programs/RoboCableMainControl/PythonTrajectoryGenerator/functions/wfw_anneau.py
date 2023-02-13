import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from numpy.linalg import pinv

H = 1; L = 1

b1x = -L/2; b1y = -H/2
b2x = L/2; b2y = -H/2
b3x = -L/2; b3y = H/2
b4x = L/2; b4y = H/2

b1 = np.array([b1x,b1y])
b2 = np.array([b2x,b2y])
b3 = np.array([b3x,b3y])
b4 = np.array([b4x,b4y])

Ra = 10e-3 # rayon de l'anneau

def W(X) : # jacobian of robot
    # X shape (2,)
    n1 = (b1 - X)/np.linalg.norm(b1 - X)
    n2 = (b2 - X)/np.linalg.norm(b2 - X)
    n3 = (b3 - X)/np.linalg.norm(b3 - X)
    n4 = (b4 - X)/np.linalg.norm(b4 - X)
    W = np.zeros((2,4))
    W[:,0] = n1; W[:,1] = n2; W[:,2] = n3; W[:,3] = n4
    return W

def loss(t,x): # function to minimize to solve Wt = 0
    x.reshape((2,))
    return np.dot(np.dot(W(x),t).T,np.dot(W(x),t)) # (W.t)T . (W.t) = norm(W.t) should be zero

def workspace(N,tmin,tmax): # returns the x and y coordinates of points inside the workspace
    
    bnds = ((tmin, tmax), (tmin, tmax), (tmin, tmax), (tmin, tmax))
    xs = np.linspace(-L/2,L/2,N); ys = np.linspace(-H/2,H/2,N)
    x, y = [], []

    for i in xs:
        for j in ys:
            xd = np.array([i,j])
            res = minimize(loss, np.ones((4,)), args=(xd), bounds=bnds)

            if loss(res.x,xd) < 1:
                x.append(i); y.append(j)

    return x,y

def deadzone(N,tmin,tmax): # returns the x and y coordinates of points outside the workspace
    bnds = ((tmin, tmax), (tmin, tmax), (tmin, tmax), (tmin, tmax))
    xs = np.linspace(-L/2,L/2,N); ys = np.linspace(-H/2,H/2,N)
    x, y = [], []

    for i in xs:
        for j in ys:
            xd = np.array([i,j])
            res = minimize(loss, np.ones((4,)), args=(xd), bounds=bnds)

            if loss(res.x,xd) > 1:
                x.append(i); y.append(j)

    return x,y

def in_wfw(xd,tmin,tmax): # returns true if point is inside the workspace
    u1 = (b1 - xd)/np.linalg.norm(b1 - xd)
    u2 = (b2 - xd)/np.linalg.norm(b2 - xd)
    u3 = (b3 - xd)/np.linalg.norm(b3 - xd)
    u4 = (b4 - xd)/np.linalg.norm(b4 - xd)
    U = np.zeros((2,4));U[:,0] = u1; U[:,1] = u2; U[:,2] = u3; U[:,3] = u4
    loss = lambda f : np.dot(np.dot(U,f).T,np.dot(U,f))
    bnds = ((tmin, tmax), (tmin, tmax), (tmin, tmax), (tmin, tmax))
    res = minimize(loss, np.ones((4,)), bounds=bnds)
    return  loss(res.x) < 1
