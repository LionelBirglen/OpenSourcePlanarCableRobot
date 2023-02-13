"""
this script was to evaluate the maximum operational speed of the robot
assumming 10000 steps/s being the maximum rotational speed of the motors
"""

import numpy as np
from numpy.linalg import pinv
from scipy.optimize import minimize, minimize_scalar
from math import pi

# dimensions du robot
L = 1
H = 1

B1 = np.array([-L/2,-H/2]) # base câble 1 en bas à gauche
B2 = np.array([ L/2,-H/2]) # base câble 2 en bas à droite
B3 = np.array([-L/2, H/2]) # base câble 3 en haut à gauche
B4 = np.array([ L/2, H/2]) # base câble 4 en haut à droite

Ra = 10e-3 # rayon de l'anneau

# Diamètre moyen de la poulie : ~55mm pour fil bleu de 1mm; 30mm pour fil vert 0.25mm
D_poulie = 51e-3# pour les câble court 65mm pour les câbles long


def Jcb(Xd): # jacobian of the robot
    n1 = -(B1 - Xd)/np.linalg.norm(B1 - Xd)
    n2 = -(B2 - Xd)/np.linalg.norm(B2 - Xd)
    n3 = -(B3 - Xd)/np.linalg.norm(B3 - Xd)
    n4 = -(B4 - Xd)/np.linalg.norm(B4 - Xd)

    J = np.zeros((2,4))
    J[:,0] = n1; J[:,1] = n2; J[:,2] = n3; J[:,3] = n4
    return J

Xd = np.array([0,0])
N = 50
dx = 0.001
x = np.linspace(-L/2+dx,L/2-dx,N) # discretizing the workspace
y = np.linspace(-H/2+dx,H/2-dx,N)
vmax = 10 # initial guess
ldotmax = 0.3 # maximum cable speed m/s (10000/6533)*2*PI*Rpoulie

def loss(vmax): # function to minimize
    Lmax = 0
    # xdot is an array containing speed vectors in 8 direction at the vmax speed
    xdot = np.array([[0,vmax],[vmax,0],[0,-vmax],[-vmax,0],[vmax,vmax],[-vmax,-vmax],[vmax,-vmax],[-vmax,vmax]])
    for i in x: # for every position of the workspace
        for j in y:
            J = Jcb(np.array([i,j])) # we calculate the jacobian
            
            for k in range(8): # for every possible direction we calculate the maximum cable speed needed
                L = np.dot(pinv(J),xdot[k]) #
                Lmax = max(Lmax,max(abs(L)))
    return abs(Lmax-ldotmax) # return the difference

res = minimize_scalar(loss) ## minimizing this function gives vmax as the highest speed acheivable
print(res)

# print(loss(0.24869631643550857))
# print(loss(1))
# print(loss(0.23))


# Steps = np.ones((N+1,4))
# dX = np.array([0.01,0.01])
# xdot = 0.25*dX/np.linalg.norm(dX)
# J = Jcb(np.array([0.1,0.1]))
# Ldot = np.dot(pinv(J),xdot)
# vstep = (2/D_poulie)*(6533/(2*pi))*Ldot
# print(Steps[0]," ", vstep)
# print(Steps[0]/vstep)

