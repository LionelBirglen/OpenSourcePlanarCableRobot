import numpy as np
from math import pi, cos, sin, sqrt
from scipy.optimize import minimize
from .wfw_anneau import *
from numpy.linalg import norm

# workspace size
L = 1
H = 1

Ra = 10e-3 # rayon de l'anneau

# Diamètre moyen de la poulie : ~55mm pour fil bleu de 1mm; 30mm pour fil vert 0.25mm
D_poulie = 51e-3
Dw = 1e-3 # wire diameter
r_init= (65/2 - 7)*0.001 # start radius
r0 = 15e-3 # spiral start diameter

X0 = np.array([[0],[0]]) # position de référence

# positions des points d'ancrage des câbles sur la base
# on définie la position 0 au centre de l'espace

B1 = np.array([[-L/2],[-H/2]]) # base câble 1 en bas à gauche
B2 = np.array([[ L/2],[-H/2]]) # base câble 2 en bas à droite
B3 = np.array([[-L/2],[ H/2]]) # base câble 3 en haut à gauche
B4 = np.array([[ L/2],[ H/2]]) # base câble 4 en haut à droite

################### reference position calculations ##############
# vecteurs directeurs des câbles
n10 = (B1 - X0)/norm(B1 - X0)
n20 = (B2 - X0)/norm(B2 - X0)
n30 = (B3 - X0)/norm(B3 - X0)
n40 = (B4 - X0)/norm(B4 - X0)

# positions sur l'anneau à la position intitiale X0
# longueur des câbles à la position X0
P10 = X0 + Ra*n10; L10 = norm(P10-B1)
P20 = X0 + Ra*n20; L20 = norm(P20-B2)
P30 = X0 + Ra*n30; L30 = norm(P30-B3)
P40 = X0 + Ra*n40; L40 = norm(P40-B4)


######################### MGI ANNEAU ##########################
def steps_from_pos(Xd):
    # Xd = [x,y]
    Xd = Xd.reshape((2,1))

    ########### desired position calculations ###################
    n1 = (B1 - Xd)/norm(B1 - Xd)
    n2 = (B2 - Xd)/norm(B2 - Xd)
    n3 = (B3 - Xd)/norm(B3 - Xd)
    n4 = (B4 - Xd)/norm(B4 - Xd)

    P1 = Xd + Ra*n1
    P2 = Xd + Ra*n2
    P3 = Xd + Ra*n3
    P4 = Xd + Ra*n4

    ############## length difference calculation #################
    DL1 = norm(P1-B1) - L10
    DL2 = norm(P2-B2) - L20
    DL3 = norm(P3-B3) - L30
    DL4 = norm(P4-B4) - L40
    DL = np.array([DL1,DL2,DL3,DL4])

    Dtheta = 2*DL/D_poulie #length to rotation with constant radius approximation

    Steps = (6533/(2*pi))*Dtheta # angle to steps conversion
    Steps = np.rint(Steps,out=np.zeros(4,int),casting='unsafe') # converting to integer

    return Steps


######################## MGI R spiral + ressorts ###############
l10 = 43e-3; l20 = 43e-3; l30 = 43e-3; l40 = 43e-3 # length of springs at rest
lr10 = 57e-3; lr20 = 55e-3; lr30 = 57e-3; lr40 = 57e-3 # mesurer la longueur initiale des ressorts
k = 264 # spring stiffness
tmin = 5; tmax = 20

# Pi02 positions sur l'anneau à la position intitiale X0
# Li02 longueur des câbles à la position X0
P102 = X0 + (Ra+lr10)*n10; L102 = norm(P102-B1)
P202 = X0 + (Ra+lr20)*n20; L202 = norm(P202-B2)
P302 = X0 + (Ra+lr30)*n30; L302 = norm(P302-B3)
P402 = X0 + (Ra+lr40)*n40; L402 = norm(P402-B4)

def steps_from_pos_v2(Xd):
    # Xd = [x,y]
    Xd = Xd.reshape((2,1))

    ########### desired position calculations ###################
    n1 = (B1 - Xd)/norm(B1 - Xd)
    n2 = (B2 - Xd)/norm(B2 - Xd)
    n3 = (B3 - Xd)/norm(B3 - Xd)
    n4 = (B4 - Xd)/norm(B4 - Xd)

    # solving W.t = 0 by optimisation
    bnds = ((tmin, tmax), (tmin, tmax), (tmin, tmax), (tmin, tmax))
    res = minimize(loss, np.ones((4,)), args=(Xd.reshape((2,))), bounds=bnds)
    dlr1 = res.x[0]/k; dlr2 = res.x[1]/k; dlr3 = res.x[2]/k; dlr4 = res.x[3]/k 

    P1 = Xd + (Ra+l10+dlr1)*n1
    P2 = Xd + (Ra+l20+dlr2)*n2
    P3 = Xd + (Ra+l30+dlr3)*n3
    P4 = Xd + (Ra+l40+dlr4)*n4

    ############## length difference calculation #################
    DL1 = norm(P1-B1) - L102
    DL2 = norm(P2-B2) - L202
    DL3 = norm(P3-B3) - L302
    DL4 = norm(P4-B4) - L402
    DL = np.array([DL1,DL2,DL3,DL4])
    
    # length to rotation with varying radius approximation (r = r0 + Dwire*theta/2pi)
    Dtheta = np.array([0.0,0.0,0.0,0.0])
    theta0 = (r_init-r0)*2*pi/Dw
    a = Dw/(4*pi); 
    b = r0 + theta0*Dw/(2*pi); 
    for i in range(4):
        c = - DL[i]; 
        delta = b**2 - 4*a*c
        Dtheta[i] = (-b+sqrt(delta))/(2*a)

    Steps = (6533/(2*pi))*Dtheta # angle to steps conversion
    Steps = np.rint(Steps,out=np.zeros(4,int),casting='unsafe') # converting to integer

    return Steps


