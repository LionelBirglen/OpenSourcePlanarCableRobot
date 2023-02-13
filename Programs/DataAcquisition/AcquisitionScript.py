from cProfile import label
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
from time import time, sleep
import numpy as np
import matplotlib.pyplot as plt

T = 5 # acquisition time (s)
Vcc = 5 # Vcc to current sensors
S_rate = 1000 # sample rate max= 1000
dt = 1/S_rate
N = int(2*T*S_rate)
Values = np.zeros((N,5))
biases = [1.19,2.27,1.47,1.55] # current biases


chanels = []
for i in range(4):
    chanels.append(VoltageInput())
    chanels[-1].setIsHubPortDevice(True)
    chanels[-1].setHubPort(i)
    chanels[-1].openWaitForAttachment(1000)
    chanels[-1].setDataRate(S_rate)

print("starting acquisition in 1s ...")
sleep(1)
print("started acquisition ...")
t0 = time()
t1= t0
c = 0
while time()-t0 < T:
    tm = time()-t0

    # if time()-t1 > dt: # acquisition every dt time steps
    Values[c,0] = tm
    for i in range(4):
        VOUT = chanels[i].getVoltage()
        I = 73.3*VOUT/Vcc - 36.7
        Values[c,i+1] = I - biases[i]
        print(round(I - biases[i],3),end=" "+"\n"*(i==3))
    # print("")
    c+=1
    t1 = time()
        

for el in chanels:
    el.close()
print("end of acquisition")

np.savetxt("mesure_courants.csv",np.array(Values))

plt.plot(Values[:c,0],Values[:c,1],label="moteur1")
plt.plot(Values[:c,0],Values[:c,2],label="moteur2")
plt.plot(Values[:c,0],Values[:c,3],label="moteur3")
plt.plot(Values[:c,0],Values[:c,4],label="moteur4")
plt.legend()
plt.show()

print("b1 :", np.mean(Values[:c,1]))
print("b2 :", np.mean(Values[:c,2]))
print("b3 :", np.mean(Values[:c,3]))
print("b4 :", np.mean(Values[:c,4]))