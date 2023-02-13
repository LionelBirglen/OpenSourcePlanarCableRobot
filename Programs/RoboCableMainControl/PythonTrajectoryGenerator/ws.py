from functions.wfw_led import *
import matplotlib.pyplot as plt
from matplotlib import rc

font = {'weight' : 'normal',
        'size'   : 14}

rc('font', **font)


x,y = workspace(100,2,10)
x1,y1 = workspace(100,2,4)
# x1,y1 = workspace(10,2,5)

plt.plot(x,y,".",label="Tmin = 2\nTmax = 10")
plt.plot(x1,y1,".",label="Tmin = 2\nTmax = 4")
plt.axis("equal")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.subplots_adjust(left=0.13)
plt.subplots_adjust(right=0.69)
plt.legend(bbox_to_anchor=(1, 0.6))
plt.show()