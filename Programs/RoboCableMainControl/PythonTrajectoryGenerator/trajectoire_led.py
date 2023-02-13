from matplotlib.backend_bases import MouseButton
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from functions.mgi_led import *
from functions.traj_save import *
from functions.wfw_led import *
from math import sqrt
from numpy.linalg import norm

np.seterr(divide='ignore', invalid='ignore') # to ingnore warning messages

max_V = 0.20 # maximum operational speed m/s
Amax = 20000 #max acceleration steps/s2

#update plots on events
def update_all(xs, ys, v, N, int_type, fig):
    plt.cla()

    plot_interpolate_traj(xs,ys,v,N,int_type[0])

    plt.plot(xdead, ydead, "r.", label = "deadzone")
    plt.plot(xs, ys, "o", label = "selected points")
    plt.xlim(-L/2,L/2)
    plt.ylim(-H/2,H/2)
    plt.legend(loc="lower right", bbox_to_anchor=(1.6,0.02))

    text1 = "Enter to finish\n\nX : clear\n\nBackspace : delete last point"
    text1 += "\n\nUp/Down : +/- points"
    text1 += "\n\nScroll : +/- speed"
    text1 += "\n\nT : interpolation type (" + int_type[0] + ")"
    myfont = {'size' : 12}

    text2 = "Points : " + str(N[0])
    text2 += "\nSpeed : " + str(round(100*v[0],2)) + " cm/s (max : " + str(int(100*max_V)) + ")"
    if len(xs) >= 4:
        eta = round(time_estimate(xs, ys, v[0], N[0], int_type[0]),2)
        text2 += "\nEstimasted time : " + str(eta) + "s"
    else:
        text2 += "\nEstimasted time : inf"
    
    if len(fig.texts) > 1 : 
        del fig.texts[0]
        del fig.texts[0]
    fig.text(0.6, 0.4, text1, font= myfont)
    fig.text(0.7, 0.2, text2, font= myfont)
    
    plt.draw()

#plotting interpolated trajectory
def plot_interpolate_traj(xs,ys,v,N,type):
    if len(xs) >= 4:
        t = np.linspace(0,len(xs),len(xs))
        t2 = np.linspace(0,len(xs),10*len(xs))
        t3 = np.linspace(0,len(xs),N[0])
        
        fy = interp1d(t, np.array(ys), kind = type)
        fx = interp1d(t, np.array(xs), kind = type)

        plt.plot(fx(t2), fy(t2), label = "continous trajectory")
        plt.plot(fx(t3), fy(t3),"x", label = "exported points")



# mouse click event handler
def on_click(event, xs, ys, v, N, int_type, fig):
    if event.button is MouseButton.LEFT:
        if(event.xdata and event.ydata):
            xs.append(event.xdata)
            ys.append(event.ydata)
            # print(event.xdata,event.ydata)
    
    update_all(xs, ys, v, N, int_type, fig)

# keyboard press event handler
def on_press(event, xs, ys, v, N, int_type, fig):
    if event.key == 'x':
        print("pressed x : trajectory cleared !")
        xs[:], ys[:], N[0] = [0], [0], 50

    if event.key == 'backspace':
        print("pressed return : last point cleared !")
        del xs[-1]; del ys[-1]; N[0] = 50
    
    if event.key == 'up':
        print("pressed up : +10 points to trajectory")
        N[0] += 10
    
    if event.key == 'down':
        print("pressed down : -10 points to trajectory")
        N[0] -= 10
    
    if event.key == 't':
        if int_type[0] == "linear":
            int_type[0] = "cubic"
        else :
            int_type[0] = "linear"
        print("pressed t : changed interpolation type to ", int_type[0])
    
    if event.key == 'enter':
        print("pressed enter : closing")
        plt.close()
    
    update_all(xs, ys, v, N, int_type, fig)

# mouse scroll event handler
def on_scroll(event, xs, ys, v, N, int_type, fig):
    if event.button == "up":
        v[0] = min(max_V, v[0] + 0.01)
    else :
        v[0] = max(0.01,v[0] -0.01)
    update_all(xs, ys, v, N, int_type, fig)

# main function
def trajectory_by_plot(xs,ys,v,N,int_type):
    fig = plt.figure(figsize=(10,4.5))
    plt.subplot(121)
    plt.xlim(-L/2,L/2)
    plt.xlim(-L/2,L/2)
    plt.axis('equal')
    update_all(xs, ys, v, N, int_type, fig)

    plt.connect('button_press_event', lambda x: on_click(x, xs, ys, v, N, int_type, fig))
    plt.connect('key_press_event', lambda x: on_press(x, xs, ys, v, N, int_type, fig))
    plt.connect('scroll_event', lambda x: on_scroll(x, xs, ys, v, N, int_type, fig))

    plt.show()



# estimation of trajectory time
def time_estimate(xs, ys, v, N, int_type):
    Steps = np.zeros((N+1,4))
    Times = np.zeros(N)

    t2 = np.linspace(0,len(xs),N+1)
    fy = interp1d(np.linspace(0,len(xs),len(xs)), np.array(ys), kind = int_type)
    fx = interp1d(np.linspace(0,len(xs),len(xs)), np.array(xs), kind = int_type)
    x_out = fx(t2)
    y_out = fy(t2)

    for i in range(N+1):
        Steps[i] = steps_from_pos(np.array([x_out[i],y_out[i]]))
  
    for i in range(1,N+1):
        dX = np.array([x_out[i]-x_out[i-1],y_out[i]-y_out[i-1]])
        dt = norm(dX)/v

        ddS = 0
        if i < N:
            ddS = Steps[i+1]-2*Steps[i]+Steps[i-1]
            astep = ddS/(dt**2)
        else:
            ddS = Steps[i]-2*Steps[i-1]+Steps[i-2]
            astep = ddS/(dt**2)
        
        dt = max(dt,sqrt(ddS[np.argmax(astep)]/Amax))

        if i == 1 : Times[0] = dt
        else : Times[i-1] = Times[i-2] + dt

    return Times[-1]

# gettings motor steps from points
def get_traj_steps(xs, ys, v, N, int_type):
    Steps = np.zeros((N+1,4))
    VSteps = np.zeros((N,4))
    ASteps = np.zeros((N,4))
    Times = np.zeros(N)
    
    ########## interpolation to N points
    t2 = np.linspace(0,len(xs),N+1)

    fy = interp1d(np.linspace(0,len(xs),len(xs)), np.array(ys), kind = int_type)
    fx = interp1d(np.linspace(0,len(xs),len(xs)), np.array(xs), kind = int_type)
    x_out = fx(t2)
    y_out = fy(t2)

    #################### IK calculation ######################################
    for i in range(N+1):
        Steps[i] = steps_from_pos(np.array([x_out[i],y_out[i]]))
    
    ################## speed acceleration and time calculation ############
  
    for i in range(1,N+1):
        dS = Steps[i]-Steps[i-1]
        
        dX = np.array([x_out[i]-x_out[i-1],y_out[i]-y_out[i-1]])
        dt = norm(dX)/v
        
        vstep = dS/dt
        ddS = 0
        if i < N:
            ddS = Steps[i+1]-2*Steps[i]+Steps[i-1]
            astep = ddS/(dt**2)
        else:
            ddS = Steps[i]-2*Steps[i-1]+Steps[i-2]
            astep = ddS/(dt**2)
        
        if max(abs(astep))>Amax:
            ind = np.argmax(astep)
            astep = Amax*astep/max(abs(astep))
            dt = sqrt(ddS[ind]/Amax)
            vstep = dS/dt

        if i == 1 : Times[0] = round(dt,2)
        else : Times[i-1] = round(Times[i-2] + dt,2)

        VSteps[i-1] = np.around(abs(vstep))
        ASteps[i-1] = np.around(abs(astep))
    
    Steps = Steps[1:,:] # removing Zero coordinates
    return Steps, VSteps, ASteps, Times

# gettings motor steps from points with model v2
def get_traj_steps_v2(xs, ys, v, N, int_type):
    Steps = np.zeros((N+1,4))
    VSteps = np.zeros((N,4))
    ASteps = np.zeros((N,4))
    Times = np.zeros(N)
    
    ########## interpolation to N points
    t2 = np.linspace(0,len(xs),N+1)

    fy = interp1d(np.linspace(0,len(xs),len(xs)), np.array(ys), kind = int_type)
    fx = interp1d(np.linspace(0,len(xs),len(xs)), np.array(xs), kind = int_type)
    x_out = fx(t2)
    y_out = fy(t2)

    #################### IK calculation ####################
    for i in range(N+1):
        Steps[i] = steps_from_pos_v2(np.array([x_out[i],y_out[i]]))
    
    
    ################## Speed and time calculation
    
    # Vmax = 10000 #steps/s
  
    for i in range(1,N+1):
        dS = Steps[i]-Steps[i-1]
        
        dX = np.array([x_out[i]-x_out[i-1],y_out[i]-y_out[i-1]])
        dt = norm(dX)/v
        
        vstep = dS/dt
        ddS = 0
        if i < N:
            ddS = Steps[i+1]-2*Steps[i]+Steps[i-1]
            astep = ddS/(dt**2)
        else:
            ddS = Steps[i]-2*Steps[i-1]+Steps[i-2]
            astep = ddS/(dt**2)
        
        if max(abs(astep))>Amax:
            ind = np.argmax(astep)
            astep = Amax*astep/max(abs(astep))
            dt = sqrt(ddS[ind]/Amax)
            vstep = dS/dt


        if i == 1 : Times[0] = round(dt,2)
        else : Times[i-1] = round(Times[i-2] + dt,2)

        VSteps[i-1] = np.around(abs(vstep))
        ASteps[i-1] = np.around(abs(astep))
    
    Steps = Steps[1:,:] # removing Zero coordinates
    return Steps, VSteps, ASteps, Times

# getting the x,y points of trajectory
def get_traj_points(v, N, xs, ys, int_type):
    eta = time_estimate(xs, ys, v, N, int_type)
    t2 = np.linspace(0,eta,N+1)

    fy = interp1d(np.linspace(0,eta,len(xs)), np.array(ys), kind = int_type)
    fx = interp1d(np.linspace(0,eta,len(xs)), np.array(xs), kind = int_type)
    x_out = fx(t2)
    y_out = fy(t2)
    return t2,x_out,y_out



tmin = 5; tmax = 20 # min and max cable tensions
xdead,ydead = deadzone(20, tmin, tmax) # points that are outside the workspace

# using lists to modify the value in the functions
v = [0.1] # trajecory speed m/s
N = [50] # number of points of the trajectory to export
xs, ys = [0], [0] # start trajectory at origin
int_type = ['cubic']
int_N = 50

####### points can be added manually
xs = [0,   0,  0,-0.3, 0.3,  0, 0]
ys = [0,-0.3,0.2, 0.2, 0.2,0.2, 0]

trajectory_by_plot(xs, ys, v, N, int_type)



# ############### trajectory points interpolations ####################
# Steps, VSteps, ASteps, Times = get_traj_steps(xs,ys,v[0],N[0],int_type[0])
# Steps2, VSteps2, ASteps2, Times2 = get_traj_steps_v2(xs,ys,v[0],N[0],int_type[0])
# Steps_i, VSteps_i, ASteps_i, Times_i = get_traj_steps(xs,ys,v[0],int_N,int_type[0])
# Steps_i2, VSteps_i2, ASteps_i2, Times_i2 = get_traj_steps_v2(xs,ys,v[0],int_N,int_type[0])


# ############## saving points to text file to compare results #########
# ts,xs,ys = get_traj_points(v[0], N[0], xs, ys, int_type[0])
# save_traj_points(ts, xs, ys)

# ############## saving trajectory points interpolations ####################
# save_traj(Steps, VSteps, Times)
# save_interp_traj(Steps_i, VSteps_i, Times_i)

# save_traj(Steps2, VSteps2, Times2,"traj2")
# save_interp_traj(Steps_i2, VSteps_i2, Times_i2,"interp_traj2")



############## test plots ####################################""

# pl_step = np.concatenate((np.zeros((1,4)), Steps), axis=0)
# pl_vstep = np.concatenate((np.zeros((1,4)), VSteps), axis=0)
# pl_astep = np.concatenate((np.zeros((1,4)), ASteps), axis=0)
# pl_step2 = np.concatenate((np.zeros((1,4)), Steps2), axis=0)
# pl_vstep2 = np.concatenate((np.zeros((1,4)), VSteps2), axis=0)
# pl_astep2 = np.concatenate((np.zeros((1,4)), ASteps2), axis=0)

# x = np.concatenate((np.zeros(1), Times), axis=0)
# x2 = np.concatenate((np.zeros(1), Times2), axis=0)
# plt.clf()
# for i in range(4):
#     plt.subplot(2,2,i+1)
#     plt.plot(x, pl_step[:,i], label = "pos")
#     plt.plot(x, pl_vstep[:,i], label = "vit")
#     plt.plot(x, pl_astep[:,i], label = "acc")
#     plt.plot(x2, pl_step2[:,i], label = "pos2")
#     plt.plot(x2, pl_vstep2[:,i], label = "vit2")
#     plt.plot(x2, pl_astep2[:,i], label = "acc2")
#     plt.legend()
# plt.show()

