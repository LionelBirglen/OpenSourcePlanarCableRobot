# exportation de la trajectoire sous forme de tableau d'entier dans un fichier traj.h par défault
def save_traj(Steps, VSteps, Times, name="traj"):
    N = Steps.shape[0]
    code = "int32_t Traj2[][4] = {  {"
    for i in range(N-1):
        code += str(Steps[i][0]) + ","
        code += str(Steps[i][1]) + ","
        code += str(Steps[i][2]) + ","
        code += str(Steps[i][3]) + "},\n\t\t\t\t\t\t{"
    
    code += str(Steps[-1][0]) + ","
    code += str(Steps[-1][1]) + ","
    code += str(Steps[-1][2]) + ","
    code += str(Steps[-1][3]) + "} };\n"

    code += "\nint32_t Vit2[][4] = {   {"

    for i in range(N-1):
        code += str(VSteps[i][0]) + ","
        code += str(VSteps[i][1]) + ","
        code += str(VSteps[i][2]) + ","
        code += str(VSteps[i][3]) + "},\n\t\t\t\t\t\t{"
    
    code += str(VSteps[-1][0]) + ","
    code += str(VSteps[-1][1]) + ","
    code += str(VSteps[-1][2]) + ","
    code += str(VSteps[-1][3]) + "} };"
    code += "\n\nint Traj2Fin="+str(N)+";"

    # code += "\ndouble Traj_Step_times["+str(N)+"] = {"
    # for i in range(N-1):
    #     code += str(Times[i]) + ","
    # code += str(Times[-1]) + "};\n"

    text_file = open("./"+name+".h", "w")

    text_file.write(code)

    text_file.close()

# exportation de la trajectoire interpolée sous forme de tableau float dans un fichier interp_traj.h par défault
def save_interp_traj(Steps, VSteps, Times, name = "interp_traj"):
    N = Steps.shape[0]
    code = "const int N_steps = "+str(N)+";\n"

    code += "float M1_steps["+str(N)+"] = {"
    for i in range(N-1):
        code += str(Steps[i][0]) + ","
    code += str(Steps[-1][0]) + "};\n"

    code += "float M2_steps["+str(N)+"] = {"
    for i in range(N-1):
        code += str(Steps[i][1]) + ","
    code += str(Steps[-1][1]) + "};\n"

    code += "float M3_steps["+str(N)+"] = {"
    for i in range(N-1):
        code += str(Steps[i][2]) + ","
    code += str(Steps[-1][2]) + "};\n"

    code += "float M4_steps["+str(N)+"] = {"
    for i in range(N-1):
        code += str(Steps[i][3]) + ","
    code += str(Steps[-1][3]) + "};\n\n"

    code += "float M1_V_steps["+str(N)+"] = {"
    for i in range(N-1):
        code += str(VSteps[i][0]) + ","
    code += str(VSteps[-1][0]) + "};\n"

    code += "float M2_V_steps["+str(N)+"] = {"
    for i in range(N-1):
        code += str(VSteps[i][1]) + ","
    code += str(VSteps[-1][1]) + "};\n"

    code += "float M3_V_steps["+str(N)+"] = {"
    for i in range(N-1):
        code += str(VSteps[i][2]) + ","
    code += str(VSteps[-1][2]) + "};\n"

    code += "float M4_V_steps["+str(N)+"] = {"
    for i in range(N-1):
        code += str(VSteps[i][3]) + ","
    code += str(VSteps[-1][3]) + "};\n"

    code += "\nfloat Step_times["+str(N)+"] = {"
    for i in range(N-1):
        code += str(Times[i]) + ","
    code += str(Times[-1]) + "};\n"

    # easier python array to test

    code += "\n\n/*\n"
    code += "X1 = np.array(["
    for i in range(N-1):
        code += str(Steps[i][0]) + ","
    code += str(Steps[-1][0]) + "])\n"

    code += "X2 = np.array(["
    for i in range(N-1):
        code += str(Steps[i][1]) + ","
    code += str(Steps[-1][1]) + "])\n"

    code += "X3 = np.array(["
    for i in range(N-1):
        code += str(Steps[i][2]) + ","
    code += str(Steps[-1][2]) + "])\n"

    code += "X4 = np.array(["
    for i in range(N-1):
        code += str(Steps[i][3]) + ","
    code += str(Steps[-1][3]) + "])\n"

    code += "\nV1 = np.array(["
    for i in range(N-1):
        code += str(VSteps[i][0]) + ","
    code += str(VSteps[-1][0]) + "])\n"

    code += "V2 = np.array(["
    for i in range(N-1):
        code += str(VSteps[i][1]) + ","
    code += str(VSteps[-1][1]) + "])\n"

    code += "V3 = np.array(["
    for i in range(N-1):
        code += str(VSteps[i][2]) + ","
    code += str(VSteps[-1][2]) + "])\n"

    code += "V4 = np.array(["
    for i in range(N-1):
        code += str(VSteps[i][3]) + ","
    code += str(VSteps[-1][3]) + "])\n"

    code += "*/" 

    text_file = open("./"+name+".h", "w")

    text_file.write(code)

    text_file.close()

# exportation des points x,y de la trajectoire dans un fichier txt
def save_traj_points(ts,xs,ys):
    N = len(ts)
    code = "t\tx théorique\ty théoriqe\n"
    for i in range(N):
        code += str(ts[i]) + "\t"
        code += str(xs[i]) + "\t"
        code += str(ys[i]) + "\n"

    code = code.replace(".",",")
    text_file = open("./traj_saves/traj_points.txt", "w")
    text_file.write(code)
    text_file.close()