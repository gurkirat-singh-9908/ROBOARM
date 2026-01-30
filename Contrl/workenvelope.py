import dh 
from dh import dh
import numpy as np
workspace = []

def workenvelope(goal, theeta, link, link_positions):
    for a in range(0,181,900):
        for b in range(0,181,90):
            for c in range (0,181,90):
                for d in range(0,181,90):
                    for e in range(0,181,90):
                        for f in range(0,181,90):
                            theeta = [a,b,c,d,e,f]
                            final,pos = dh(goal, theeta=theeta, link=link, link_positions=link_positions)
                            intermediate_points = pos[-3:]
                            workspace.append(intermediate_points)
                            workspace.append(theeta)
                            #print(f"the pos at angles {theeta} = {workspace}")
    x_filtered_workspace = {workspace[i][0] for i in range(0, len(workspace), 2)}

    x_filtered_workspace_with_angles = {workspace[i][0]:workspace[i+1] for i in range(0, len(workspace), 2)}

    #y_filtered_workspace = {np.abs(m[1]) for m in workspace}
    #z_filtered_workspace = {np.abs(m[2]) for m in workspace}
    #cworkspace = [n[0] for n in workspace]
    x = [max(x_filtered_workspace),min(x_filtered_workspace)]
    print(x_filtered_workspace_with_angles.get(x[0]))
    #y = [max(y_filtered_workspace),min(y_filtered_workspace)]
    #z = [max(z_filtered_workspace),min(z_filtered_workspace)]
    #cx = [max(cworkspace),min(cworkspace)]
    #return(x,y,z)
    return(x)

def main():
    #workenvelope(goal=[10,10,10], theeta=[0, 0, 0, 0, 0, 0], link=[20.5, 25.5, 29.5, 3.8, 2.5, 4, 10], link_positions=[0, 0, 0, 0, 0, 20.5, 0, 0, 46.5, 29.5, 0, 46.5, 43.3, 0, 46.5, 43.3, 0, 40.2, 43.3, 0, 30.2])
    x = workenvelope(goal=[10,10,10], theeta=[0, 0, 0, 0, 0, 0], link=[20.5, 25.5, 29.5, 3.8, 2.5, 4, 10], link_positions=[0, 0, 0, 0, 0, 20.5, 0, 0, 46.5, 29.5, 0, 46.5, 43.3, 0, 46.5, 43.3, 0, 40.2, 43.3, 0, 30.2])
    print(x)
if __name__=="__main__":
    main()


