import numpy as np
import robot_visuals
from robot_visuals import plot

link = [20.5, 28, 28.5, 4, 3.25, 20]
link_positions=[0, 0, 0, 0, 0, link[0], 0, 0, link[0]+link[1], link[2], 0, link[0]+link[1], link[2]+link[3], 0, link[0]+link[1], link[2]+link[3]+link[4], 0, link[0]+link[1], link[2]+link[3]+link[4]+link[5], 0, link[0]+link[1]]

def dh(goal=[10,10,10], theeta=[0, 0, 0, 0, 0, 0], link = link, link_position=link_positions):
    final = np.eye(4)
    pos = np.array([0, 0, 0])
    z_vector = np.array([0,0,1])

    #checks if the angles are in rad if not converts them 
    if any(abs(angle) > 2 * np.pi for angle in theeta):
        theeta = np.radians(theeta)

    dh_matrix = [
        [theeta[0]        , +np.pi/2, 0.0     , link[0]],
        [theeta[1]+np.pi/2, 0.0     , +link[1], 0.0],
        [theeta[2]        , +np.pi/2, 0.0     , 0.0],
        [theeta[3]        , -np.pi/2, 0.0     , link[2] + link[3]],
        [theeta[4]        , +np.pi/2, 0.0     , 0.0]              ,
        [theeta[5]        , 0.0     , 0.0     , link[5] + link[4]]
    ]

    for i, val in enumerate(dh_matrix):
        #print(val)
        tb = np.array([
            [np.cos(val[0]), -np.sin(val[0]) * np.cos(val[1]), np.sin(val[0]) * np.sin(val[1]), val[2] * np.cos(val[0])],
            [np.sin(val[0]), np.cos(val[0]) * np.cos(val[1]), -np.cos(val[0]) * np.sin(val[1]), val[2] * np.sin(val[0])],
            [0.0, np.sin(val[1]), np.cos(val[1]), val[3]],
            [0.0, 0.0, 0.0, 1.0]
        ])
        #print(np.round(tb))
        final = np.dot(final, tb)
        #print(np.round(final))
        pos = np.concatenate((pos, [final[0, 3], final[1, 3], final[2, 3]]), axis=0)
        z_vector = np.concatenate((z_vector, [final[0, 2], final[1, 2], final[2, 2]]), axis=0)

    #print(pos)
    #print(z_vector)
    #plot(pos, goal, link_positions, z_vector, len(link),home=False)
    return final,pos

def main(local=False):
    if local:
        theeta = input().split()
        theeta = [float(a) for a in theeta]
    else:
        theeta = float(input())
        theeta = [theeta,theeta,theeta,theeta,theeta,theeta]
    #print(theeta)
    final,pos = dh(goal=[-12,-6.9,93],theeta=theeta)
    print(np.array2string(final,separator=', '))
    #print(f"wx = {final[0,3]-final[0,2]*23.25}\nwy = {final[1,3]-final[1,2]*23.25}\nwz = {final[2,3]-final[2,2]*23.25}")
    return(final)

if __name__ == "__main__":
    main()
