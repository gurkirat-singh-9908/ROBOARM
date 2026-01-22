import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create a figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def plot(pos, goal, link_positions, direction=[], number_of_link=7, home=True, linestyle='-', color='blue'):
    #ax.text(0,0,0,0,color='black')
    print(f"the postion of joint 0 is {0,0,0}")
    for i in range(number_of_link):
        x = [pos[i*3], pos[(i+1)*3]]
        y = [pos[i*3+1], pos[(i+1)*3+1]]
        z = [pos[i*3+2], pos[(i+1)*3+2]]
        #print(x,y,z)

        # Plot current position links
        ax.plot(x, y, z, linestyle=linestyle, color=color)
        #print(x[0],y[0],z[0])
        '''
        if ((x[0]!=x[1])&(y[0]!=y[1])&(z[0]!=z[1])):
            ax.text(x[1],y[1],z[1],i+1,color='black')
            print(f"the postion of joint {i+1} is {x[1],y[1],z[1]}")
        else:
            ax.text(x[1]+10,y[1]+10,z[1],i+1,color='black')
            print(f"the postion of joint {i+1} is {x[1],y[1],z[1]}")


        # Plot home position links
        if (home):
            x_ = [link_positions[i*3], link_positions[(i+1)*3]]
            y_ = [link_positions[i*3+1], link_positions[(i+1)*3+1]]
            z_ = [link_positions[i*3+2], link_positions[(i+1)*3+2]]
            ax.plot(x_, y_, z_, marker='.', linestyle=':', color='green')

        if i==0:
            ax.quiver(x[i], y[i], z[i], direction[i]*10, direction[i+1]*10, direction[i+2]*10)
        else:
            ax.quiver(x[-1], y[-1], z[-1], direction[i]*10, direction[i+1]*10, direction[i+2]*10)'''

        

    # Plot goal
    #ax.scatter(goal[0], goal[1], goal[2], s=100, color='red', marker='x')

    # Set the limits of the plot
    ax.set_xlim([-200, 200])
    ax.set_ylim([-200, 200])
    ax.set_zlim([-50, 100])
    # Set labels for the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def main():
    print(" ")

if __name__ == "__main__":
    main()