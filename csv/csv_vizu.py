# load csv and  plot it
import numpy as np
import matplotlib.pyplot as plt

# csv header: x,y,z,yaw,mps,change_flag
# x = [:, 0]
# y = [:, 1]
# z = [:, 2]
# yaw = [:, 3]


# read the data
def read_data(filename = 'csv/sim_waypoints1.csv'):
    points = np.genfromtxt(filename, delimiter=',', skip_header=1)
    return points


# main function
if __name__ == '__main__':
    my_file = 'csv/sim_waypoints2.csv'
    points = read_data(my_file)
    plt.plot(points[:, 0], points[:, 1], 'r.-', label=my_file, alpha=0.1, linewidth=4.2)
    # plt.quiver(points[:, 0], points[:, 1], np.cos(points[:, 3]), np.sin(points[:, 3]), angles='xy', scale_units='xy', scale=2.2, alpha=0.4)
    plt.xlabel('x', fontsize=6)
    plt.ylabel('y', fontsize=6)
    plt.title(my_file, fontsize=6)
    plt.axis('equal')
    plt.grid()
    plt.legend()
    plt.show()