# load csv and  plot it
import numpy as np
import matplotlib.pyplot as plt

# csv header: x,y,z,yaw,mps,change_flag
# x = [:, 0]
# y = [:, 1]
# z = [:, 2]
# yaw = [:, 3]


# read the data
def read_data(filename = 'csv/sim_waypoints3.csv'):
    points = np.genfromtxt(filename, delimiter=',', skip_header=1)
    return points

def rotate_points(points, theta):
    t = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    rotatedxy = np.dot(points[:, 0:2], t)
    points[:, 0] = rotatedxy[:, 0]  
    points[:, 1] = rotatedxy[:, 1]
    return points

def scale_points(points,scale=60.0):
    # scale xy points
    points[:, 0] =  points[:, 0] / scale
    points[:, 1] =  points[:, 1] / scale
    return points

def plot_points(points):
    plt.plot(points[:, 0], points[:, 1], 'r.-', label='xy', alpha=0.1, linewidth=4.2)
    plt.quiver(points[:, 0], points[:, 1], np.cos(points[:, 3]), np.sin(points[:, 3]), angles='xy', scale_units='xy', scale=2.2, alpha=0.4)
    plt.xlabel('x', fontsize=6)
    plt.ylabel('y', fontsize=6)
    plt.title('CSV plot', fontsize=6)
    plt.axis('equal')
    plt.grid()
    plt.show()

def recalc_yaw(points):
    angle = 0.0
    prev_angle = 0.0
    # get next point orientation
    for i in range(len(points)-1):
        dx = points[i + 1, 0] - points[i, 0]
        dy = points[i + 1, 1] - points[i, 1]
        angle = np.arctan2(dy, dx) 
        points[i, 3] = angle
        points[i, 2] = 0.0
        points[i, 4] = 4.8 - np.abs(prev_angle - angle) * 2 
        points[i, 5] = 0
        prev_angle = angle
        # print(f"Index: {i}, Point: {point}")
    return points


# main function
if __name__ == '__main__':
    points = read_data()
    # extend [:, 4:5]
    points = np.append(points, np.zeros((len(points), 2)), axis=1)
    # resample points
    points = points[::4, :]
    points = scale_points(points, 10.0)
    points = rotate_points(points, -0.85 + np.pi/2*3)
    points = recalc_yaw(points)
    plt.plot(points[:, 0], points[:, 1], 'r.-', label='xy', alpha=0.1, linewidth=4.2)
    plt.quiver(points[:, 0], points[:, 1], np.cos(points[:, 3]), np.sin(points[:, 3]), angles='xy', scale_units='xy', scale=2.2, alpha=0.4)
    # print(points.shape)
    plt.xlabel('x', fontsize=6)
    plt.ylabel('y', fontsize=6)
    plt.title('CSV plot', fontsize=6)
    plt.axis('equal')
    plt.grid()
    plt.show()
    # save points
    # file_saved = 'csv/sim_waypoints2.csv'
    # f=open(file_saved,'a')
    # np.savetxt(f, np.array(['x', 'y', 'z', 'yaw', 'mps', 'change_flag']), newline=",", fmt="%s")
    # f.write("\n")
    # np.savetxt(f, points, delimiter=',', fmt='%.4f')