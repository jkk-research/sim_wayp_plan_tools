# load csv and  plot it
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# waypoint csv header: x,y,z,yaw,mps,change_flag
W_X = 0
W_Y = 1
W_Z = 2
W_YAW = 3
W_MPS = 4
# measure csv header: x,y,time,steering_angle,speed_mps,cur_lat_dist_abs,trg_way_lon_dist
M_X = 0
M_Y = 1
M_TIME = 2
M_STEER = 3
M_SPEED = 4
M_LAT_DIST = 5
M_LON_DIST = 6

# create plot layout
# f, (ax_xy, ax_st, ax_dist) = plt.subplots(nrows=3, ncols=1, sharex=False, gridspec_kw={"height_ratios": [3, 1, 1]})

f = plt.figure(constrained_layout=True)
gs = f.add_gridspec(3, 3, width_ratios=[3, 1, 1], height_ratios=[3, 1, 1])
ax_xy = f.add_subplot(gs[0, :])
ax_st = f.add_subplot(gs[1, :-1])
ax_txt = f.add_subplot(gs[1:, -1])
ax_dist = f.add_subplot(gs[-1, :-1])

# ax_xy.set_xlabel("x") # , fontsize=6
# ax_xy.set_ylabel("y")
ax_st.set_xlabel("time")
ax_st.set_ylabel("steer ang")
ax_dist.set_xlabel("time")
ax_dist.set_ylabel("lat dist")
# ax_st.set_title("steering angle")
# ax_dist.set_title("lat distance (abs)")
ax_xy.axis("equal")
ax_xy.grid()
ax_st.grid()
ax_dist.grid()
ax_xy.legend(['first stock name', 'second stock name'])
ax_txt.axis("off")



# read the data
def read_data(filename):
    points = np.genfromtxt(filename, delimiter=",", skip_header=1)
    return points

def read_data_arr(filenames):
    points = []
    for filename in filenames:
        points.append(np.genfromtxt(filename + "_xypose.csv", delimiter=",", skip_header=1))
    return points

# plot x y data
def plot_xy(points, fmt="", labl=""):
    ax_xy.plot(points[:, W_X], points[:, W_Y], fmt, label=labl, alpha=0.8)
    ax_xy.legend()

def plot_all(points, fmt="", labl=""):
    ax_xy.plot(points[:, W_X], points[:, W_Y], label=labl, alpha=0.8)
    ax_st.plot(points[:, M_TIME], points[:, M_STEER], fmt, label=labl, alpha=0.8)
    ax_dist.plot(points[:, M_TIME], points[:, M_LAT_DIST], fmt, label=labl, alpha=0.8)
    ax_xy.legend()

def read_text(filenames):
    text = ""
    for filename in filenames:
        text += "\n\n" + filename + "\n"
        with open(filename + "_metrics.csv", "r") as file:
            text += file.read()
    return text
            

# main function
if __name__ == "__main__":
    meas_arr = ["csv/tmp01", "csv/tmp02", "csv/tmp03"]
    wp_file = "csv/sim_waypoints3.csv"
    points_wp = read_data(wp_file)
    points_meas = read_data_arr(meas_arr)
    # ax_xy.plot(points_meas[0][:, M_X], points_meas[0][:, M_Y], "r.-", label="xy", alpha=0.1, linewidth=4.2)
    plot_xy(points_wp, fmt="k--", labl="waypoints")
    plot_all(points_meas[0], labl="p pursuit A")
    plot_all(points_meas[1], labl="p pursuit B")
    plot_all(points_meas[2], labl="p pursuit C")
    ax_txt.text(0.5, 0.5, read_text(meas_arr), ha="center", va="center", fontsize=8)
    # f.tight_layout()
    # f.savefig("img/csv_eval01.svg")
    plt.show()