# import necessary module
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
def call_back(event):
    axtemp=event.inaxes
    x_min, x_max = axtemp.get_xlim()
    y_min, y_max = axtemp.get_ylim()
    fanwei = (x_max - x_min) / 10
    fanwei1 = (y_max - y_min) / 10
    if event.button == 'up':
        axtemp.set(xlim=(x_min + fanwei, x_max - fanwei))
        axtemp.set(ylim=(y_min + fanwei1, y_max - fanwei1))
        #print('up')
    elif event.button == 'down':
        axtemp.set(xlim=(x_min - fanwei, x_max + fanwei))
        axtemp.set(ylim=(y_min - fanwei1, y_max + fanwei1))
        #print('down')
    fig.canvas.draw_idle()  
fig.canvas.mpl_connect('scroll_event', call_back)
fig.canvas.mpl_connect('button_press_event', call_back)
# load data from file
# you can replace this using with open
data1 = np.loadtxt("data_odom_trajectory.txt")
#data1 = np.loadtxt("/home/pj/SLAM/ORB_SLAM2/CameraTrajectory.txt")
#data1 = np.loadtxt("/home/pj/other/PLSVIO/test/ROS/PLSVIO/euroc_plvio.txt")
first_2000 = data1[:, 0]
second_2000 = data1[:, 1]
#third_2000 = data1[:, 3]
#data2 = np.loadtxt("data.txt")
#data2 = np.loadtxt("/home/pj/other/ORB_SLAM2_Enhanced/KeyFrameTrajectory1.txt")
data2 = np.loadtxt("data_scan_trajectory.txt")
first_1000 = data2[:, 0]
second_1000 = data2[:, 1]
#third_1000 = data2[:,3]
# print to check data
#print first_2000
#print second_2000
#print third_2000

# new a figure and set it into 3d
#fig = plt.figure()
#ax = fig.gca(projection='3d')

# set figure information
#ax.set_title("3D_Curve")
#ax.set_xlabel("x")
#ax.set_ylabel("y")
#ax.set_zlabel("z")

# draw the figure, the color is r = read
plt.plot(first_2000, second_2000, c='r')
plt.plot(first_1000, second_1000, c='b')
plt.show()

