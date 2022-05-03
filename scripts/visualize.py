#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

def visualization():
    # load csv file and plot trajectory
    trajectory = np.loadtxt("trajectory.csv", delimiter=',')
    
    plot_limits = [[-3.14, 3.14], [-3.14, 3.14], [-.1, .35]]
    plot_y_labels = ["Position (radians)", "Position (radians)", "Position (meters)"]

    figure, axis = plt.subplots(3,1)

    figure.suptitle("Joint Positions through Trajectory")

    for index in range(3):
        axis[index].plot(trajectory[:,index], linewidth=2)
        plt.ylim(plot_limits[index][0], plot_limits[index][1])
        axis[index].minorticks_on()
        axis[index].grid(which='both')
        axis[index].set_xlabel('Time')
        axis[index].set_ylabel(plot_y_labels[index])
    plt.show()


if __name__ == '__main__':
    visualization()
