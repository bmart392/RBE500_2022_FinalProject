#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def visualization():
    # load csv file and plot trajectory

    trajectory = np.loadtxt("trajectory.csv", delimiter=',')
    """     plt.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], linewidth=2)
    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.minorticks_on()
    plt.grid(which='both')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.show() """

    #fig = plt.figure()
    ax = plt.axes(projection='3d')

    # Data for a three-dimensional line
    zline = np.linspace(-10, 10, 50)
    xline = np.linspace(-10, 10, 50)
    yline = np.linspace(-10, 10, 50)
    ax.plot3D(xline, yline, zline, 'gray')

    # Data for three-dimensional scattered points
    xdata = trajectory[:, 0]
    ydata = trajectory[:, 1]
    zdata = trajectory[:, 2]
    ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')

    plt.show()
    

if __name__ == '__main__':
    visualization()
