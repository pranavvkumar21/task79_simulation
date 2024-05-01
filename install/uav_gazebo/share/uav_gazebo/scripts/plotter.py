#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import os

def read_csv(filename):
    positions = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)  # skip header
        for row in csvreader:
            positions.append([float(val) for val in row])
    return positions

def plot_xy_positions(ax, x_positions, y_positions, title):
    ax.plot(x_positions, y_positions, 'o', markersize=2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    ax.grid(True)

def main():
    os.chdir("./plot_data")
    a = os.listdir()
    a.sort()
    print(a)
    os.chdir(a[-1])
    tracked_positions = read_csv('tracked_pose.csv')
    slam_out_positions = read_csv('orb_slam_pose.csv')

    tracked_x = [pose[0] for pose in tracked_positions]
    tracked_y = [pose[1] for pose in tracked_positions]

    slam_x = [pose[0] for pose in slam_out_positions]
    slam_y = [pose[1] for pose in slam_out_positions]

    fig, axs = plt.subplots(1, 2, figsize=(12, 6))

    plot_xy_positions(axs[0], tracked_x, tracked_y, 'Tracked Pose (X vs Y)')
    plot_xy_positions(axs[1], slam_x, slam_y, 'SLAM Output (X vs Y)')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()

