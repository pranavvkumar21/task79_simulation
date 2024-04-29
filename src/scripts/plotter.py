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

def plot_xy_positions(ax, x_positions, y_positions, title,label="cartographer"):
    ax.plot(x_positions, y_positions, 'o', markersize=2, label=label)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    ax.grid(True)
    ax.legend()

def main():
    os.chdir("./plot_data")
    a = os.listdir()
    a.sort()
    print(a)
    os.chdir(a[-1])
    tracked_positions = read_csv('tracked_pose.csv')
    slam_out_positions = read_csv('dr_pose.csv')
    ground_truth = read_csv('odometry_pose.csv')

    tracked_x = [pose[0] for pose in tracked_positions]
    tracked_y = [pose[1] for pose in tracked_positions]
    
    gt_x = [pose[0] for pose in ground_truth]
    gt_y = [pose[1] for pose in ground_truth]

    slam_x = [pose[0] for pose in slam_out_positions]
    slam_y = [pose[1] for pose in slam_out_positions]

    fig, axs = plt.subplots(1, 2, figsize=(12, 6))

    plot_xy_positions(axs[0], tracked_x, tracked_y, 'Tracked Pose (X vs Y)',"cartographer")
    plot_xy_positions(axs[0], gt_x, gt_y, 'Cartographer vs ground truth',"ground truth")
    plot_xy_positions(axs[1], slam_x, slam_y, 'SLAM Output (X vs Y)',"hector_slam")
    plot_xy_positions(axs[1], gt_x, gt_y, 'Hector_slam vs ground truth', "ground truth")
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()

