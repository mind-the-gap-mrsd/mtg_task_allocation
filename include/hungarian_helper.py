from scipy.optimize import linear_sum_assignment #hungarian algorithm
#from hungarian_helper import initialise_random_map, plot_init, plot_tasks
from scipy.spatial import distance
import numpy as np
import matplotlib.pyplot as plt
import random


def hungarian(cost_matrix):
    #cost_matrix = distance.cdist(robot_pose, goal_pose)
    return linear_sum_assignment(cost_matrix)

def initialise_random_map(length, breadth, gap, gap_width):
    robot_pose = np.concatenate((np.random.randint(low = 0, high = length - gap, size = (5, 1)), np.random.randint(low = 0, high = breadth, size = (5,1))), axis = 1)
    goal_poses = np.concatenate((np.random.randint(low = 0, high = length, size = (10,1)), np.random.randint(low = 0, high = breadth, size = (10,1))), axis = 1)
    #goal_poses_fin = goal_poses[np.where(not((goal_poses[:,0].any() >= gap) and (goal_poses[:,0].any()<=gap+ gap_width))), :]
    #goal_poses_fin = np.where(!((goal_poses>= gap and goal_poses<=)))
    goal_poses_fin = goal_poses
    for i in range(0, len(goal_poses)): #eliminate goals in gap
        if goal_poses[i, 0] >= gap and goal_poses[i,0]<=gap+ gap_width:
            #goal_poses.pop(goal_poses[i,0])
            goal_poses_fin = np.delete(goal_poses, i, 0)
    return robot_pose, goal_poses_fin

def plot_init(length, breadth, gap, gap_width, robot_pose, goal_pose):
    fig, ax = plt.subplots()
    ax.plot(robot_pose[:, 0], robot_pose[:,1], 'ro')
    ax.plot(goal_pose[:, 0], goal_pose[:, 1], 'bx')
    ax.set(xlim = (0, length), ylim = (0, breadth))
    ax.fill_betweenx(np.arange(0, breadth+1), gap, gap + gap_width)
    plt.show()
    

def plot_tasks(length, breadth, gap, gap_width, ta, robot_pose, goal_pose, goals_left):
    #robot_pose, goal_pose = initialise_random_map(length, breadth, gap, gap_width)
    #plt.axis(0, length, 0, breadth)
    fig, ax = plt.subplots()
    ax.plot(robot_pose[:, 0], robot_pose[:,1], 'ro')
    ax.plot(goal_pose[:, 0], goal_pose[:, 1], 'bx')
    ax.set(xlim = (0, length), ylim = (0, breadth))
    ax.fill_betweenx(np.arange(0, breadth+1), gap, gap + gap_width)
    for i in range (len(ta[1])):
        print(i)
        r = ta[0][i]
        g = ta[1][i]
        ax.plot([robot_pose[r,0], goals_left[g,0]], [robot_pose[r,1], goals_left[g,1]], 'g', linestyle = '--')
    plt.show()
    
def plot_map(length, breadth, gap, gap_width, ta, robot_pose, goal_pose, goals_left):
    #robot_pose, goal_pose = initialise_random_map(length, breadth, gap, gap_width)
    #plt.axis(0, length, 0, breadth)
    fig, ax = plt.subplots()
    ax.plot(robot_pose[:, 0], robot_pose[:,1], 'ro')
    ax.plot(goal_pose[:, 0], goal_pose[:, 1], 'bx')
    ax.set(xlim = (0, length), ylim = (0, breadth))
    ax.fill_betweenx(np.arange(0, breadth+1), gap, gap + gap_width)
    plt.show()
    
    fig, ax = plt.subplots()
    ax.plot(robot_pose[:, 0], robot_pose[:,1], 'ro')
    ax.plot(goal_pose[:, 0], goal_pose[:, 1], 'bx')
    ax.set(xlim = (0, length), ylim = (0, breadth))
    ax.fill_betweenx(np.arange(0, breadth+1), gap, gap + gap_width)
    for i in range (len(ta[1])):

        r = ta[0][i]
        g = ta[1][i]
        ax.plot([robot_pose[r,0], goals_left[g,0]], [robot_pose[r,1], goals_left[g,1]], 'g', linestyle = '--')
    plt.show()
    