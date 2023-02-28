#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from ta_node_pkg.srv import *
from scipy.optimize import linear_sum_assignment #hungarian algorithm
from scipy.spatial import distance
import numpy as np
import matplotlib.pyplot as plt
import random

def hungarian(agents, goals):
    n_agents = len(agents)
    n_goals = len(goals)
    #cost_matrix = np.zeros(len(agents), len(goals))
    agent_pose = np.zeros((n_agents, 2))
    goal_pose = np.zeros((n_goals, 2))
    for i in range (0, n_agents):
        agent_pose[i, :] = np.asarray([agents[i].position.x, agents[i].position.y])
    for i in range (0, n_goals):
        goal_pose[i, :] = np.asarray([goals[i].position.x, goals[i].position.y])
        #cost_matrix[i, j] = distance.cdist(agents
    cost_matrix = distance.cdist(agent_pose, goal_pose)        
    return linear_sum_assignment(cost_matrix)

def handle_ta_out(req):
    req = "ta"
    rospy.wait_for_service("ta_input")
    mission_start_srv = rospy.ServiceProxy("/ta_input", ta_input)
    start_info = mission_start_srv(req)
    print(resp)
    hungarian_out = hungarian(start_info.agent_locations, start_info.agent_goal_locations)
    agent_locations = np.concatenate((np.random.randint(low = 0, high = 7, size = (4, 1)), np.random.randint(low = 0, high = 7, size = (4,1))), axis = 1)
    goal_locations = np.concatenate((np.random.randint(low = 0, high = 7, size = (4,1)), np.random.randint(low = 0, high = 7, size = (4,1))), axis = 1)
    hungarian_out = hungarian(agent_locations, goal_locations)
    agents = agent_locations
    ta_output = ta_outResponse()
    ta_output.agent_numbers =  hungarian_out[0]
    ta_output.agent_goals = hungarian_out[1]
    ta_output.agent_start = agents
    return ta_output

if __name__ == '__main__':
    rospy.init_node("ta_node")
    rate = rospy.Rate(10)
    #while not rospy.is_shutdown():

    #Client
    req = "ta"
    rospy.wait_for_service("ta_input")
    mission_start_srv = rospy.ServiceProxy("/ta_input", ta_input)
    resp = mission_start_srv(req)
    print(resp)
    rospy.spin()

    #Hungarian
    agents = resp.agent_locations
    goals = resp.agent_goal_locations
    global hungarian_out 
    hungarian_out = hungarian(resp.agent_locations, resp.agent_goal_locations)
    #hungarian_out = hungarian(agent_locations, ragent_goal_locations)
    #Service Server
    rospy.init_node("ta_node")
    s = rospy.Service("/ta_out", ta_out, handle_ta_out)

    rospy.spin()


    



# gapx = 10
# gap_length = 2
# l = 20
# b = 10
# robot_pose, goal_pose = initialise_random_map(20, 10, gapx, 2)
# robot_pose = np.asarray([[1,1], [1,1], [1,1], [1,1]])
# #goal_pose = np.asarray([[1,2], [3,2], [6,3], [7,4], [5,8], [9,2], [1,3], [12, 10], [14,3], [19,8]])
# plot_init(20, 10, gapx, gap_length, robot_pose, goal_pose)
# goals_lhs = goal_pose[np.where(goal_pose[:,0]<=gapx)[0], :] #make sure goals are on one side of the gap
# goals_rhs = goal_pose[np.where(goal_pose[:,0]>=gapx+gap_length)[0], :]
# #allocate POIs for each robot, even when n(POIs)>n(robots)
# while goals_lhs.size>0:
#     ta = hungarian(robot_pose, goals_lhs)
#     plot_tasks(20, 10, 10, 2, ta, robot_pose, goal_pose, goals_lhs)
#     #call planner here
#     robot_pose[ta[0], :] = goals_lhs[ta[1], :] #update robot pose, assuming task is complete
#     goals_lhs = np.delete(goals_lhs, ta[1], 0) #remove already reached goals
'''
# Gap crossing task assignment
# 1. finds the optimal coupling point by searching across the gap to find a location closest to all robots
# 2. calls on hungarian for 3 goal locations at coupling point

# '''
# #crossing_pts =np.hstack((np.vstack((gapx*np.ones_like(robot_pose[:,1]).T, robot_pose[:,1].T)), np.asarray([[gapx, b/2]]).T)).T
# crossing_pts = np.vstack((gapx*np.ones(10,), np.linspace(0, b, num = 10))).T
# crossing_pt = crossing_pts[np.argmin(np.sum(distance.cdist(robot_pose, crossing_pts), axis = 0)), :]
# gap_goal = crossing_pt*np.ones((3,2))
# ta_gap = hungarian(robot_pose, gap_goal)
# plot_tasks(20, 10, gapx, gap_length, ta_gap, robot_pose, goal_pose, gap_goal)
# #call planner here
# '''
# Goal Assignment after gap crossing
# 1. Assuming 2/3 robots make it past the gap, edit number of robots if needed
# '''

# #first, initialise robots to position on the other side of the gap
# robot_pose = (crossing_pt + np.asarray([gap_length, 0]))*np.ones((2,2))
# while goals_rhs.size>0:
#     ta = hungarian(robot_pose, goals_rhs)
#     plot_tasks(20, 10, 10, 2, ta, robot_pose, goal_pose, goals_rhs)
#     #call planner here
#     robot_pose[ta[0], :] = goals_rhs[ta[1], :] #update robot pose, assuming task is complete
#     goals_rhs = np.delete(goals_rhs, ta[1], 0) #remove already reached goals
    
