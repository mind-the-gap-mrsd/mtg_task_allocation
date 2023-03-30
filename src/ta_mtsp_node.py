#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool
from mtg_mtsp_task_allocator.srv import *
from mtg_mtsp_task_allocator.msg import *
from scipy.optimize import linear_sum_assignment #hungarian algorithm
from scipy.spatial import distance
import numpy as np
import matplotlib.pyplot as plt
import random
#from puzzlebots_ws.src.mtg_mtsp_task_allocator.src.
from helper_pkg.ta_mtsp_helper import solve_mtsp, geometry_msgs_to_array

#########
# def callback(flag):
#     if flag == 1:
#         s = rospy.Service("/ta_out", ta_out, handle_ta_out)
#########
def handle_ta_out(req):
    print("Handling Service")
    req = 'ta'
    # rospy.wait_for_service("ta_input")
    # mission_start_srv = rospy.ServiceProxy("/ta_input", ta_input)
    # start_info = mission_start_srv(req)
    # print(resp)
    #agents = #start_info.agent_locations
    #goals = #start_info.agent_goal_locations
    
    #total_task_list = agents + goals
    #cost_matrix = np.zeros(len(agents), len(goals))
    #nodes = geometry_msgs_to_array(agents, goals)


    ###############################################
    ### DUMMY AGENT AND GOAL LOCATIONS ###
    agents = np.asarray([[1,3], [3, 5], [2,3], [1,1]])
    goals = np.asarray([[1,2],[3,4],[2,4],[3,5], [2,3],[4,5],[3,5]])
    nodes = np.concatenate((agents, goals), axis = 0)
    print(nodes)
    ##########################################

    allocated_tasks = solve_mtsp(agents, goals)
    print(allocated_tasks)
    ta = ta_outResponse()
    #ta_output.agent_goals = agent_tasks_lists
    for i in range(len(allocated_tasks)):
        agent = mtg_mtsp_task_allocator.msg.agent_route()
        agent.agent_id = i 
        agent.goal_list = []
        for j in range(len(allocated_tasks[i])):
            task_idx = allocated_tasks[i][j]
            task = mtg_mtsp_task_allocator.msg.task()
            #ta.single_agent_route.goal_list.append([task])
            task.position = nodes[task_idx-1,:].tolist()
            
            if j>0:
                task.index = task_idx - len(agents)
            else:
                task.index = 0 #Starting task
            agent.goal_list.append(task)
        ta.single_agent_route.append(agent)
    #ta_output.agent_numbers =  
    print(ta)
    #ta_output.agent_start = agents
    return ta

if __name__ == '__main__':
    rospy.init_node("ta_mtsp_node")
    rate = rospy.Rate(10)
    #while not rospy.is_shutdown():

    #Client
    #req = "ta"
    #rospy.wait_for_service("ta_input")
    #mission_start_srv = rospy.ServiceProxy("/ta_input", ta_input)
    #resp = mission_start_srv(req)
    #print(resp)
    #rospy.spin()

    rospy.init_node("ta_mtsp_node")
    print("Initiated TA Node")
    #rospy.Subscriber("/goal_points_set", bool, callback)
    s = rospy.Service("/ta_out", ta_out, handle_ta_out)

    rospy.spin()
    