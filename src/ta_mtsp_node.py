#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool
from mtg_messages.srv import ta_outResponse, ta_out, task_allocator_input as ta_input
from mtg_messages.msg import agent_route, task
import numpy as np
import matplotlib.pyplot as plt
import random
#from puzzlebots_ws.src.mtg_mtsp_task_allocator.src.
from helper_pkg.ta_mtsp_helper import solve_mtsp, geometry_msgs_to_array, Costmap, get_gap_point, assign_all_tasks

def handle_ta_out(req):
    print("Handling Service")
    req = 'ta'
    rospy.wait_for_service("ta_input")
    mission_start_srv = rospy.ServiceProxy("/ta_input", ta_input)
    start_info = mission_start_srv(req)
    # print(resp)
    agents = start_info.agent_locations
    goals = start_info.agent_goal_locations
    
    #total_task_list = agents + goals
    # cost_matrix = np.zeros(len(agents), len(goals))
    agents, goals = geometry_msgs_to_array(agents, goals)
    nodes = np.concatenate((agents, goals), axis = 0)
    ### DUMMY AGENT AND GOAL LOCATIONS ###
    # agents = np.asarray([[1,3], [3, 5], [2,3], [1,1]])
    # goals = np.asarray([[1,2],[3,4],[2,4],[3,5], [2,3],[4,5],[3,5]])
    # nodes = np.concatenate((agents, goals), axis = 0)
    # print(nodes)
    #costmap = Costmap()
    ta = assign_all_tasks(agents, goals)
    #Format and send out response
    taMsg = ta_outResponse()

    for i in range(len(ta)):
        agent = agent_route()
        agent.agent_id = i
        agent.goal_list = []
        for j in range(len(ta)):
            task_idx = ta[i][j]
            task = task()
            task.x, task.y = nodes[task_idx-1,:].tolist()
            if j>0:
                task.index = task_idx - len(agents)
            #How to
            else:
                task.index = 0 #Starting task
            agent.goal_list.append(task_object)
        taMsg.agent_routes.append(agent)
    #ta_output.agent_numbers =  
    print(taMsg)
    ta_out_pub = rospy.Publisher('ta_output', ta_outResponse, queue_size=1)
    ta_out_pub.publish(taMsg)
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
    ta_out_pub = rospy.Publisher('ta_output', ta_outResponse, queue_size=1)

    rospy.spin()
    