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
    # print("Original goals array: ", goals)

    ### DUMMY AGENT AND GOAL LOCATIONS ###
    # agents = np.asarray([[1,3], [3, 5], [2,3], [1,1]])
    # goals = np.asarray([[1,2],[3,4],[2,4],[3,5], [50,10],[60,20],[55,15]])
    
    
    # print(nodes)
    # costmap = Costmap()
    ta, reordered_goals, goal_map_index, crossing_agents, crossing_pairing, costmap = assign_all_tasks(agents, goals)
    nodes = np.concatenate((agents, reordered_goals), axis = 0)
    # print(ta)
    # print("Goal map index: ", goal_map_index)
    #Format and send out response
    taMsg = ta_outResponse()

    for i in range(len(ta)):
        agent = agent_route()
        agent.agent_id = i
        agent.goal_list = []
        for j in range(len(ta[i])):
            task_idx = ta[i][j]
            task_object = task()
            if task_idx != 100 and task_idx != 101:
                task_object.x, task_object.y = nodes[task_idx-1,:].tolist()
                if j>0:
                    task_object.index = goal_map_index[task_idx - len(agents) - 1] + 1
                #How to
                else:
                    task_object.index = 0 #Starting task
            elif task_idx == 100:
                task_object.x = crossing_pairing[i][0][0]
                task_object.y = crossing_pairing[i][0][1]
                task_object.index = 100
            elif task_idx == 101:
                task_object.x = crossing_pairing[i][1][0]
                task_object.y = crossing_pairing[i][1][1]
                task_object.index = 101
            agent.goal_list.append(task_object)
        taMsg.agent_routes.append(agent)
    #ta_output.agent_numbers =
    
    taMsg.gap_centroid_x = costmap.centroid_world_coords[0,0]
    taMsg.gap_centroid_y = costmap.centroid_world_coords[0,1] 
    # print("Final message: ", taMsg) 
    ta_out_pub = rospy.Publisher('ta_output', ta_outResponse, queue_size=1)
    ta_out_pub.publish(taMsg)
    #ta_output.agent_start = agents
    # TODO: Add gap orientation to output message -> radians of yaw in map frame
    return taMsg

if __name__ == '__main__':
    rospy.init_node("ta_mtsp_node")
    rate = rospy.Rate(10)
    #while not rospy.is_shutdown():

    #Client
    # req = "ta"
    # rospy.wait_for_service("ta_input")
    # mission_start_srv = rospy.ServiceProxy("/ta_input", ta_input)
    # resp = mission_start_srv(req)
    #print(resp)
    #rospy.spin()

    rospy.init_node("ta_mtsp_node")
    print("Initiated TA Node")
    #rospy.Subscriber("/goal_points_set", bool, callback)
    s = rospy.Service("/ta_out", ta_out, handle_ta_out)
    ta_out_pub = rospy.Publisher('ta_output', ta_outResponse, queue_size=1)

    rospy.spin()
    