#!/usr/bin/python3
import rospy
from scipy.spatial import distance
import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
#Note - add get_gap_points to allocate gap crossing task!
def geometry_msgs_to_array(agents, goals):
    n_agents = len(agents)
    n_goals = len(goals)
    #cost_matrix = np.zeros(len(agents), len(goals))
    agent_pose = np.zeros((n_agents, 2))
    goal_pose = np.zeros((n_goals, 2))
    for i in range (0, n_agents):
        agent_pose[i, :] = np.asarray([float(agents[i].position.x), float(agents[i].position.y)])
    for i in range (0, n_goals):
        goal_pose[i, :] = np.asarray([float(goals[i].position.x), float(goals[i].position.y)])
    #Format for  Google OR tools
    nodes = np.concatenate((agent_pose, goal_pose), axis = 0)
    #print("nodes as geometry", nodes)
    return nodes

def nodes_to_map_array(nodes):
    for i in nodes:
        i[0] = int(i[0]/0.05)
        i[1] = int(210 - int(i[1]/0.05))
    #print("converted nodes: ", nodes)
    return nodes

def get_distance_matrix(agents, goals):
    #Set up problem, convert the mission_start values to arrays
    nodes = geometry_msgs_to_array(agents, goals)
    #print(goals)
    # nodes = np.concatenate((agents, goals), axis = 0)
    #print(nodes)
    #Set up dummy end location
    nodes = np.concatenate((np.asarray([[1,1]]), nodes), axis = 0) 
    n_agents = len(agents)
    #Calculate cost matrix
    cost_mat = np.zeros((len(nodes),len(nodes)))
    nodes = nodes_to_map_array(nodes)
    #cost_mat = distance.cdist(nodes, nodes, "euclidean")
    path_to_file = "path_to_distancez.npz"
    distances = np.load(path_to_file)["distances"]
    for i in range (len(nodes)):
        for j in range (len(nodes)):
            #arg = np.concatenate((i, j)).tolist()
            #rint(int(nodes[i,0]), int(nodes[i,1]), int(nodes[j,0]), int(nodes[j,1]), "Nodes")
            cost_mat[i, j] = distances[int(nodes[i,0]), int(nodes[i,1]), int(nodes[j,0]), int(nodes[j,1])]
    #Modify cost matrix such that the distance from the goal as index 0 is equidistant (i.e. does not affect cost)
    cost_mat[0, :] = np.zeros_like(cost_mat[0,:])
    np.fill_diagonal(cost_mat, 0)
    cost_mat[:,0] = np.zeros_like(cost_mat[:,0])
    print(cost_mat)
    return cost_mat.tolist(), n_agents

def create_data_model(agents, goals): 
    data = {}
    distance_matrix, n_agents = get_distance_matrix(agents, goals)
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = n_agents
    data['starts'] = [] #start locations
    data['ends'] = []
    #Start locations indeces 1 to N_agents
    for i in range(n_agents):
        data['starts'].append(i+1)
        data['ends'].append(0)
    return data

''' Format MTSP solver's solution to be read by planner'''

def format_solution(data, manager, routing, solution, goals):
    #print(f'Objective: {solution.ObjectiveValue()}')
    max_route_distance = 0
    ta = []
    route_lengths = []
    for agent_id in range(data['num_vehicles']):
        task_list = []
        index = routing.Start(agent_id)
        plan_output = 'Route for vehicle {}:\n'.format(agent_id)
        route_distance = 0
        while not routing.IsEnd(index):
            task_list.append(manager.IndexToNode(index))
            #plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, agent_id)
            #print(route_distance, "route distance")
        #print(plan_output)
        ta.append(task_list)
        route_lengths.append(route_distance)   
        #print(route_lengths)
        max_route_distance = max(route_distance, max_route_distance)
    #ta, route_lengths = get_gap_point(ta, route_lengths, goals)
    return ta, route_lengths #, crossing_point

def get_gap_point(ta, route_lengths, goals):
    #Assume that there is a gap along x = 7
    y = np.linspace(0, 7, 10)
    gap_locs = np.vstack(( 7.*np.ones_like(y), y)).T
    end_pt = np.zeros((len(ta), 2))
    
    for task_list in range (len(ta)):
        end_pt[task_list,:] = goals[ta[task_list][-1], :]
    
    cost_matrix_unsorted = distance.cdist(gap_locs, end_pt)
    #print(cost_matrix_unsorted)
    for i in range (len(ta)):
        cost_matrix_unsorted[:, i] += route_lengths[i]
    #print(cost_matrix_unsorted)
    cost_matrix_sorted = np.sort(cost_matrix_unsorted)
    cost_matrix_sorted = cost_matrix_sorted[:, :3]
    crossing_point_idx = np.argmin(np.sum(cost_matrix_sorted, axis  = 1))
    crossing_point = np.asarray([7, y[crossing_point_idx]])
    crossing_agents = np.argsort(cost_matrix_unsorted[crossing_point_idx, :])[:3]


    for a in crossing_agents:
        ta[a].append(100)
        route_lengths[a] = cost_matrix_unsorted[crossing_point_idx, a]
    return ta, route_lengths, crossing_point

'''MTSP Solver'''
def solve_mtsp(agents, goals):
    data = create_data_model(agents, goals)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['starts'], data['ends'])
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        #nodes = nodes_to_map_array([from_node, to_node])
        #print("Callback", data['distance_matrix'][from_node][to_node])
        return data['distance_matrix'][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        100,  # vehicle maximum travel distance
        True,  # True starts cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    #xyz = int(input("Enter coefficient for distance dimension: "))
    #print("Coefficient for distance dimension", xyz)
    distance_dimension.SetGlobalSpanCostCoefficient(1000)
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
    search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 5
    search_parameters.log_search = False
    #Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    
    ta, route_lengths = format_solution(data, manager, routing, solution, goals)
    print(route_lengths)
    print(ta)
    print(len(ta))
    return ta
