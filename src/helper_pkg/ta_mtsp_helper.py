#!/usr/bin/python3
import rospy
from scipy.spatial import distance
import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import cv2
#Note - add get_gap_points to allocate gap crossing task!
class Costmap():
    ''' Initiates a costmap class with information about gap location. Splits and stores the tasks based on location wrt gap.'''
    def __init__(self, agents, goals):
        self.map_file = "/home/sandy/Desktop/VSCode/puzzlebots_ws/src/mtg_task_allocation/map/map_gap.png" #rospy.get_param(map_file)
        self.get_gap_coords() 
        self.agents = agents
        self.goals = goals
        #self.find_vertices()
        self.split_tasks()


    def get_gap_coords(self):
        #read map
        self.map = cv2.imread(self.map_file)
        plt.imshow(self.map)
        plt.show()
        #find gap points where pixel value is 127
        binary_map = np.where(self.map==127, self.map, 0) #Set all non gap points to 0
        binary_map = cv2.cvtColor(binary_map, cv2.COLOR_RGB2GRAY)

        ret, thresh = cv2.threshold(binary_map, 126, 255, 0)
        
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #ASSUMING THIS ALWAYS GIVES US 4 POINTS FOR A QUADRILATERAL
        self.gap_contour = contours[0]

    def split_tasks(self):
        #Check where the task lies with respect to the gap
        tasks_lhs = []
        tasks_rhs = []
        # Ensure that the gap_contour is closed by adding the first point to the end
        print(self.gap_contour)
        gap_contour_closed = np.concatenate((self.gap_contour, np.array([self.gap_contour[0]])), axis = 0)
        self.gap_contour = gap_contour_closed
        print(len(gap_contour_closed))
        # Iterate through each pair of consecutive points in the contour
        for task in self.goals:
            print(task)
            flag = 0
            for i in range(len(gap_contour_closed) - 1):
                #TO-DO: Discard points within gaps
                A = gap_contour_closed[i][0]
                B = gap_contour_closed[i + 1][0]
                # Calculate vectors AB and AP
                vector_ab = (B[0] - A[0], B[1] - A[1])
                vector_ap = (task[0] - A[0], task[1] - A[1])

                # Calculate the cross product
                cross_product = vector_ab[0] * vector_ap[1] - vector_ab[1] * vector_ap[0]

                # Determine the point's position
                if cross_product > 0:
                    print("lhs", task)
                    tasks_lhs.append(task)
                    #flag = 1
                    break
                elif cross_product < 0:
                    print("rhs", task)
                    tasks_rhs.append(task)
                    #flag = 1
                    break
        self.tasks_lhs = tasks_lhs
        self.tasks_rhs = tasks_rhs
    
    def find_vertices(self):
        vertices = self.gap_contour[:, 0, :]
        print(vertices)
        top_left = vertices[np.argmin(vertices, axis=1)]
        bottom_left = vertices[np.argmin(vertices, axis=1, keepdims=True).T[0]]
        top_right = vertices[np.argmax(vertices, axis=1)]
        bottom_right = vertices[np.argmax(vertices, axis=1, keepdims=True).T[0]]
        self.gap_contour = np.concatenate((top_left, bottom_left, top_right, bottom_right))
    
    #TO-DO: Locate along which axis the gap lies so can handle gaps parallel to x as well. 
    #Reorder vertices accordingly so vertices[1]-vertices[2] is the first edge and 3 to 4 is second edge 
    #Consider how to add gap width and see where the agents reach        



def geometry_msgs_to_array(agents, goals): #Converts geometry messages to array for mtsp solver
    n_agents = len(agents)
    n_goals = len(goals)
    rospy.set_param("n_agents", n_agents)
    rospy.set_param("n_goals", n_goals)
    #cost_matrix = np.zeros(len(agents), len(goals))
    agent_pose = np.zeros((n_agents, 2))
    goal_pose = np.zeros((n_goals, 2))
    for i in range (0, n_agents):
        agent_pose[i, :] = np.asarray([float(agents[i].position.x), float(agents[i].position.y)])
    for i in range (0, n_goals):
        goal_pose[i, :] = np.asarray([float(goals[i].position.x), float(goals[i].position.y)])
    #Format for  Google OR tools
    #nodes = np.concatenate((agent_pose, goal_pose), axis = 0)
    #print("nodes as geometry", nodes)
    return agents, goals

def img_to_map(nodes): #Makes sure everything is in map coordinates
    for i in nodes:
        i[0] = int(i[0]/0.05)
        i[1] = int(210 - int(i[1]/0.05))
    #print("converted nodes: ", nodes)
    return nodes

def get_distance_matrix(agents, goals):
    #Set up problem, convert the mission_start values to arrays
    nodes = np.concatenate((agents, goals), axis = 0)
    #Set up dummy end location
    nodes = np.concatenate((np.asarray([[1,1]]), nodes), axis = 0) 
    n_agents = len(agents)
    #Calculate cost matrix
    cost_mat = np.zeros((len(nodes),len(nodes)))
    #nodes = img_to_map(nodes)
    print("nodes in mapp arr", nodes)
    #cost_mat = distance.cdist(nodes, nodes, "euclidean")
    path_to_file = "/home/sandy/Desktop/VSCode/puzzlebots_ws/src/mtg_task_allocation/src/helper_pkg/distances_final_5.npz" #rospy.get_param(distances_file) #"/home/dsreeni/Sem2/mtg_codebase/mtg_ws/src/mtg_task_allocation/src/helper_pkg/distances.npz" #Make this a rosparam in some launch file
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

def format_solution(data, manager, routing, solution, goals):
    '''Formats MTSP solution so that it is readable by the planner'''
    #print(f'Objective: {solution.ObjectiveValue()}'
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

def get_gap_point(ta, route_lengths, costmap):
    print("Getting gap points")
    goals = costmap.goals
    agents = costmap.agents
    contour = costmap.gap_contour
    print(costmap.gap_contour[0, 0] )
    #costmap.find_vertices
    edgeLHS = np.linspace(costmap.gap_contour[0, 0], costmap.gap_contour[1, 0], 10)
    edgeRHS = np.linspace(costmap.gap_contour[2, 0], costmap.gap_contour[3, 0], 10)
    print(contour)
    robot_pose = np.zeros((len(ta), 2))
    for task_list_index in range (len(ta)):
        robot_pose[task_list_index,:] = goals[ta[task_list_index][-1]-1, :] #robot pose after it has reached points of interest / pose of final task in robots task list
    
    cost_matrix_unsorted = distance.cdist(edgeLHS, robot_pose)
    #print(cost_matrix_unsorted)
    for i in range (len(ta)):
        cost_matrix_unsorted[:, i] += route_lengths[i] #Include agent route length in metric for cost to crossing point
    #print(cost_matrix_unsorted)
    cost_matrix_sorted = np.sort(cost_matrix_unsorted)
    cost_matrix_sorted = cost_matrix_sorted[:, :3] #Discard final column
    crossing_point_idx = np.argmin(np.sum(cost_matrix_sorted, axis  = 1))
    crossing_point = edgeLHS[crossing_point_idx]
    # crossing_point = np.asarray([c, y[crossing_point_idx]])
    crossing_agents = np.argsort(cost_matrix_unsorted[crossing_point_idx, :])[:3]
    for agent in crossing_agents: #list of agent indices
        ta[agent].append(100)
        route_lengths[agent] = cost_matrix_unsorted[crossing_point_idx, agent]
    print(ta)
    print(route_lengths)
    return ta, crossing_point, crossing_agents


def solve_mtsp(agents, goals):
    '''MTSP Solver'''
    print("Solving MTSP")
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
    distance_dimension.SetGlobalSpanCostCoefficient(500)
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
    return ta, route_lengths

def assign_all_tasks(agents, goals):
    #agents, goals = geometry_msgs_to_array(agents, goals)
    costmap = Costmap(agents, goals)
    print(costmap)
    tasksLHS = costmap.tasks_lhs
    taLHS, route_lengths = solve_mtsp(agents, tasksLHS)
    taGap, crossing_point, crossing_agents = get_gap_point(taLHS, route_lengths, costmap)
    crossing_agent_pose = np.zeros((len(crossing_agents),2))
    for num in range(len(crossing_agents)):
        crossing_agent_pose[num] = crossing_point 
    print(crossing_agent_pose)
    taRHS, __ = solve_mtsp(crossing_agent_pose, costmap.tasks_rhs)
    #Format RHS tasks to be readable by mapping to actual robot index
    for taskListIdx in range(len(taRHS)):
        for task in taRHS[taskListIdx]:
            agentIdx = crossing_agents[taskListIdx] #crossing agents stores original agent indices
            #For example, if agents 2,3,4 are assigned to gap crossing, agent 2 would be the 0th entry in taRHS
            taGap[agentIdx].append(task)
    print(taGap)

if __name__=="__main__":
    agents = np.asarray([[1,3], [3, 5], [2,3], [1,1]])
    goals = np.asarray([[1,2],[3,4],[2,4],[3,5], [42,38],[55,35],[73,15]])
    costmap = Costmap(agents, goals)
    costmap.split_tasks()
    assign_all_tasks(agents, goals)
    #How to loop through? First, tasks LHS