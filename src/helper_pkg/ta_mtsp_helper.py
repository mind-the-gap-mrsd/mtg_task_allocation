#!/usr/bin/python3
import rospy
from scipy.spatial import distance
import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import cv2
import copy
from typing import Union, List
from shapely import LineString, Point
from collections import OrderedDict
from scipy.optimize import linear_sum_assignment

class Costmap():
    ''' Initiates a costmap class with information about gap location. Splits and stores the tasks based on location wrt gap.'''
    def __init__(self, agents: np.ndarray, goals: np.ndarray)-> None:
        '''
        ### Inputs
            1. agents: 2D array of agent locations in world coordinates
            2. goals: 2D array of goal locations in world coordinates

        ### Returns
            1. Gap coordinates
            2. free space dictionary
            3. split tasks by LHS and RHS of gap
        '''

        self.map_file = "/home/ros_ws/src/mtg_task_allocation/map/map_with_gap_no_obs.png" #rospy.get_param(map_file)
        self.agents = agents
        self.goals = goals
        self.get_gap_coords()
        self.free_space_dict = self.find_free_space() 
        
        #self.find_vertices()
        self.split_tasks()

    def get_gap_coords(self):
        ''' Given a map with a gap, finds the coordinates of the gap and the orientation of the gap. 
        
        ### Method
        1. Read the map file and find the gap coordinates
        2. Threshold for gap pixles (value = 127)
        3. Find contours (cv2.findContours))
        4. Find rotated bounding box (cv2.minAreaRect)
        5. Find centroid and orientation of gap
        '''
        self.map = cv2.imread(self.map_file)
        
        #find gap points where pixel value is 127
        binary_map = np.where(self.map==127, self.map, 0) #Set all non gap points to 0
        self.binary_map = cv2.cvtColor(binary_map, cv2.COLOR_RGB2GRAY)

        ret, thresh = cv2.threshold(self.binary_map, 126, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #ASSUMING THIS ALWAYS GIVES US 4 POINTS FOR A QUADRILATERAL
        # contours in the form of (x,y) pixel coordinates
        self.contours_pixels = np.squeeze(contours)
        self.contour_linestring = LineString(self.contours_pixels.tolist())
        self.centroid_of_gap = list(self.contour_linestring.centroid.coords)[0]
        self.centroid_world_coords = self.pixel_to_world(np.array([self.centroid_of_gap]))
        centroid_of_agents = [0,0]
        
        # Finding agent centroid
        for agent_loc in self.agents:
            centroid_of_agents[0] += agent_loc[0]
            centroid_of_agents[1] += agent_loc[1]
        centroid_of_agents[0] /= len(self.agents)
        centroid_of_agents[1] /= len(self.agents)

        self.gap_orientation_agents = (self.centroid_of_gap[0] - centroid_of_agents[0], self.centroid_of_gap[1] - centroid_of_agents[1])
        
        # Find bounding box of gap and get orientation
        self.find_gap_orientation()
    
    def find_gap_orientation(self) -> None:
        ''' Finds the orientation of the gap. '''
        rect = cv2.minAreaRect(self.contours_pixels)
        box = cv2.boxPoints(rect)
        bounding_box = np.int0(box)
        index = np.argmin(np.linalg.norm(bounding_box - np.roll(bounding_box, 1, axis = 0), axis = 1))
        self.gap_orientation = bounding_box[index] - np.roll(bounding_box, 1, axis=0)[index]
        self.gap_orientation = self.gap_orientation.astype(np.float64) / np.linalg.norm(self.gap_orientation)
        if(self.gap_orientation.T@self.gap_orientation_agents < 0):
            self.gap_orientation *= -1
        
        # print("Obtained gap orientation: ", self.gap_orientation)
        


    def split_tasks(self):
        
        tasks_lhs = []
        tasks_rhs = []
        self.index_lhs = []
        self.index_rhs = []
        
        for iter, task in enumerate(self.goals):
            direction = (self.centroid_world_coords[0,0] - task[0])*self.gap_orientation[0] + (self.centroid_world_coords[0,1] - task[1])*self.gap_orientation[1]
            if direction > 0:
                
                tasks_lhs.append(task)
                self.index_lhs.append(iter)
            else:
                
                tasks_rhs.append(task)
                self.index_rhs.append(iter)

        self.tasks_lhs = np.array(tasks_lhs)
        self.tasks_rhs = np.array(tasks_rhs)
        self.goal_map_index = np.concatenate((self.index_lhs, self.index_rhs))
        self.goals = np.concatenate((tasks_lhs, tasks_rhs), axis = 0)
        
        
    
    def find_free_space(self, num_of_crossing_agents: int = 3) -> dict:
        ''' Finds the free space for coupling/decoupling in the map
         
          ### Method
            1. Find the gap orientation
            2. Rotate map so that gap is vertical
            3. Find the free space for coupling/decoupling by checking a sliding window
            4. Rotate points back to original orientation
            5. Return a dictionary mapping LHS to RHS points viable for crossing  '''
        # Rotate map so that gap is vertical
        height, width = self.binary_map.shape
        angle = -1*np.arctan2(self.gap_orientation[1], self.gap_orientation[0])
        self.rotmatrix= cv2.getRotationMatrix2D((width/2, height/2), (180/np.pi)*angle, 1)
        self.rotated_map = cv2.warpAffine(self.map, self.rotmatrix, (width, height), flags=cv2.INTER_NEAREST)
        self.rotmatrix = np.vstack([self.rotmatrix, np.array([0, 0, 1])])

        self.free_space_map = copy.deepcopy(self.map)
        free_space_lhs = []
        free_space_rhs = []
        free_space_dict = dict()

        for y in range(4, height-4, 1):
            for x in range(10, width-10, 1):
                window = self.map[y-4:y+4, x-10:x+10]
                original_point = self.unrotate_point(np.array([x,y]))
                if(np.any(window == 0)):
                    self.free_space_map[int(original_point[1]),int(original_point[0])] = 0
                elif(np.any(window == 127)):
                    self.free_space_map[int(original_point[1]),int(original_point[0])] = 127
                    prev_point = self.unrotate_point(np.array([x-1,y]))
                    if(self.free_space_map[int(prev_point[1]),int(prev_point[0]), 0] == 255):
                        free_space_lhs.append(prev_point)
                else:
                    self.free_space_map[int(original_point[1]),int(original_point[0])] = 255
                    prev_point = self.unrotate_point(np.array([x-1,y]))
                    if(self.free_space_map[int(prev_point[1]),int(prev_point[0]), 0] == 127):
                        free_space_rhs.append(original_point)
        plt.imshow(self.free_space_map)
        plt.show()
        for point in free_space_lhs:
            closest_rhs_point = self.find_closest_point(np.array(point), np.array(free_space_rhs))
            diff_vector = np.array(closest_rhs_point) - np.array(point)
            diff_vector /= np.linalg.norm(diff_vector)
            gap_perp = np.array([-self.gap_orientation[1], self.gap_orientation[0]])
            if(np.abs(diff_vector.T@gap_perp) > 1e-2):
                continue
            else:
                free_space_dict[tuple(point)] = closest_rhs_point
        
        return free_space_dict

    def unrotate_point(self,point: np.ndarray) -> np.ndarray:
        ''' Unrotates a point using the rotation matrix '''
        point = np.linalg.inv(self.rotmatrix)@np.array([point[0],point[1],1]).reshape(-1,1)
        point = point.flatten()
        point /= point[-1]
        return point[:2]
    
    def find_closest_point(self, point: np.ndarray, point_set: np.ndarray) -> np.ndarray:
        ''' Finds the closest point in a set of points to a given point '''
        closest_point_idx = np.argmin(np.linalg.norm(point_set - point, axis = 1))
        return point_set[closest_point_idx]

    def pixel_to_world(self, pixel_point: np.ndarray) -> np.ndarray:
        ''' Converts a pixel point to world coordinates '''
        height, _ = self.binary_map.shape
        world_points = np.hstack([0.05*pixel_point[:,0].reshape(-1,1), 0.05*(height - pixel_point[:,1].reshape(-1,1))])
        return world_points
     

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
    ##print("nodes as geometry", nodes)
    return agent_pose, goal_pose

def world_to_pixel(world_points: np.ndarray, costmap: Costmap) -> np.ndarray:
    ''' Converts a world point to pixel coordinates '''
    height, _ = costmap.binary_map.shape
    pixel_points = np.hstack([(world_points[:,0].reshape(-1,1)/0.05), (height - world_points[:,1].reshape(-1,1)/0.05)]).astype(int)
    return pixel_points

def get_distance_matrix(agents, goals, costmap: Costmap):
    # Set up problem, convert the mission_start values to arrays
    nodes = np.concatenate((agents, goals), axis = 0)
    nodes = np.concatenate((np.asarray([[1,1]]), nodes), axis = 0) 
    n_agents = len(agents)
    #Calculate cost matrix
    cost_mat = np.zeros((len(nodes),len(nodes)))
    cost_mat = distance.cdist(nodes, nodes, "euclidean")/0.05
    # path_to_file = "/home/ros_ws/src/mtg_task_allocation/src/helper_pkg/distances.npz" #rospy.get_param(distances_file) #"/home/dsreeni/Sem2/mtg_codebase/mtg_ws/src/mtg_task_allocation/src/helper_pkg/distances.npz" #Make this a rosparam in some launch file
    # distances = np.load(path_to_file)["distances"]
    # for i in range (len(nodes)):
    #     for j in range (len(nodes)):
    #         cost_mat[i, j] = distances[int(nodes[i,0]), int(nodes[i,1]), int(nodes[j,0]), int(nodes[j,1])]
    cost_mat[0, :] = np.zeros_like(cost_mat[0,:])
    np.fill_diagonal(cost_mat, 0)
    cost_mat[:,0] = np.zeros_like(cost_mat[:,0])

    return cost_mat.tolist(), n_agents

def create_data_model(agents, goals, costmap): 
    data = {}
    distance_matrix, n_agents = get_distance_matrix(agents, goals, costmap)
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
    max_route_distance = 0
    ta = []
    route_lengths = []
    for agent_id in range(data['num_vehicles']):
        task_list = []
        index = routing.Start(agent_id)
        # print("agent id: ", index)
        plan_output = 'Route for vehicle {}:\n'.format(agent_id)
        route_distance = 0
        while not routing.IsEnd(index):
            task_list.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, agent_id)

        ta.append(task_list)
        route_lengths.append(route_distance)   

        max_route_distance = max(route_distance, max_route_distance)

    return ta, route_lengths



def create_crossing_tasks(robot_poses: np.ndarray, crossing_agents: np.ndarray, crossing_point: List, opposite_point, costmap: Costmap) -> List:
    '''
    Given the crossing agents, their final poses and the crossing point, greedily assign n points
    centered about crossing point to the n crossing agents and create pseudo-goals for it
    '''
    num_of_agents = len(crossing_agents)
    shifting_directions = np.linspace(-len(crossing_agents)/2, len(crossing_agents)/2, len(crossing_agents))
    shifting_magnitude = 0.27 # TODO: change from arbitrary distance to some measured quantity
    np_gap_orientation = costmap.gap_orientation
    shifting_delta = np.multiply((shifting_directions*shifting_magnitude).reshape(-1,1), np_gap_orientation)
    np_crossing_points = shifting_delta + np.array(crossing_point)
    robot_locations = robot_poses[crossing_agents]
    dist_matrix = distance.cdist(robot_locations, np_crossing_points)
    pairing = OrderedDict()
    output_pairing_hungarian = list(zip(linear_sum_assignment(dist_matrix)))
    for pair in output_pairing_hungarian:
        pairing[crossing_agents[pair[0]]] = [np_crossing_points[pair[1]].tolist(), (opposite_point + (pair[1]-1)*shifting_magnitude*costmap.gap_orientation).tolist()]

    return pairing

def get_gap_point(ta, route_lengths, costmap):

    goals = costmap.goals

    boundary = np.array(list(costmap.contour_linestring.coords))

    robot_pose = np.zeros((len(ta), 2))
    for task_list_index in range (len(ta)):
        robot_pose[task_list_index,:] = goals[ta[task_list_index][-1]-(len(ta)+1), :] #robot pose after it has reached points of interest / pose of final task in robots task list
    
    free_space_points = costmap.pixel_to_world(np.array(list(costmap.free_space_dict.keys())))
    cost_matrix_unsorted = distance.cdist(free_space_points, robot_pose, 'euclidean')/0.05

    for i in range (len(ta)):
        cost_matrix_unsorted[:, i] += route_lengths[i] #Include agent route length in metric for cost to crossing point

    cost_matrix_sorted = np.sort(cost_matrix_unsorted)
    cost_matrix_sorted = cost_matrix_sorted[:, :3] #Discard final column
    crossing_point_idx = np.argmin(np.sum(cost_matrix_sorted, axis  = 1))
    crossing_point = free_space_points[crossing_point_idx]

    opposite_point = costmap.free_space_dict[tuple(world_to_pixel(np.array([crossing_point]), costmap).flatten().tolist())]

    opposite_point = costmap.pixel_to_world(np.array([opposite_point])).flatten()
    

    crossing_agents = np.argsort(cost_matrix_unsorted[crossing_point_idx, :])[:3]
    crossing_task_pairing = create_crossing_tasks(robot_pose, crossing_agents, crossing_point, opposite_point, costmap)
    # print("Pairing: ", crossing_task_pairing)
    for agent in crossing_agents: 
        ta[agent].append(100)
        ta[agent].append(101)
        route_lengths[agent] = cost_matrix_unsorted[crossing_point_idx, agent]
    return ta, opposite_point, crossing_agents, crossing_task_pairing


def solve_mtsp(agents, goals, costmap):
    '''MTSP Solver'''

    # print("Agents", agents)
    # print("Goals", goals)
    data = create_data_model(agents, goals, costmap)
    # print("Data: ", data)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['starts'], data['ends'])
    # print("Map:", manager.GetIndexToNodeMapping())
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)


    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        100,  # vehicle maximum travel distance
        True,  # True starts cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)

    distance_dimension.SetGlobalSpanCostCoefficient(750)
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
    search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 5
    search_parameters.log_search = False
    #Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    # print("solution", solution)
    
    ta, route_lengths = format_solution(data, manager, routing, solution, goals)
    # print("Route lengths: ", route_lengths)
    return ta, route_lengths

def assign_all_tasks(agents, goals):

    costmap = Costmap(agents, goals)

    reordered__goals = costmap.goals
    num_lhs = costmap.tasks_lhs.shape[0]
    goal_map_index = costmap.goal_map_index
    taLHS, route_lengths = solve_mtsp(agents, costmap.tasks_lhs, costmap)
    # print(taLHS)
    taGap, crossing_point, crossing_agents, crossing_pairing = get_gap_point(taLHS, route_lengths, costmap)
    crossing_agent_pose = np.zeros((len(crossing_agents),2))
    crossing_points_pose = np.linspace(-len(crossing_agents)/2, len(crossing_agents)/2, len(crossing_agents))
    for num in range(len(crossing_agents)):
        crossing_agent_pose[num] = crossing_point + crossing_points_pose[num]*0.27*costmap.gap_orientation
    
    # print("Task Allocation part 1: ", taGap)
    taRHS, __ = solve_mtsp(crossing_agent_pose, costmap.tasks_rhs, costmap)
    # print(taRHS)
    #Format RHS tasks to be readable by mapping to actual robot index
    for taskListIdx in range(len(taRHS)):
        for task in taRHS[taskListIdx][1:]:
            agentIdx = list(crossing_pairing.keys())[taskListIdx] #crossing agents stores original agent indices
            #For example, if agents 2,3,4 are assigned to gap crossing, agent 2 would be the 0th entry in taRHS
            taGap[agentIdx].append(task + num_lhs + len(agents) - len(crossing_agents))

    return taGap, reordered__goals, goal_map_index, crossing_agents, crossing_pairing, costmap