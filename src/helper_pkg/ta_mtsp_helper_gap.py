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

class Costmap():
    ''' Initiates a costmap class with information about gap location. Splits and stores the tasks based on location wrt gap.'''
    def __init__(self, agents : np.ndarray, goals: np.ndarray) -> None:
        self.map_file = "/home/ros_ws/src/mtg_task_allocation/map/map_with_gap.png" #rospy.get_param(map_file)
        self.agents = agents
        self.goals = goals
        self.get_gap_coords()
        self.find_free_space() 
        
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
        self.contour_linestring = LineString(contours.tolist())
        self.centroid_of_gap = list(self.contour_linestring.centroid.coords)[0]
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
    
    def find_free_space(self, num_of_crossing_agents: int = 3) -> np.ndarray:
        ''' Finds the free space for coupling/decoupling in the map
         
          ### Method
            1. Find the gap orientation
            2. Rotate map so that gap is vertical
            3. Find the free space for coupling/decoupling by checking a sliding window
            4. Rotate points back to original orientation
            5. Return 2D array of free points  '''
        # Rotate map so that gap is vertical
        height, width = self.binary_map.shape
        angle = -1*np.arctan2(self.gap_orientation[1], self.gap_orientation[0])
        self.rotmatrix= cv2.getRotationMatrix2D((width/2, height/2), (180/np.pi)*angle, 1)
        self.rotated_map = cv2.warpAffine(self.map, self.rotmatrix, (width, height), flags=cv2.INTER_NEAREST)
        self.rotmatrix = np.vstack([self.rotmatrix, np.array([0, 0, 1])])

        self.free_space_map = copy.deepcopy(self.map)
        free_space = []

        for y in range(3, height-3, 1):
            for x in range(7, width-7, 1):
                window = self.map[y-3:y+4, x-7:x+7]
                if(np.any(window == 0)):
                    self.free_space_map[y,x] = 0
                elif(np.any(window == 127)):
                    self.free_space_map[y,x] = 127
                else:
                    self.free_space_map[y,x] = 255
                    original_point = np.linalg.inv(self.rotmatrix)@np.array([x,y,1]).reshape(-1,1)
                    original_point = original_point.flatten()
                    original_point /= original_point[-1]
                    free_space.append(original_point[:2])
        
        plt.imshow(self.free_space_map)
        plt.show()
        
        return np.array(free_space)