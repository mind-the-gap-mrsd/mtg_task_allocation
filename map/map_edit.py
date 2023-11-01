import cv2
import numpy as np
from typing import List
from shapely import LineString, Point, Polygon

def find_extreme_points(line_slope: float, line_point: np.ndarray, x_lim: List =[52, 90], y_lim: List=[42, 117]) -> List:
    # x_lim[0] intersect
    y_x_lim_0 = line_slope*(x_lim[0] - line_point[0]) + line_point[1]
    p_x_lim_0 = [x_lim[0], y_x_lim_0]

    # x_lim[1] intersect
    y_x_lim_1 = line_slope*(x_lim[1] - line_point[0]) + line_point[1]
    p_x_lim_1 = [x_lim[1], y_x_lim_1]

    # y_lim[0] intersect
    x_y_lim_0 = (y_lim[0] - line_point[1])/line_slope + line_point[0] 
    p_y_lim_0 = [x_y_lim_0, y_lim[0]]

    # y_lim[1] intersect
    x_y_lim_1 = (y_lim[1] - line_point[1])/line_slope + line_point[0] 
    p_y_lim_1 = [x_y_lim_1, y_lim[1]]

    points = [p_x_lim_0, p_x_lim_1, p_y_lim_0, p_y_lim_1]
    interior_points = []

    for pt in points:
        dir_1 = np.array(pt) - np.array([x_lim[0], y_lim[0]])
        dir_2 = np.array(pt) - np.array([x_lim[1], y_lim[1]])
        output = np.sum(np.multiply(dir_1, dir_2))
        if output <= 0:
            interior_points.append(pt)
    
    print((interior_points))
    return interior_points

if __name__ == "__main__":

    img = cv2.imread('no_obs_map.png')
    bw_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    height, width = bw_img.shape
    # x-y flipped from geeqie readings
    top_left = [50, 40]
    bottom_right = [92,120]
    slope = 0.0
    while(np.abs(slope) < 0.001):
        # randomly pick a non-zero slope in -pi/2 to pi/2 
        slope_angle = np.random.uniform(-np.pi/2, np.pi/2, 1)[0]
        slope = np.tan(slope_angle)
    gap_loc = np.random.uniform(top_left, bottom_right, size=2)
    #line_equation : y-gap_loc[0] = slope*(x - gap_loc[1])
    # 4 points of intersection:
    # at y = 42
    # at y = 117
    # x = 52
    # x = 90
    extreme_points = find_extreme_points(slope, gap_loc)
    line_points = np.linspace(extreme_points[0], extreme_points[1], 20)

    direction = np.array([slope, -1.0])
    direction /= np.linalg.norm(direction)
    direction = np.repeat(np.array([direction]), 20, axis= 0)

    left_line = line_points - (0.19/0.05)*direction
    right_line = line_points + (0.19/0.05)*direction
    random_shifts = np.random.uniform(-1, 1, 20)
    direction = np.array([slope, -1.0])
    direction /= np.linalg.norm(direction)
    direction = np.repeat(np.array([direction]), 20, axis= 0)
    delta = np.multiply(direction, random_shifts.reshape(-1,1))
    # print(delta)
    left_line_coords_rand = (left_line + delta).astype(int)
    right_line_coords_rand = (right_line - delta).astype(int)
    gap_polygon_coords = np.concatenate((left_line_coords_rand, np.flipud(right_line_coords_rand)), axis = 0)
    gap_polygon = Polygon(gap_polygon_coords)
    # print(left_line_coords_rand)
    # print(right_line_coords_rand)
    # print(list(center_line.coords))
    # print(list(left_line.coords)) ma
    # print(list(right_line.coords))

    # for x in range(top_left[0]-1, bottom_right[0]+1):
    #     for y in range(top_left[1]-1, bottom_right[1]+1):
    #         p = Point(x, y)
    #         if(gap_polygon.contains(p)):
    #             bw_img[x][y] = 127
    for x in range(79,83):
        for y in range(52, 91):
            bw_img[y][x] = 127


    # cv2.imshow('lol', bw_img)
    # cv2.waitKey(0)

    cv2.imwrite('map_with_gap_no_obs.png', bw_img)
