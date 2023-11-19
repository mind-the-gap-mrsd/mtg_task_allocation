import numpy as np
from heapq import heappush, heappop
import cv2
from PIL import Image
import matplotlib.pyplot as plt
import time
def isWall(row, col, binary_map):
    if 0 <= row < binary_map.shape[0] and 0 <= row < binary_map.shape[1]:
        return False
    else:
        return True
     
def dijkstra_binary_map(binary_map, start):
    # convert start tuple to integer indices
    start_row, start_col = start
    
    # initialize the distances array with infinite distances except for the starting point
    distances = np.full(binary_map.shape, np.inf)
    distances[start_row, start_col] = 0
    
    # create a priority queue and add the starting point
    pq = []
    #visited = []#np.full(binary_map.shape, 10000)
    heappush(pq, (0, start_row, start_col))
    directions = [[0,1], [0, -1], [1, 0], [-1, 0]]
    # process the queue until it's empty

    while pq:
        #print(visited)
        # get the closest point in the queue
        curr_dist, curr_row, curr_col = heappop(pq)
        # if curr_dist > visited.get(current, np.inf):
        #     continue
        if curr_dist > distances[curr_row, curr_col]:
            continue

        for step in directions: #check all neighboring points
            new_row = curr_row + step[0]
            new_col = curr_col + step[1]
            #print(new_row, new_col)
            

        # check all the neighboring points
        #for new_row, new_col in [(curr_row, curr_col+1), (curr_row, curr_col-1), (curr_row+1, curr_col), (curr_row-1, curr_col)]:
            # if the neighbor is within the bounds of the map and is not a wall
            if  0 <= new_row < binary_map.shape[0] and 0 <= new_col < binary_map.shape[1]: 
            #isWall(new_row, new_col, binary_map): #and [new_row, new_col] not in visited:
                #print(1)
                #visited[str(new_row) + "," + str(new_col)] = True
                #visited.append([new_row, new_col])
                if binary_map[new_row, new_col] != 1:
            #if 0 <= new_row < binary_map.shape[0] and 0 <= new_col < binary_map.shape[1] and binary_map[new_row, new_col] != 1:
                # calculate the new distance
                    new_dist = curr_dist + 1
                
                # if the neighbor hasn't been visited or the new distance is shorter
                    if new_dist < distances[new_row, new_col]:
                    # update the distance
                        distances[new_row, new_col] = new_dist
                    #print(row, col, new_row, new_col, new_dist)
                    # add the neighbor to the queue
                        heappush(pq, (new_dist, new_row, new_col))
                #print(visited)
    
    # return the distances array
    return distances

# create a binary map
# binary_map = np.array([[0, 0, 0, 1, 0],
#                        [0, 1, 0, 1, 0],
#                        [0, 1, 0, 0, 0],
#                        [0, 0, 1, 1, 0],
#                        [0, 0, 0, 0, 0]])

#map = cv2.imread(r"puzzlebots_ws/src/mtg_mtsp_task_allocator/src/helper_pkg/map.png")
# cv2.imshow('m', map)
# cv2.waitKey(0)
# map = cv2.cvtColor(map, cv2.COLOR_BGR2GRAY)
# r, binary_map = cv2.threshold(map, 127, 1, cv2.THRESH_BINARY)
# cv2.imshow('s', binary_map)
# cv2.waitKey(0)


# read the PNG file and convert it to a binary map
img = Image.open(r"/home/sandy/Desktop/VSCode/final_map_cv.png").convert('L')
binary_map = np.array(img) < 128
#cv2.waitKey(0)

# define the start point
start = (100, 60)

# compute the distances from all points to all other points
# starttime = time.time()
# distances = dijkstra_binary_map(binary_map, start)
# end = time.time()
# print(starttime - end)
# #print(distances[106, 144])
# # plot the binary map and the distances
# fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
# ax1.imshow(binary_map, cmap='gray')
# ax1.set_title('Binary Map')
# ax2.imshow(distances, cmap='jet')
# ax2.set_title('Distances')
# plt.show()
distances = np.zeros((binary_map.shape[0], binary_map.shape[1], binary_map.shape[0], binary_map.shape[1]))
visited = []
print("starting loop...")
for row in range(binary_map.shape[0]):
    for col in range(binary_map.shape[1]):
        if [row, col] not in visited:
            distances[row, col, :, :] = dijkstra_binary_map(binary_map, (row, col))
            distances[:, :, row, col] = distances[row, col, :, :]
            visited.append([row, col])
print("jnkskdaxm,")
# print(distances)
np.savez_compressed("/home/sandy/Desktop/VSCode/mind-the-gap-mrsd/src/mtg_task_allocation/src/helper_pkg/distances_final_cached.npz", distances = distances)
#print the distances arrayprint(distances)