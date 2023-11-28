# mtg_task_allocation
Task Allocation ros node, using Hungarian algorithm and Python

## Note

Use this [drive link]{https://drive.google.com/drive/u/1/folders/1Wcjlr-nAQY3tuC02ui3PcKZtfMmOxHiD} to access the distances.npz file for the task allocator

Make change to line 44 of ta_mtsp_helper.py, replacing "PATH_TO_FILE" with the path to the downloaded npz file

Changes for ta_mtsp_helper to handle variable lengths:
    1. Instead of discretizing the map wrt the position of 3 agents, we calculate the gap boundaries based on the rotated map. 
    2. We find the distance to the closest point and threshold it to figure if we need 2 or 3 agents
    3. Check for obstacles by creating a window based on these point pairs, discard points with obstacles
    3. A new dictionary maps crossing points to a bool (false = 3 agents, true = 2 agents)
    4. When calculating cost of crossing in get gap points its calculated relative to the number of agents at each crossing point
    5. Extra cost for more points opposite the gap. 