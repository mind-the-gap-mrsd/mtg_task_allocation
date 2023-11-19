import matplotlib.pyplot as plt
import numpy as np
import cv2
from PIL import Image


img = Image.open(r"/home/sandy/Desktop/VSCode/final_map_cv.png").convert('L')
binary_map = np.array(img) < 128
cv2.waitKey(0)
distances = np.load('/home/sandy/Desktop/VSCode/mind-the-gap-mrsd/src/mtg_task_allocation/src/helper_pkg/distances_final_5.npz')["distances"]
# define the start point
start = (70, 72)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
ax1.imshow(binary_map, cmap='gray')
ax2.imshow(distances[start[0], start[1]], cmap = 'jet')
ax1.set_title('Binary Map')
ax2.set_title('Distances')
plt.show()