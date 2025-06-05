# %%
import os
import numpy as np
import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.pyplot import cm
data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 


# %%
heat_map_file = data_dir + "hk_data_heatmap.json"
aff_pos = {}
with open(heat_map_file, "r") as fs:
    aff_pos = json.load(fs)

grd_truth = {}
with open(data_dir+"car_ground_truth.json", "r") as fs:
    grd_truth = json.load(fs)

gps_log = {}
with open(data_dir+"hk_data_car_gps_log.json", "r") as fs:
    gps_log = json.load(fs)

marker_size = 20
plt.figure(figsize=(20,10))
plt.subplot(1,2,1)
image_dir =data_dir+"map.png"
img = mpimg.imread(image_dir)
plt.imshow(img, aspect="auto")
plt.axis('off')
plt.title("Hong Kong World")

plt.subplot(1,2,2)
#Calculating the size of the heat map based on the lat lon values
size_x_min = (int)(abs(min(aff_pos["aff_x"]))/10)
size_y_min = (int)(abs(min(aff_pos["aff_y"]))/10)
size_x = (int)(max(aff_pos["aff_x"])/10 + size_x_min)
size_y = (int)(max(aff_pos["aff_y"])/10 + size_y_min)

heat_map_offset = np.zeros((size_y,size_x))

for i in range(len(aff_pos["true_x"])):
    lon = (int)(aff_pos["true_y"][i]/10 + size_y_min -1)
    lat = (int)(aff_pos["true_x"][i]/10 + size_x_min - 1)
    #Computing the norm of the offset at each location
    heat_map_offset[lon,lat] = np.linalg.norm(np.array([aff_pos["offset_x"][i],aff_pos["offset_y"][i]]))

plt.imshow(heat_map_offset, cmap=cm.hot, interpolation='bilinear', aspect="auto")
plt.colorbar()
plt.gca().invert_yaxis()
plt.axis('off')
plt.title("Error Offset")
plt.savefig('heatmap.eps', format='eps')
# plt.subplot(1,3,3)
# heat_map_sat = np.zeros((size_y,size_x))
# for i in range(len(aff_pos["true_x"])):
#     lon = (int)(aff_pos["true_y"][i]/10 + size_y_min -1)
#     lat = (int)(aff_pos["true_x"][i]/10 + size_x_min - 1)
#     #Computing the norm of the offset at each location
#     heat_map_sat[lon,lat] = aff_pos['num_block_sat'][i]
# plt.imshow(heat_map_sat, cmap=cm.Blues, interpolation='bilinear', aspect="auto")
# plt.colorbar()
# plt.axis('off')
# plt.gca().invert_yaxis()   
# plt.title("Number of Blocked Satellites")
plt.show()
# %%
