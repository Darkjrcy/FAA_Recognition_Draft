import math
import os
from matplotlib import figure
import matplotlib
import numpy as np
import json
import matplotlib.pyplot as plt
import matplotlib.colors as colors 
from multipath_sim.coord import conv_gzb_to_latlon, conv_latlon_to_gzb
import utm
data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 


gps_pos_file = data_dir + "pos_data_log.json"

gzb_gps_info = {}
with open(gps_pos_file, "r") as fs:
    gzb_gps_info = json.load(fs)
gzb_true_pos = np.array(gzb_gps_info['true_pos'])
gzb_gps_pos = np.array(gzb_gps_info['gps_pos'])

grd_truth = {}
with open(data_dir+"car_ground_truth.json", "r") as fs:
    grd_truth = json.load(fs)

gps_log = {}
with open(data_dir+"hk_data_car_gps_log.json", "r") as fs:
    gps_log = json.load(fs)




# Lat/Lon bounds for the segment of the map in gazebo     
lat_bound = np.array([22.3066, 22.2903])
lon_bound = np.array([114.171, 114.188])

# Applying correction in the offset to align the origin
origin_offset = np.array([77, 15])

# Get the Lat/Lon from ENU (gzb world)
#lat, lon = conv_gzb_to_latlon(gzb_true_pos[:,0], gzb_true_pos[:,1], lat_bound, lon_bound, origin_offset)
x, y = conv_latlon_to_gzb(np.array(gps_log["lat"]),np.array(gps_log["lon"]), lat_bound, lon_bound, origin_offset)



plt.figure(figsize=(8,6))
plt.plot(gzb_true_pos[:,0], gzb_true_pos[:,1],linestyle = 'solid', color = 'black', marker = None,markevery=100, markersize=5, linewidth=1)
plt.plot(gzb_gps_pos[:,0], gzb_gps_pos[:,1], linestyle = 'solid', color = 'red', marker = 'v', markevery=50 , markersize=5, linewidth=1)
plt.plot(x[:393], y[:393], color = 'blue', marker = '.', markersize=5, linewidth=1)
#plt.plot(x[:393], y[:393], '.',linestyle = 'dotted', color = 'blue', marker = 'v', markevery=100  ,markersize=5, linewidth=1)
plt.legend(["Ground Truth", "GNSS Multipath Plugin", "u Blox EKV-M8T"], fontsize = 14)
#plt.axis('off')
plt.xlabel('Easting (m)', fontsize = 14)
plt.ylabel('Northing (m)', fontsize = 14)
plt.grid()
plt.savefig('comparison_plugin_M8T.pdf')  
plt.show()




# #Calculating the size of the heat map based on the lat lon values
# scale = 5
# size_x_min = (int)((np.abs(gzb_true_pos[:,0].min()))/scale)
# size_y_min = (int)((np.abs(gzb_true_pos[:,1].min()))/scale)
# size_x = (int)((np.abs(gzb_true_pos[:,0].max()))/scale + size_x_min)
# size_y = (int)((np.abs(gzb_true_pos[:,1].max()))/scale + size_y_min)

# heat_map = np.zeros((size_y+1,size_x+1))
# for i in range(gzb_true_pos.shape[0]):
#     lon = (int)(gzb_true_pos[i,1]/scale + size_y_min)
#     lat = (int)(gzb_true_pos[i,0]/scale + size_x_min)
#     #Computing the norm of the offset at each location
#     #print(lon, lat, gzb_true_pos[i,1],gzb_true_pos[i,0], max_offset[i])
#     # if max_offset[i] > 10:
#     #     max_offset[i]=10
#     # heat_map[lon,lat] = max_offset[i]

#     error = np.linalg.norm(gzb_true_pos[i,:] - gzb_gps_pos[i,:])
#     if not math.isnan(error):
#         if error > 20:
#             error = 20
#         heat_map[lon,lat] = error
#     else: 
#         heat_map[lon,lat] = 20    

# from matplotlib.pyplot import cm
# import matplotlib.colors
# cmap = colors.LinearSegmentedColormap.from_list("", ["green","yellow","orange","red"])
# plt.imshow(heat_map, cmap=cmap, interpolation='bilinear')
# #plt.colorbar()
# plt.gca().invert_yaxis()
# plt.axis('off')
# cb = plt.colorbar() 
# cb.remove()
# #plt.savefig('ComparisonPlot.png', dpi=500)
# plt.show()
# plt.figure(figsize=(10,10))
# speed = np.zeros(500)
# for i in range (1,500):
#     speed[i]= np.sqrt((grd_truth['lat'][i] - grd_truth['lat'][i-1])**2 + (grd_truth['lon'][i] - grd_truth['lon'][i-1])**2)
# plt1 = plt.plot(speed, label = 'Speed')
# print(len(grd_truth['lat']),len(gps_log['lat']))
# error = np.zeros(500)
# for i in range (500):
#     error[i]= np.sqrt((grd_truth['lat'][i] - gps_log['lat'][i])**2 + (grd_truth['lon'][i] - gps_log['lon'][i])**2)
# plt2 = plt.gca().twinx().plot(error, color = 'r', label = 'Error in GPS position') 
# lns = plt1 + plt2
# labels = [l.get_label() for l in lns]
# plt.legend(lns, labels, loc=0)
# plt.show()

# plt.figure(figsize=(10,10))
# speed = np.zeros(len(aff_pos['true_x']))
# for i in range (1,len(aff_pos['true_x'])):
#     speed[i]= np.sqrt((aff_pos['true_x'][i] - aff_pos['true_x'][i-1])**2 + (aff_pos['true_y'][i] - aff_pos['true_y'][i-1])**2)
# plt1 = plt.plot(speed, label = 'Speed')

# error = np.zeros(len(aff_pos['true_x']))
# for i in range (len(aff_pos['true_x'])):
#     error[i]= np.sqrt((aff_pos['true_x'][i] - aff_pos['aff_x'][i])**2 + (aff_pos['true_y'][i] - aff_pos['aff_y'][i])**2)
# plt2 = plt.gca().twinx().plot(error, color = 'r', label = 'Error in GPS position') 
# lns = plt1 + plt2
# labels = [l.get_label() for l in lns]
# plt.legend(lns, labels, loc=0)
# plt.show()