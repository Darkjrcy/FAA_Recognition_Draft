# %%
import os
import numpy as np
import json
import matplotlib.pyplot as plt

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 

# %%
random_walk_file = data_dir + "hk_data_heatmap.json"

random_walk = {}
with open(random_walk_file, "r") as fs:
    random_walk = json.load(fs)
marker_size = 20
plt.subplot(1,2,1)
plt.title("Gaussian Random Walk Noise", fontsize = 15)
plt.plot(random_walk["gps_bias_x"][:500], random_walk["gps_bias_y"][:500])


plt.subplot(1,2,2)
plt.title("Uncorrelated Gaussian Noise", fontsize = 15)
mu, sigma = 0, 0.5 # mean and standard deviation
random_x = np.random.normal(mu, sigma, 500)
random_y  =  np.random.normal(mu, sigma, 500)
plt.plot(random_x, random_y)
plt.show()