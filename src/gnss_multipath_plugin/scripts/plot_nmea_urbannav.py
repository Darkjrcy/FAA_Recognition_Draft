import os
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
#from pynmeagps import NMEAReader as nmr
from nmea import input_stream, data_frame

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/hk_data" 

if __name__ == "__main__":
    filename = "/20210517.light-urban.tste.ublox.m8t.GEJ.nmea"
    data_file = data_dir + filename
    


    stream = input_stream.GenericInputStream.open_stream(data_file)
    matplotlib.rcParams.update({'font.size': 22})
    plt.rcParams["figure.figsize"] = (10,10)
    with stream:
        new_frame = data_frame.DataFrame.get_next_frame(stream)
        fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
        azimuth = []
        elevation = []
        prn = []
        for obs in new_frame.sv_observations:
            azimuth.append(obs.azimuth)
            elevation.append(obs.elevation)
            prn.append(obs.prn)
        print(elevation, azimuth, prn)
        ax.scatter(azimuth,elevation, color = 'r', marker='H', label = 'Satellite', s = 300)
        for i in range(len(azimuth)):    
            ax.set_rlim(bottom=90, top=0)
            ax.set_rlabel_position(135)
            ax.set_rticks([0, 30, 60, 90])  # less radial ticks
            ax.grid(True)
            ax.legend(loc="lower left")
            ax.annotate("{}".format('PRN'+str(prn[i])), xy=[azimuth[i], elevation[i]-5], fontsize=22)
        plt.savefig('satellite_constellation.pdf')  
        plt.show()    
        
            
 