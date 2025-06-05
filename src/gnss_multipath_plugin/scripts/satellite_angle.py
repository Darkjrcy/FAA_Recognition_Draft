import os
import georinex as gr
import sys

import numpy as np
sys.path.append('../')
from multipath_sim.sat_pos import get_sat_position_ecef
from datetime import datetime, timezone
from multipath_sim.laika.gps_time import GPSTime
from multipath_sim.laika.astro_dog import AstroDog

from orbit_predictor.sources import get_predictor_from_tle_lines
from orbit_predictor.sources import EtcTLESource, NoradTLESource
from orbit_predictor.locations import Location
from orbit_predictor.utils import jday_from_datetime


def main(args=None):
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    nav_file = data_dir + '/hksc135c.21n'
    nav_data = gr.rinexnav(nav_file, use='G')
    #sats = ['G01','G04','G07','G08', 'G09','G16','G21', 'G27']
    sats  = ['G01']
    sat_pos = get_sat_position_ecef(nav_data, sats)
    print(sat_pos)

    # dog = AstroDog(valid_const=['GPS'])
    # time = GPSTime.from_datetime(datetime(2021, 5, 15,  4,  0))
    # # We use RINEX3 PRNs to identify satellites
    # sat_prn = 'G01'
    # sat_pos, _, _, _, ephemeris = dog.get_sat_info(sat_prn, time)
    # print(sat_pos)
    source = NoradTLESource.from_file(filename="gps.tle")
    predictor = source.get_predictor("GPS BIIF-2  (PRN 01)")

    # predictor = get_predictor_from_tle_lines(TLE_LINES)
    position = predictor.get_position(datetime(2021, 5, 15, 2, 30,0))
    hk = Location("hongkong", 22.299721, 114.178133, 0)
    print(hk.is_visible(position), hk.get_azimuth_elev_deg(position))
    print(position)
    predictor = source.get_predictor("GPS BIIRM-6 (PRN 07)")

    
    # predictor = get_predictor_from_tle_lines(TLE_LINES)
    position = predictor.get_position(datetime(2021, 5, 15, 2, 30,0))
    
    t  = datetime(2021, 5, 15, 3, 0, 0)
    timestamp = t.replace(tzinfo=timezone.utc).timestamp()
    print(timestamp, jday_from_datetime(t))
    hk = Location("hongkong", 22.299721, 114.178133, 0)
    print(hk.is_visible(position), hk.get_azimuth_elev_deg(position))
    print(position)








    #     # We can use these helpers to plot the orbits of satellites
    # # by plotting the satellite positions over the course
    # # of 12 hours, which is the approximate orbital period
    # # of GNSS satellites.

    # from collections import defaultdict
    # import PIL
    # import matplotlib.pyplot as plt
    # import numpy as np
    # import matplotlib.patches as mpatches
    # from mpl_toolkits.mplot3d import Axes3D
    # from laika.constants import EARTH_RADIUS, EARTH_ROTATION_RATE
    # from laika.helpers import get_constellation
    # from laika.lib.orientation import rot_from_euler

    # # We start by plotting the world

    # # load bluemarble with PIL
    # bm = PIL.Image.open('bluemarble.jpg')
    # # it's big, so I'll rescale it, convert to array, and divide by 256 to get RGB values that matplotlib accept 
    # bm = np.array(bm.resize([d // 5 for d in bm.size])) / 256.
    # lons = np.linspace(-180, 180, bm.shape[1]) * np.pi / 180
    # lats = np.linspace(-90, 90, bm.shape[0])[::-1] * np.pi / 180
    # fig = plt.figure(figsize=(14, 14))
    # ax = fig.add_subplot(111, projection='3d')
    # x = EARTH_RADIUS * np.outer(np.cos(lons), np.cos(lats)).T
    # y = EARTH_RADIUS * np.outer(np.sin(lons), np.cos(lats)).T
    # z = EARTH_RADIUS * np.outer(np.ones(np.size(lons)), np.sin(lats)).T
    # ax.plot_surface(x, y, z, rstride=4, cstride=4, facecolors=bm)
    # ax.set_xlim(-2.5e7, 2.5e7)
    # ax.set_ylim(-2.5e7, 2.5e7)
    # ax.set_zlim(-2.5e7, 2.5e7)
    # ax.set_box_aspect((1, 1, 1))

    # # Now we get all the satellite positions
    # # over a 12 hour period and plot them
    # sat_positions = defaultdict(lambda: [])
    # colors = {
    #     'GPS': 'b',
    #     'GLONASS': 'r'
    # }

    # for i in range(49):
    #     dt = i * 15 * 60
    #     all_sat_info = dog.get_all_sat_info(time + dt)
    #     for sat_prn, sat_info in all_sat_info.items():
    #         positions = sat_positions[sat_prn]
    #         theta = EARTH_ROTATION_RATE * dt
    #         rot_matrix = rot_from_euler((0, 0, theta))
    #         positions.append(rot_matrix.dot(sat_info[0]))

    # for sat_prn, positions in sat_positions.items():
    #     if len(positions) < 6:
    #         continue
    #     ax.plot([p[0] for p in positions],
    #             [p[1] for p in positions],
    #             [p[2] for p in positions],
    #             color=colors[get_constellation(sat_prn)])

    # patches = [mpatches.Patch(color='b', label='GPS orbits')]
    # plt.legend(handles=patches, fontsize=14)
    # plt.title('GPS orbits', fontsize=25)
    # plt.show()





if __name__ == '__main__':
    main()