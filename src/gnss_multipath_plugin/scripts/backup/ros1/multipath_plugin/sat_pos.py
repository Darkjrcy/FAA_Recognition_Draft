import math
import numpy as np
from typing import Dict, List, Optional, Union
from collections import defaultdict
SPEED_OF_LIGHT = 2.99792458e8  # m/s
# Physical parameters of the Earth
EARTH_GM = 3.986005e14  # m^3/s^2 (gravitational constant * mass of earth)
EARTH_RADIUS = 6.3781e6  # m
EARTH_ROTATION_RATE = 7.2921151467e-005  # rad/s (WGS84 earth rotation rate)
GM = 3.986005 * 10**14
OMEGA_e = 7.292115 * 10**(-5)

def calculate_tk(t, toe) -> float:
    tk = t - toe
    if tk > 302400.0:
        tk = tk - 604800.0
    elif tk < -302400.0:
        tk = tk + 604800.0

    return tk

def calculate_Ek(Mk, e):
    Ek = Mk
    temp = Ek
    while math.fabs(Ek-temp) >= 1e-10:
        temp = Ek
        Ek = Mk + e*math.sin(Ek)

    return Ek

def calculate_satpos(sat_rinex: dict) -> tuple:
    A = sat_rinex['sqrtA']**2
    n_0 = math.sqrt(GM/ (A**3))
    n = n_0 + sat_rinex['DeltaN']
    e = sat_rinex['Eccentricity']
    tk = calculate_tk(sat_rinex['TransTime'],sat_rinex['Toe'])
    Mk = sat_rinex['M0'] + n * tk
    Ek = calculate_Ek(Mk, e)
    
    vk = math.atan2(math.sqrt(1-e*e) * math.sin(Ek), math.cos(Ek) - e)
    phi_k = vk + sat_rinex['omega']

    d_uk = sat_rinex['Cuc'] * math.cos(2*phi_k) + sat_rinex['Cus'] * math.sin(2*phi_k)
    d_rk = sat_rinex['Crc'] * math.cos(2*phi_k) + sat_rinex['Crs'] * math.sin(2*phi_k)
    d_ik = sat_rinex['Cic'] * math.cos(2*phi_k) + sat_rinex['Cis'] * math.sin(2*phi_k)

    uk = phi_k + d_uk
    rk = A * (1 - e * math.cos(Ek)) + d_rk
    ik = sat_rinex['Io'] + d_ik + sat_rinex['IDOT'] * tk

    xk_prim = rk * math.cos(uk)
    yk_prim = rk * math.sin(uk)

    omega_k = sat_rinex['Omega0'] + (sat_rinex['OmegaDot'] - OMEGA_e) * tk - OMEGA_e * sat_rinex['Toe'] 

    xk = xk_prim * math.cos(omega_k) - yk_prim * math.cos(ik) * math.sin(omega_k)
    yk = xk_prim * math.sin(omega_k) - yk_prim * math.cos(ik) * math.cos(omega_k)
    zk = yk_prim * math.sin(ik)

    return (xk, yk, zk)

def get_sat_position_ecef(obs, sat_string):
    sat_pos = []
    sat_rinex = {}
    for sat in sat_string:
        # Taking the only last element from the data obtained from the rinex file. 
        # This assumes that the satellite positions are fixed during the simulation window. 
        sat_rinex['SVclockBias'] = np.array(obs['SVclockBias'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['SVclockDrift'] = np.array(obs['SVclockDrift'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['SVclockDriftRate'] = np.array(obs['SVclockDriftRate'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['IODE'] = np.array(obs['IODE'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Crs'] = np.array(obs['Crs'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['DeltaN'] = np.array(obs['DeltaN'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['M0'] = np.array(obs['M0'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Cuc'] = np.array(obs['Cuc'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Eccentricity'] = np.array(obs['Eccentricity'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Cus'] = np.array(obs['Cus'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['sqrtA'] = np.array(obs['sqrtA'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Toe'] = np.array(obs['Toe'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Cic'] = np.array(obs['Cic'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Omega0'] = np.array(obs['Omega0'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Cis'] = np.array(obs['Cis'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['Io'] = np.array(obs['Io'].sel(sv=sat).dropna(dim='time',how='all'))[-1]  
        sat_rinex['Crc'] = np.array(obs['Crc'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['omega'] = np.array(obs['omega'].sel(sv=sat).dropna(dim='time',how='all'))[-1]  
        sat_rinex['OmegaDot'] = np.array(obs['OmegaDot'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['IDOT'] = np.array(obs['IDOT'].sel(sv=sat).dropna(dim='time',how='all'))[-1]  
        sat_rinex['CodesL2'] = np.array(obs['CodesL2'].sel(sv=sat).dropna(dim='time',how='all'))[-1]     
        sat_rinex['GPSWeek'] = np.array(obs['GPSWeek'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['L2Pflag'] = np.array(obs['L2Pflag'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['SVacc'] = np.array(obs['SVacc'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['health'] = np.array(obs['health'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['TGD'] = np.array(obs['TGD'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['IODC'] = np.array(obs['IODC'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        sat_rinex['TransTime'] = np.array(obs['TransTime'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        #sat_rinex['FitIntvl'] = np.array(obs['FitIntvl'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        (xk,yk,zk) = calculate_satpos(sat_rinex)
        sat_pos.append(np.array([xk, yk, zk]))
    return np.array(sat_pos) 