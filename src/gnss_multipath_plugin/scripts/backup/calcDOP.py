import numpy as np
import georinex as gr
import math
import utm

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

def get_sat_position_ecef(rinex_file, sat_string):
    obs = gr.rinexnav(rinex_file, use='G')
    sat_pos = {}
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
        sat_rinex['FitIntvl'] = np.array(obs['FitIntvl'].sel(sv=sat).dropna(dim='time',how='all'))[-1] 
        (xk,yk,zk) = calculate_satpos(sat_rinex)
        sat_pos[sat] = {
            'x': xk,
            'y': yk,
            'z': zk
        }
    return sat_pos 

def conv_lla_to_ecef(lat, lon, alt):
    # convert lat and long from degrees to radians because numpy expects 
    # radians for trig functions
    deg_2_rads = np.pi/180
    lat = deg_2_rads*lat
    lon = deg_2_rads*lon    
    
    # convert altitude from kilometers to meters
    alt = 1000*alt    
    
    # convert LLA to ECEF with the following equations
    cos_lat = np.cos(lat)
    cos_lon = np.cos(lon)
    sin_lat = np.sin(lat)

    A = 6378137
    B = 6356752.31424518
    H = alt
    E1 = np.sqrt((A**2-B**2)/A**2)
    E2 = E1**2
    N = A/np.sqrt(1-E2*(sin_lat**2))

    X = (N+H)*cos_lat*cos_lon
    Y = (N+H)*cos_lat*np.sin(lon)
    Z = (N*(1-E2)+H)*sin_lat
    
    return X,Y,Z

def compute_dop(pos, pos_latlon, sat_pos, sat_vis):
    num_sats = len(sat_vis) 
    #format 'posOBS' to only have the observations ECEF coordinates
    obsX = pos[0]
    obsY = pos[1]
    obsZ = pos[2]
    # calculate the 'A' matrix
	# calculate the satellite locations and ranges to them
    satX = []
    satY = []
    satZ = []
    r = []
    for sat in sat_vis:
        satX.append(sat_pos[sat]['x'])
        satY.append(sat_pos[sat]['y'])
        satZ.append(sat_pos[sat]['z'])
        r.append(np.linalg.norm(np.array([obsX,obsY,obsZ]) - np.array([sat_pos[sat]['x'],sat_pos[sat]['y'],sat_pos[sat]['z']]))) 
    satX = np.array(satX)
    satY = np.array(satY)
    satZ = np.array(satZ)
	# format 'A' matrix
    A = np.column_stack((np.divide((obsX-satX),r), np.divide((obsY-satY),r), np.divide((obsZ-satZ),r), -np.ones((num_sats))))
    # calculate the cofactor matrix 'Q' which is the inverse of the normal
    # equation matrix the 'Q' matrix has the following components
    # [ qXX qXY qXZ qXt; qYX qYY qYZ qYt; qZX qZY qZZ qZt; qtX qtY qtZ qtt]
    Q = np.linalg.inv(np.matmul(np.transpose(A),A))
    # compute 'GDOP' 'PDOP', and 'TDOP' 
    GDOP = np.sqrt(np.trace(Q))
    PDOP = np.sqrt(np.trace(Q[0:3,0:3]))
    TDOP = np.sqrt(Q[3,3])
    # to compute 'HDOP' and 'VDOP' need rotation matrix from ECEF to local frame
	# convert ECEF OBS into latitude-longitude coordinates
    phi = np.deg2rad(pos_latlon[0])			# latitude
    lamda = np.deg2rad(pos_latlon[1])		# longitude
	# rotation matrix  'R'
    R = np.array([[-np.cos(lamda)*np.sin(phi), -np.sin(lamda)*np.sin(phi), np.cos(phi), 0],
         [ np.sin(lamda), np.cos(lamda), 0, 0],
         [ np.cos(lamda)*np.cos(phi), np.cos(phi)*np.sin(lamda), np.sin(phi), 0],
         [0, 0, 0, 1]])
	# calculate the local cofactor matrix
    Qlocal = np.matmul(np.matmul(R,Q), np.transpose(R))
	# calculate 'HDOP' and 'VDOP' 
    HDOP = np.sqrt(Qlocal[1,1] * Qlocal[2,2])
    VDOP = np.sqrt(Qlocal[3,3])
    
    return [GDOP, PDOP, TDOP, HDOP, VDOP]
