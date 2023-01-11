# Adapted from https://github.com/gisbi-kim/PyICP-SLAM/blob/master/utils/ScanContextManager.py

import numpy as np
np.set_printoptions(precision=4)

import time
from scipy import spatial


def xy2theta(x, y):
    if (x >= 0 and y >= 0): 
        theta = 180/np.pi * np.arctan(y/x)
    elif (x < 0 and y >= 0): 
        theta = 180 - ((180/np.pi) * np.arctan(y/(-x)))
    elif (x < 0 and y < 0): 
        theta = 180 + ((180/np.pi) * np.arctan(y/x))
    elif ( x >= 0 and y < 0):
        theta = 360 - ((180/np.pi) * np.arctan((-y)/x))
    return theta


def pt2rs(point, gap_ring, gap_sector, num_ring, num_sector):
    x = point[0]
    y = point[1]
    # z = point[2]
    
    if(x == 0.0):
        x = 0.001
    if(y == 0.0):
        y = 0.001
    
    theta = xy2theta(x, y)
    faraway = np.sqrt(x*x + y*y)
    
    idx_ring = np.divmod(faraway, gap_ring)[0]       
    idx_sector = np.divmod(theta, gap_sector)[0]

    if(idx_ring >= num_ring):
        idx_ring = num_ring-1 # python starts with 0 and ends with N-1
    
    return int(idx_ring), int(idx_sector)


def ptcloud2sc(ptcloud, sc_shape, max_length):
    num_ring = sc_shape[0]
    num_sector = sc_shape[1]

    gap_ring = max_length/num_ring
    gap_sector = 360/num_sector
    
    enough_large = 500
    sc_storage = np.zeros([enough_large, num_ring, num_sector])
    sc_counter = np.zeros([num_ring, num_sector])
    
    num_points = ptcloud.shape[0]
    for pt_idx in range(num_points):
        point = ptcloud[pt_idx, :]
        if np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2]):
            # Reject spurious points
            continue
        point_height = point[2] + 2.0 # for setting ground is roughly zero 
        
        idx_ring, idx_sector = pt2rs(point, gap_ring, gap_sector, num_ring, num_sector)
        
        if sc_counter[idx_ring, idx_sector] >= enough_large:
            continue
        sc_storage[int(sc_counter[idx_ring, idx_sector]), idx_ring, idx_sector] = point_height
        sc_counter[idx_ring, idx_sector] = sc_counter[idx_ring, idx_sector] + 1

    sc = np.amax(sc_storage, axis=0)
        
    return sc


def sc2rk(sc):
    return np.mean(sc, axis=1)

def distance_sc(sc1, sc2):
    num_sectors = sc1.shape[1]

    # repeat to move 1 columns
    _one_step = 1 # const
    sim_for_each_cols = np.zeros(num_sectors)
    for i in range(num_sectors):
        # Shift
        sc1 = np.roll(sc1, _one_step, axis=1) #  columne shift

        #compare
        sum_of_cossim = 0
        num_col_engaged = 0
        for j in range(num_sectors):
            col_j_1 = sc1[:, j]
            col_j_2 = sc2[:, j]
            if (~np.any(col_j_1) or ~np.any(col_j_2)): 
                # to avoid being divided by zero when calculating cosine similarity
                # - but this part is quite slow in python, you can omit it.
                continue 

            cossim = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
            sum_of_cossim = sum_of_cossim + cossim

            num_col_engaged = num_col_engaged + 1

        # save 
        if num_col_engaged == 0:
            sim_for_each_cols[i] = 0.0
        else:
            sim_for_each_cols[i] = sum_of_cossim / num_col_engaged

    yaw_diff = np.argmax(sim_for_each_cols) + 1 # because python starts with 0 
    sim = np.max(sim_for_each_cols)
    dist = 1 - sim

    return dist, yaw_diff