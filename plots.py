# -*- coding: utf-8 -*-
"""
Created on Thu May 10 17:44:08 2018

@author: wcgru
"""
from pandas import DataFrame, read_csv
import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from mpl_toolkits.axes_grid1 import make_axes_locatable
# from matplotlib.animation import FuncAnimation

def create_pd_data(rel_path, csv_filename, windowsFlag = False):
    # Creates pandas data structure from csv file
    
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, rel_path + csv_filename)
    
    # this is only needed for windows
    if windowsFlag:
        filename = filename.replace('/','\\')
    
    pd_data = pd.read_csv(filename)
    
    return pd_data

def plot_gmapping_entropy(pd_data, title):
    minTimeStamp = pd_data['rosbagTimestamp'].min()
    
    pd_data['rosbagTimestamp'] = pd_data['rosbagTimestamp'] - minTimeStamp

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(pd_data['rosbagTimestamp'],pd_data['data'])
    ax.set_title(title)
    ax.set_xlabel('rosbag Timestamp (?)')
    ax.set_ylabel('gmapping entropy')
    fig.show()

def plot_cmd_vel(pd_data, title):
    minTimeStamp = pd_data['rosbagTimestamp'].min()
    
    pd_data['rosbagTimestamp'] = pd_data['rosbagTimestamp'] - minTimeStamp
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(pd_data['rosbagTimestamp'],np.asarray([pd_data['x'],pd_data['y'],pd_data['z']]).T)
    ax.set_title(title)
    ax.set_xlabel('rosbag Timestamp (?)')
    ax.set_ylabel('Linear Velocities (units?)')
    ax.legend(['x','y','z'])
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(pd_data['rosbagTimestamp'],np.asarray([pd_data['x.1'],pd_data['y.1'],pd_data['z.1']]).T)
    ax.set_title(title)
    ax.set_xlabel('rosbag Timestamp (?)')
    ax.set_ylabel('Angular Velocities (units?)')
    ax.legend(['x.1','y.1','z.1'])
    
    fig.show()

def plot_occupancy_grid(map_data, timestep, scenarioTitle):
    map_data_data = map_data['data']
    width = map_data['width'][0]
    height = map_data['height'][0]

    if timestep > len(map_data.index)-1:
        print('Timestep exeed map index, setting to final step of: ' + str(len(map_data.index)-1))
        timestep = len(map_data.index)-1
    
    initial_map = np.asarray(list(map(int,map_data_data[timestep][1:-1].split(','))))
    
    reshaped = np.reshape(initial_map, (width,height))
    
    reshaped_cut = reshaped[150:250,150:250]
    
    # replace -1 with -100 so it shows up clearly on the contour plot
    reshaped_cut[reshaped_cut < 0] = -100
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    divider = make_axes_locatable(ax)
    
    cs = ax.imshow(reshaped_cut)
    ax.set_title('Map at timestep ' + str(timestep) + scenarioTitle)
    ax.set_xlabel('Width')
    ax.set_ylabel('Height')
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = fig.colorbar(cs,cax,ticks=[-100, 0, 100])
    cbar.ax.set_yticklabels(['Not Explored', 'Unoccupied', 'Occupied'])
    fig.show()
    
def plot_map_coverage(map_data, scenarioTitle):
    minTimeStamp = map_data['rosbagTimestamp'].min()
    
    map_data['rosbagTimestamp'] = map_data['rosbagTimestamp'] - minTimeStamp
    
    map_data_data = map_data['data']
    width = map_data['width'][0]
    height = map_data['height'][0]
    
    final_timestep = len(map_data.index) -1
    final_map = np.asarray(list(map(int,map_data_data[final_timestep][1:-1].split(','))))
        
    reshaped = np.reshape(final_map, (width,height))
    
    reshaped_cut = reshaped[150:250,150:250]
        
    finalCoverage = (reshaped_cut >-1).sum()/(np.isfinite(reshaped_cut)).sum()
    
    rel_coverage =[]
    
    for timestep in range(len(map_data.index)):
        map_t = np.asarray(list(map(int,map_data_data[timestep][1:-1].split(','))))
        
        reshaped = np.reshape(map_t, (width,height))
    
        reshaped_cut = reshaped[150:250,150:250]
            
        rel_coverage.append((reshaped_cut >-1).sum()/(np.isfinite(reshaped_cut)).sum()/finalCoverage )
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    ax.plot(map_data['rosbagTimestamp'],rel_coverage)
    ax.set_title('Relative Map Coverage over Time (based off final map) for ' + scenarioTitle)
    ax.set_xlabel('rosbag Timestamp (?)')
    ax.set_ylabel('Map Coverage')
    fig.show()
    
    
#==============================================================================
# Nav example
#==============================================================================
rel_path = 'Example/nav_csv/'

# Time vs. Gmapping Entropy
csv_filename = '_slash_turtlebot3_slam_gmapping_slash_entropy.csv'
ent_data = create_pd_data(rel_path, csv_filename, windowsFlag = True)

plot_title = 'gmapping entropy - nav example'
plot_gmapping_entropy(ent_data, plot_title)

# Time vs. Commanded Velocity
csv_filename = '_slash_cmd_vel.csv'
cmd_vel_data = create_pd_data(rel_path, csv_filename, windowsFlag = True)

title = 'Commanded Velocities - Nav Example'
plot_cmd_vel(cmd_vel_data, title)


# Time vs. % Map Coverage (normalize based on final % map coverage?)
# need to update rel_path since _map is a large csv is not tracked on git

rel_path = 'Ignored/'
csv_filename = '_slash_nav_map.csv'

map_data = create_pd_data(rel_path, csv_filename, windowsFlag = True)
timestep = 5
scenarioTitle = '- nav Example'
plot_occupancy_grid(map_data, timestep, scenarioTitle)

# plot coverage over time
plot_map_coverage(map_data, scenarioTitle)

# uncomment below to plot occupancy grid for ALL timesteps
#==============================================================================
# for timestep in range(len(map_data.index)):
#     plot_occupancy_grid(map_data, timestep, scenarioTtle)
#==============================================================================

        
#initial_map = np.reshape(np.asarray(map_data_data[0]),(width,height))
#==============================================================================
# print(map_data_data[0])
#==============================================================================
# Distance vs. Entropy

# Distance vs. % Map Coverage


#==============================================================================
# teleop example
#==============================================================================
rel_path = 'Example/teleop_csv/'

# Time vs. Gmapping Entropy
csv_filename = '_slash_turtlebot3_slam_gmapping_slash_entropy.csv'
ent_data = create_pd_data(rel_path, csv_filename, windowsFlag = True)

plot_title = 'gmapping entropy - teleop example'
plot_gmapping_entropy(ent_data, plot_title)

# Time vs. Commanded Velocity
csv_filename = '_slash_cmd_vel.csv'
cmd_vel_data = create_pd_data(rel_path, csv_filename, windowsFlag = True)

title = 'Commanded Velocities - teleop Example'
plot_cmd_vel(cmd_vel_data, title)

# Time vs. % Map Coverage (normalize based on final % map coverage?)
rel_path = 'Ignored/'
csv_filename = '_slash_map.csv'

map_data = create_pd_data(rel_path, csv_filename, windowsFlag = True)

timestep = 80
scenarioTitle = '- teleop Example'
plot_occupancy_grid(map_data, timestep, scenarioTitle)

# Distance vs. Entropy

# Distance vs. % Map Coverage


    
    