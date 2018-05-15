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
    fig.show

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
    
    fig.show

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

# Distance vs. Entropy

# Distance vs. % Map Coverage


    
    