#!usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import csv

# data_dir = "/home/saun/outDataSampling/src/data_exp/du_lieu_thuc_te"
data_dir = "/home/saun/outDataSampling/src/data_exp/Sai_so_mo_phong"

def plotting_data(load_dir):
    x_global, y_global = np.loadtxt(load_dir+'/data_global.csv', delimiter = ',', unpack=True)
    x_encoder, y_encoder = np.loadtxt(load_dir+'/data_encoder.csv', delimiter = ',', unpack=True)
    x_gps, y_gps, = np.loadtxt(load_dir+'/data_gps.csv', delimiter = ',', unpack=True)
    
    plt.style.use('seaborn-ticks')

    plt.figure(1)
    plt.plot(x_global, y_global, color='teal')
    plt.xlabel('x position')
    plt.ylabel('y position')
    plt.title('Odometry Global')
    plt.grid()
    
    plt.figure(2)
    plt.plot(x_encoder, y_encoder, color='darkslateblue')
    plt.xlabel('x position')
    plt.ylabel('y position')
    plt.title('Odometry Encoder')
    plt.grid()
    
    plt.figure(3)
    plt.plot(x_gps, y_gps, color='darkviolet')
    plt.xlabel('x position')
    plt.ylabel('y position')
    plt.title('Odometry GPS')
    plt.grid()
    
    plt.tight_layout()
    plt.show()
    
def plotting_multiple_data(load_dir):
    csv_files = [file for file in os.listdir(load_dir) if file.endswith('.csv')]
    plt.style.use('seaborn-poster')
    for csv_file in csv_files:
        csv_file_path = os.path.join(load_dir, csv_file)
        data = np.loadtxt(csv_file_path, delimiter=',')

        # Extract the x values from the first column
        x = data[:, 0]

        # Extract y values from all other columns (each column represents a line)
        y_columns = data[:, 1:]

        # Plot each line
        for i in range(y_columns.shape[1]):
            plt.plot(x, y_columns[:, i], marker='o', linestyle='-', label=f'{csv_file}')

    # Customize the plot
    # plt.title("Comparision between robot's odometry signals")
    # plt.xlabel('X-position (meters)')
    # plt.ylabel('Y-position (meters)')
    plt.title("Do thi so sanh du lieu dinh vi robot")
    plt.xlabel('Toa do X (meters)')
    plt.ylabel('Toa do Y (meters)')
    plt.grid(True)
    plt.legend()
    plt.show()
    

def plotting_single_columns(load_dir):
    csv_files = [file for file in os.listdir(load_dir) if file.endswith('.csv')]

    for csv_file in csv_files:
        # Construct the full path to the CSV file
        csv_file_path = os.path.join(load_dir, csv_file)

        # Read data from the CSV file
        with open(csv_file_path, 'r') as file:
            reader = csv.reader(file)
            x_values = [float(row[0]) for row in reader]

        # Plot the x-values against their index
        plt.plot(range(len(x_values)), x_values, marker='o', linestyle='-', label=f'{csv_file}')

    # Customize the plot
    plt.title('Du lieu sai so mo phong')
    plt.xlabel('t')
    plt.ylabel('Truc toa do (m)')
    plt.grid(True)
    plt.legend()

    # Show the plot
    plt.show()

if __name__ == '__main__':
    try:
        print("** PLOTTING DATA ***")
        # plotting_multiple_data(data_dir)
        plotting_single_columns(data_dir)
    except rospy.ROSInterruptException:
        print("--> Exiting !!!")
        pass