# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np 
from glob import glob 
import os 

def plot_errors(filename, output_folder):
    
    headers, values=FileReader(filename).read_file() 

    if "laser" in filename:  # ONLY WORKS if laser is in the laser scan csv filenames

        # get first row
        row = values[0]  
        ranges = np.array(row[:-2], dtype=float)

        # angle increment is 2nd last val in row (see data or code in motion.py)
        angle_increment = row[-2]

        # convert lidar data to angle 
        angles = np.arange(len(ranges)) * angle_increment 

        # fiter infintie vals
        filtered = np.isfinite(ranges)

        # get x y vals from filtered data 
        x = ranges[filtered] * np.cos(angles[filtered])
        y = ranges[filtered] * np.sin(angles[filtered])

        # scatter plot w/ x y as axis 
        plt.scatter(x, y, s=4, label="Laser Scan")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.axis("equal")


    # regular case
    else: 
        time_list=[]
        first_stamp=values[0][-1]
        
        for val in values:
            time_list.append(val[-1] - first_stamp)

        for i in range(0, len(headers) - 1):
            plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.legend()
    plt.grid()

    # save into folder based on output arg 
    if output_folder:

        # make dir 
        os.makedirs(output_folder, exist_ok=True)

        # save 
        base = os.path.splitext(os.path.basename(filename))[0]
        save_path = os.path.join(output_folder, base + ".png")
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print("saved", save_path)
    else:
        plt.show()
    plt.show()
    
import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', help='List of files to process')
    parser.add_argument('--folder', help='Folder containing files to process')
    parser.add_argument('--output_folder', help='Folder to save data')

    
    args = parser.parse_args()
    
    filenames = []

    if args.files:
        filenames.extend(args.files)

    # reads from folder and plots all csvs in the folder path 
    if args.folder:
        folder_files = sorted(glob(os.path.join(args.folder, "*.csv")))
        filenames.extend(folder_files)

    if not filenames:
        parser.error("Provide at least one of --files or --folder")

    print("plotting the files", filenames)

    for filename in filenames:
        plot_errors(filename, args.output_folder)