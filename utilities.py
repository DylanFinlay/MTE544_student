import os
from math import atan2, asin, sqrt

M_PI=3.1415926535

import numpy as np

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"], sync_root=True):
        
        self.filename = filename
        # If writing to a subdirectory, also mirror lines to the root filename for quick inspection
        self.sync_filename = os.path.basename(filename) if (sync_root and os.path.dirname(filename)) else None

        dir_name = os.path.dirname(self.filename)
        if dir_name:
            os.makedirs(dir_name, exist_ok=True)

        header_str = ""
        for header in headers:
            header_str += header + ", "
        header_str += "\n"

        with open(self.filename, 'w') as file:
            file.write(header_str)

        if self.sync_filename:
            with open(self.sync_filename, 'w') as file:
                file.write(header_str)

    def log_values(self, values_list):

        vals_str = ""
        for value in values_list:
            vals_str += f"{value}, "
        vals_str += "\n"

        with open(self.filename, 'a') as file:
            file.write(vals_str)

        if self.sync_filename:
            with open(self.sync_filename, 'a') as file:
                file.write(vals_str)

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line
            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    
# Conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    # just unpack yaw for tb
    return yaw


# Calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    return sqrt( (current_pose[0] - goal_pose[0])**2 +
                (current_pose[1] - goal_pose[1])**2 )

# Calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    error_angular= atan2(goal_pose[1]-current_pose[1],
                        goal_pose[0]-current_pose[0]) - current_pose[2]
    
    if error_angular <= -M_PI:
        error_angular += 2*M_PI
    
    
    elif error_angular >= M_PI:
        error_angular -= 2*M_PI
    
    return error_angular