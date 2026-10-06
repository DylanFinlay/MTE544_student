# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# Message types for motion commands and sensors
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_ = 0.0
        self.linear_velocity_ = 0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # QoS profile for sensor data (TB4 sensors use Best Effort)
        qos=QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publisher for velocity commands
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos)

        # Sensor subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos)
        
        self.create_timer(0.1, self.timer_callback)


    # Callback functions to log sensor data
    def imu_callback(self, imu_msg: Imu):
        self.imu_initialized = True
        stamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
        self.imu_logger.log_values([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.angular_velocity.z,
            stamp
        ])
        
    def odom_callback(self, odom_msg: Odometry):
        self.odom_initialized = True
        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        th = euler_from_quaternion(odom_msg.pose.pose.orientation)
        self.odom_logger.log_values([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            th,
            stamp
        ])
                
    def laser_callback(self, laser_msg: LaserScan):
        self.laser_initialized = True
        stamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        self.laser_logger.log_values([
            list(laser_msg.ranges),
            laser_msg.angle_increment,
            stamp
        ])
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # Motion functions
    def make_circular_twist(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.4
        return msg

    def make_spiral_twist(self):
        msg = Twist()
        # gradually increase radius
        self.radius_ = min(self.radius_ + 0.001, 1.0)
        msg.angular.z = 0.4
        msg.linear.x = msg.angular.z * self.radius_
        return msg
    
    def make_acc_line_twist(self):
        msg = Twist()
        # gradually increase forward speed
        self.linear_velocity_ = min(self.linear_velocity_ + 0.002, 0.3)
        msg.linear.x = self.linear_velocity_
        msg.angular.z = 0.0
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {args.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
