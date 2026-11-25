

import json
import os
import sys
import time
from datetime import datetime


from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error, Logger
from pid import PID_ctrl
from plot_runs import generate_run_plots

from rclpy import init, spin, spin_once
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensors, kalmanFilter

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController


from geometry_msgs.msg import PoseStamped


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class decision_maker(Node):
    
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER, heuristic="e"):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)


        self.create_subscription(PoseStamped, "/goal_pose", self.designPathFor, 10)
        
        self.pathPublisher = self.create_publisher(Path, '/designedPath', 10)
        
        publishing_period=1/rate

        # Create a dedicated run folder under ./logs/<heuristic>_<timestamp>/
        heuristic_map = {"e": "euclidean", "m": "manhattan"}
        self.heuristic = heuristic
        self.heuristic_name = heuristic_map.get(heuristic, heuristic).capitalize() if motion_type == TRAJECTORY_PLANNER else "Point"
        folder_prefix = heuristic_map.get(heuristic, heuristic) if motion_type == TRAJECTORY_PLANNER else "point"
        run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        logs_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        self.run_dir = os.path.join(logs_root, f"{folder_prefix}_{run_timestamp}")
        os.makedirs(self.run_dir, exist_ok=True)
        print(f"Logging run outputs to: {self.run_dir}")

        self.goal_index = 0
        self.active_goal_meta = None
        self.active_traj_logger = None
        self.run_info = {
            "heuristic": heuristic,
            "heuristic_name": self.heuristic_name,
            "timestamp": run_timestamp,
            "map_name": "room",
            "goals": [],
        }
        self._save_run_info()

        # TODO PART 5 choose your threshold
        # from our prev lab 
        self.reachThreshold=0.05

        # TODO PART 5 your localization type
        self.localizer=localization(
            kalmanFilter,
            loggerName=os.path.join(self.run_dir, "robotPose.csv"),
            loggerHeaders=["x", "y", "yaw", "stamp"],
        )


        
        if motion_type==POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
            self.planner=planner(POINT_PLANNER)

        
        elif motion_type==TRAJECTORY_PLANNER:
            # TODO PART 5 Put the gains that you concluded from lab 2
            self.controller=trajectoryController(klp=0.1, klv=0.2, kli=0.05, kap=0.8, kav=0.4, kai=0.0)
            self.planner=planner(TRAJECTORY_PLANNER, heuristic=heuristic)
        
        else:
            print("Error! you don't have this type of planner", file=sys.stderr)

        # Redirect PID error loggers into this run's folder
        if hasattr(self, "controller"):
            self.controller.PID_linear.logger = Logger(os.path.join(self.run_dir, "linear.csv"))
            self.controller.PID_angular.logger = Logger(os.path.join(self.run_dir, "angular.csv"))


        self.goal = None

        self.create_timer(publishing_period, self.timerCallback)


        print("waiting for your input position, use 2D nav goal in rviz2")

    def _save_run_info(self):
        info_path = os.path.join(self.run_dir, "run_info.json")
        with open(info_path, "w") as f:
            json.dump(self.run_info, f, indent=2)

    def generate_plots(self, force=True):
        self._save_run_info()
        try:
            created = generate_run_plots(self.run_dir, map_name=self.run_info.get("map_name", "room"), force=force)
            if created:
                print(f"Saved {len(created)} plot(s) to {self.run_dir}")
        except Exception as exc:
            print(f"Warning: failed to generate plots automatically: {exc}", file=sys.stderr)




    # This is for the rviz2 interface
    def designPathFor(self, msg: PoseStamped):
        
        spin_once(self.localizer)
        
        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        pose = self.localizer.getPose()
        start_xy = [float(pose[0]), float(pose[1])]
        end_xy = [float(msg.pose.position.x), float(msg.pose.position.y)]

        t0 = time.time()
        self.goal=self.planner.plan(start_xy, end_xy)
        plan_time_s = time.time() - t0

        if self.goal is None:
            print(f"Warning: no path found from {start_xy} to {end_xy}")
            self.active_traj_logger = None
            self.active_goal_meta = None
            return

        # Record planned path and start a dedicated actual-trajectory CSV for this goal
        self.goal_index += 1
        planned_file = f"planned_path_goal{self.goal_index}.csv"
        actual_file = f"actual_trajectory_goal{self.goal_index}.csv"

        planned_logger = Logger(os.path.join(self.run_dir, planned_file), headers=["x", "y"])
        if isinstance(self.goal, list) and len(self.goal) > 0 and isinstance(self.goal[0], (list, tuple)):
            for pt in self.goal:
                planned_logger.log_values([float(pt[0]), float(pt[1])])
        else:
            planned_logger.log_values([float(self.goal[0]), float(self.goal[1])])

        self.active_traj_logger = Logger(
            os.path.join(self.run_dir, actual_file),
            headers=["x", "y", "yaw", "stamp"],
        )
        self.active_goal_meta = {
            "goal_index": self.goal_index,
            "start": start_xy,
            "end": end_xy,
            "planned_path_file": planned_file,
            "actual_trajectory_file": actual_file,
            "plan_time_s": round(plan_time_s, 4),
            "reached": False,
        }
        self.run_info["goals"].append(self.active_goal_meta)
        self._save_run_info()

    
    def timerCallback(self):
        
        spin_once(self.localizer)

        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        pose = self.localizer.getPose()
        stamp_ns = Time.from_msg(pose[3]).nanoseconds
        pose_row = [float(pose[0]), float(pose[1]), float(pose[2]), stamp_ns]
        self.localizer.loc_logger.log_values(pose_row)
        
        vel_msg=Twist()
        
        if self.goal is None:
            return

        if self.active_traj_logger is not None:
            self.active_traj_logger.log_values(pose_row)
        
        if type(self.goal) == list:
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal[-1]) <self.reachThreshold else False
        else: 
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal) <self.reachThreshold else False




        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()

            if self.active_goal_meta is not None:
                self.active_goal_meta["reached"] = True
            self.active_traj_logger = None
            self.active_goal_meta = None
            self.generate_plots(force=True)
            
            self.goal = None
            print("waiting for the new position input, use 2D nav goal on map")

            return
        
        velocity, yaw_rate = self.controller.\
            vel_request(self.localizer.getPose(), self.goal, True)

        
        vel_msg.linear.x=velocity
        vel_msg.angular.z=yaw_rate
        
        self.publisher.publish(vel_msg)
        self.publishPathOnRviz2(self.goal)



    def publishPathOnRviz2(self, path):

        Path_ =  Path()

        Path_.header.frame_id ="map"
        Path_.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            # Set the orientation of the pose. Here, it's set to a default orientation.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            Path_.poses.append(pose)

        self.pathPublisher.publish(Path_)

import argparse
def main(args=None):
    
    
    init()
    
    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    
    if args.motion == "point":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=POINT_PLANNER)
    elif args.motion == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=TRAJECTORY_PLANNER, heuristic=args.heuristic)
    else:
        print("invalid motion type", file=sys.stderr)



    try:
        spin(DM)
    except KeyboardInterrupt:
        print("\nInterrupted (CTRL+C). Finalizing logs and generating plots...")
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")
    finally:
        DM.generate_plots(force=True)




if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="trajectory")
    argParser.add_argument("--heuristic", choices=("m", "e"), default="e", help="m: Manhattan, e: Euclidean")
    args = argParser.parse_args()

    main(args)
