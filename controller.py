import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

#NOTE: BEFORE RUNNING THE REAL ROBOT STUFF MAKE SURE YOU SWITCH SATURATION LIMITS SINCE SIM ONES ARE HIGHER

# sim saturation limits
TB3_MAX_ANGULAR_VEL = 2.84 # rad 
TB3_MAX_LINEAR_VEL = 0.22 # m/s

# real saturation limits
TB4_MAX_ANGULAR_VEL = 1.9 # rad 
TB4_MAX_LINEAR_VEL = 0.31 # m/s

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(P, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(P, kap, kav, kai, filename_="angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity

        linear_vel = TB3_MAX_LINEAR_VEL if linear_vel > TB3_MAX_LINEAR_VEL else linear_vel
        angular_vel= TB3_MAX_ANGULAR_VEL if angular_vel > TB3_MAX_ANGULAR_VEL else angular_vel
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity

        # same as previous 
        linear_vel = TB3_MAX_LINEAR_VEL if linear_vel > TB3_MAX_LINEAR_VEL else linear_vel
        angular_vel= TB3_MAX_ANGULAR_VEL if angular_vel > TB3_MAX_ANGULAR_VEL else angular_vel
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
