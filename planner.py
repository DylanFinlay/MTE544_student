import math

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1
NUMBER_OF_POINTS = 100
SIGMOID_RANGE = 2.5
QUADRATIC_RANGE = 1.5

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        
        points = []

        # quadtratic function
        for i in range(NUMBER_OF_POINTS): 
            x = QUADRATIC_RANGE * i / (NUMBER_OF_POINTS -1) 
            y = x **2 
            points.append([x,y])

        # Sigmoid function 
        # for i in range(NUMBER_OF_POINTS): 
        #     x = SIGMOID_RANGE * i / (NUMBER_OF_POINTS -1) 
        #     y = 2 / (1 + math.exp(-2*x)) -1 
        #     points.append([x,y])
            
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return points 
