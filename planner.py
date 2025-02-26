import numpy as np

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_, trajectory_type="parabola"):

        self.type=type_
        self.trajectory_type = trajectory_type
    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return [x, y]

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        trajectory_points = []
        
        if self.trajectory_type == "parabola":
            x_values = np.linspace(0.0, 1.5, num=50)
            trajectory_points = [[x, x**2] for x in x_values]

        elif self.trajectory_type == "sigmoid":
            x_values = np.linspace(0.0, 2.5, num=50)
            trajectory_points = [[x, 2 / (1 + np.exp(-2 * x)) - 1] for x in x_values]

        return trajectory_points
