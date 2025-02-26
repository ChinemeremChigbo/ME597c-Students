import numpy as np

# Type of planner
POINT_PLANNER = 0
TRAJECTORY_PLANNER = 1
PARABOLA_TRAJECTORY = 0
SIGMOID_TRAJECTORY = 1

class planner:
    def __init__(self, type_, trajectory_type=PARABOLA_TRAJECTORY):
        self.type = type_
        self.trajectory_type = trajectory_type
        print(trajectory_type)
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
        if self.trajectory_type == PARABOLA_TRAJECTORY:
            return self.parabola_trajectory()
        elif self.trajectory_type == SIGMOID_TRAJECTORY:
            return self.sigmoid_trajectory()

    def parabola_trajectory(self):
        # Generate a parabola from 0 to 1.5
        x_parabola = np.linspace(0.0, 1.5, num=50)
        y_parabola = x_parabola ** 2
        parabola_trajectory = np.column_stack((x_parabola, y_parabola))
        return parabola_trajectory.tolist()

    def sigmoid_trajectory(self):
        # Generate a sigmoid curve from 0 to 2.5
        x_sigmoid = np.linspace(0.0, 2.5, num=50)
        y_sigmoid = 2 / (1 + np.exp(-2 * x_sigmoid)) - 1
        sigmoid_trajectory = np.column_stack((x_sigmoid, y_sigmoid))
        return sigmoid_trajectory.tolist()
