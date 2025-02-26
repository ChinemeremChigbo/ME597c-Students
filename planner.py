# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



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
        return [x, y]

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        x_parabola = np.linspace(0.0, 1.5, num=50)
        y_parabola = x_parabola ** 2
        parabola_trajectory = np.column_stack((x_parabola, y_parabola))

        x_sigmoid = np.linspace(0.0, 2.5, num=50) 
        y_sigmoid = 2 / (1 + np.exp(-2 * x_sigmoid)) - 1
        sigmoid_trajectory = np.column_stack((x_sigmoid, y_sigmoid))

        # Combine both trajectories in a list and return them
        return parabola_trajectory.tolist(), sigmoid_trajectory.tolist()
