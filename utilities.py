from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
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
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat

    # Compute yaw (rotation around Z-axis)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(sin_yaw, cos_yaw)

    return yaw


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
    x_curr, y_curr, _, _ = current_pose
    x_goal, y_goal = goal_pose

    # Euclidean distance formula
    error_linear = sqrt((x_goal - x_curr) ** 2 + (y_goal - y_curr) ** 2)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):
    x_curr, y_curr, theta_curr, _ = current_pose
    x_goal, y_goal = goal_pose

    # Compute the desired heading using atan2
    theta_desired = atan2(y_goal - y_curr, x_goal - x_curr)

    # Compute the angular error
    error_angular = theta_desired - theta_curr

    # Normalize the angular error to be within [-π, π]
    error_angular = (error_angular + pi) % (2 * pi) - pi

    return error_angular

