
from mapUtilities import *
from utilities import *
from numpy import cos, sin
import numpy as np


class particle:

    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def motion_model(self, v, w, dt):
        #TODO: Implement the motion model for the particle
        """
        v: linear velocity
        w: angular velocity
        dt: time step
        """
        if abs(w) < 1e-6:
            self.pose[0] += v * dt * cos(self.pose[2])
            self.pose[1] += v * dt * sin(self.pose[2])
        else:
            self.pose[0] += (v / w) * (sin(self.pose[2] + w * dt) - sin(self.pose[2]))
            self.pose[1] += (v / w) * (-cos(self.pose[2] + w * dt) + cos(self.pose[2]))
            self.pose[2] += w * dt

        self.pose[2] = (self.pose[2] + np.pi) % (2 * np.pi) - np.pi

    # TODO: You need to explain the following function to TA
    def calculateParticleWeight(self, scanOutput: LaserScan, mapManipulatorInstance: mapManipulator, laser_to_ego_transformation: np.array):
        """
        Updates the particle's weight based on the likelihood of the given laser scan.

        scanOutput: LaserScan - The current laser scan data.
        mapManipulatorInstance: mapManipulator - An instance of the map utility class.
        laser_to_ego_transformation: np.array - A transformation matrix from laser frame to ego-frame.

        - The laser scan data is converted to Cartesian coordinates.
        - The scan points are transformed into the map frame using the particle's pose.
        - The transformed scan points are mapped to discrete cell positions.
        - The likelihood field (precomputed) is used to get probabilities for each scan point.
        - The particle weight is updated based on the log likelihood of all valid scan points.
        """
        # Transform the particle's pose into the map frame
        T = np.matmul(self.__poseToTranslationMatrix(), laser_to_ego_transformation)

        # Convert laser scan data to Cartesian coordinates
        _, scanCartesianHomo = convertScanToCartesian(scanOutput)
        scanInMap = np.dot(T, scanCartesianHomo.T).T

        # Retrieve likelihood field from the map
        likelihoodField = mapManipulatorInstance.getLikelihoodField()
        
        # Convert scan points to cell indices in the occupancy grid 
        cellPositions = mapManipulatorInstance.position_2_cell(
            scanInMap[:, 0:2])

        # Get the map dimensions
        lm_x, lm_y = likelihoodField.shape

        # Filter out scan points that fall outside the map boundaries
        cellPositions = cellPositions[np.logical_and.reduce(
                (cellPositions[:, 0] > 0, -cellPositions[:, 1] > 0, cellPositions[:, 0] < lm_y,  -cellPositions[:, 1] < lm_x))]

        # Compute log likelihood from the precomputed likelihood field
        log_weights = np.log(
            likelihoodField[-cellPositions[:, 1], cellPositions[:, 0]])
        log_weight = np.sum(log_weights)

        # Convert log weight back to regular probability scale
        weight = np.exp(log_weight)
        weight += 1e-10

        # Update particle's weight
        self.setWeight(weight)

    def setWeight(self, weight):
        self.weight = weight

    def getWeight(self):
        return self.weight

    def setPose(self, pose):
        self.pose = pose

    def getPose(self):
        return self.pose[0], self.pose[1], self.pose[2]

    def __poseToTranslationMatrix(self):
        x, y, th = self.getPose()

        translation = np.array([[cos(th), -sin(th), x],
                                [sin(th), cos(th), y],
                                [0, 0, 1]])

        return translation
