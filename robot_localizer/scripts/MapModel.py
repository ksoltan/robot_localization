from occupancy_field import OccupancyField
from math import cos, sin, radians
import random

class MapModel(object):
    def __init__(self):
        self.occupancy_field = OccupancyField()

    '''
    Function: get_valid_point
    Inputs:

    Generate a random point from the map.

    '''
    def get_valid_point(self):
        #TODO: Use the pose estimate function, like in the example. This may need to move out of map model
        # x axis is width
        max_width = self.occupancy_field.map.info.width * self.occupancy_field.map.info.resolution
        # y axis is height
        max_height = self.occupancy_field.map.info.height * self.occupancy_field.map.info.resolution
        return (random.uniform(0, 2), random.uniform(0, 2))

    '''
    Function: get_predicted_obstacle_error
    Inputs: int distance_reading (measurement from the laser scan)
            float x: (map frame)
            float y: (map frame)
            int angle: degrees (map frame) (must have accounted for robot's yaw)

    Use a laser scan reading to project the theoretical position of where an obstacle
    should be in a direction (angle).
    Return the distance to the closest obstacle from this position on the given map.
    This distance corresponds to the error between the predicted obstacle position
    and its location on the map given a hypothetical robot position.
    A smaller error means the reading is more likely from the given position

    '''
    def get_predicted_obstacle_error(self, distance_reading, x, y, angle):
        # Predict location of objects based on laser scan reading.
        predicted_obstacle_x, predicted_obstacle_y = self.move_coordinate(
                                                    x, y, angle, distance_reading)

        # Find the closest obstacle to the predicted obstacle position
        predicted_reading = self.occupancy_field.get_closest_obstacle_distance(
                                    predicted_obstacle_x, predicted_obstacle_y)
        print("Predicted x: {}, y: {}, reading: {}".format(predicted_obstacle_x, predicted_obstacle_y, predicted_reading))
        return predicted_reading

    '''
    Function: move_coordinate
    Inputs: float x
            float y
            int angle: degrees
            int distance

    Return a point which is the given distance away from (x, y) in the angle direction.

    '''
    def move_coordinate(self, x, y, angle, distance):
        return (x + cos(radians(angle)) * distance, y + sin(radians(angle)) * distance)
