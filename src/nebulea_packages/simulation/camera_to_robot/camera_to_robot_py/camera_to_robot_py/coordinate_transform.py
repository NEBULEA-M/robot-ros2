''' ####################
    Convert camera coordinates to robot base frame coordinates
    #################### '''

import numpy as np
import random  # Python library to generate random numbers


class CoordinateConversion(object):
    """
  Parent class for coordinate conversions
  All child classes must implement the convert function.
  Every class in Python is descended from the object class
  class CoordinateConversion == class CoordinateConversion(object)
  This class is a superclass, a general class from which 
  more specialized classes (e.g. CameraToRobotBaseConversion) can be defined.
  """

    def convert(self, frame_coordinates):
        """
    Convert between coordinate frames
 
    Input
      :param frame_coordinates: Coordinates of the object in a reference frame (x, y, z, 1)
 
    Output
      :return: new_frame_coordinates: Coordinates of the object in a new reference frame (x, y, z, 1)
 
    """
        # Any subclasses that inherit this superclass CoordinateConversion must implement this method.
        raise NotImplementedError


class CameraToRobotBaseConversion(CoordinateConversion):
    """
  Convert camera coordinates to robot base frame coordinates
  This class is a subclass that inherits the methods from the CoordinateConversion class.
  """

    def __init__(self, rot_angle, x_disp, y_disp, z_disp):
        """
    Constructor for the CameraToRobotBaseConversion class. Sets the properties.
 
    Input
      :param rot_angle: Angle between axes in degrees
      :param x_disp: Displacement between coordinate frames in the x direction in centimeters
      :param y_disp: Displacement between coordinate frames in the y direction in centimeters
      :param z_disp: Displacement between coordinate frames in the z direction in centimeters
 
    """
        self.angle = np.deg2rad(rot_angle)  # Convert degrees to radians
        self.X = x_disp
        self.Y = y_disp
        self.Z = z_disp

    def convert(self, frame_coordinates):
        """
    Convert camera coordinates to robot base frame coordinates
 
    Input
      :param frame_coordinates: Coordinates of the object in the camera reference frame (x, y, z, 1) in centimeters
 
    Output
      :return: new_frame_coordinates: Coordinates of the object in the robot base reference frame (x, y, z, 1) in centimeters
 
    """
        # Define the rotation matrix from the robotic base frame (frame 0)
        # to the camera frame (frame c).
        rot_mat_0_c = np.array([[1, 0, 0],
                                [0, np.cos(self.angle), -np.sin(self.angle)],
                                [0, np.sin(self.angle), np.cos(self.angle)]])

        # Define the displacement vector from frame 0 to frame c
        disp_vec_0_c = np.array([[self.X],
                                 [self.Y],
                                 [self.Z]])

        # Row vector for bottom of homogeneous transformation matrix
        extra_row_homgen = np.array([[0, 0, 0, 1]])

        # Create the homogeneous transformation matrix from frame 0 to frame c
        homgen_0_c = np.concatenate((rot_mat_0_c, disp_vec_0_c), axis=1)  # side by side
        homgen_0_c = np.concatenate((homgen_0_c, extra_row_homgen), axis=0)  # one above the other

        # Coordinates of the object in base reference frame
        new_frame_coordinates = homgen_0_c @ frame_coordinates

        return new_frame_coordinates


def main():
    """
  This code is used to test the methods implemented above.
  """

    # Define the displacement from frame base frame of robot to camera frame in centimeters
    x_disp = -17.8
    y_disp = 24.4
    z_disp = 0.0
    rot_angle = 180  # angle between axes in degrees

    # Create a CameraToRobotBaseConversion object
    cam_to_robo = CameraToRobotBaseConversion(rot_angle, x_disp, y_disp, z_disp)

    # Centimeter to pixel conversion factor
    # I measured 36.0 cm across the width of the field of view of the camera.
    CM_TO_PIXEL = 36.0 / 640

    print(f'Detecting an object for 3 seconds')
    dt = 0.1  # Time interval
    t = 0  # Set starting time
    while t < 3:
        t = t + dt

        # This is in pixel coordinates.
        # Since we don't have an actual camera and an object to detect,
        # we generate random pixel locations.
        # Assume x (width) can go from 0 to 640 pixels, and y (height) can go from 0 to 480 pixels
        x = random.randint(250, 450)  # Generate a random integer from 250 to 450 (inclusive)
        y = random.randint(250, 450)  # Generate a random integer from 250 to 450 (inclusive)

        # Calculate the center of the object in centimeter coordinates
        # instead of pixel coordinates
        x_cm = x * CM_TO_PIXEL
        y_cm = y * CM_TO_PIXEL

        # Coordinates of the object in the camera reference frame
        cam_ref_coord = np.array([[x_cm],
                                  [y_cm],
                                  [0.0],
                                  [1]])

        robot_base_frame_coord = cam_to_robo.convert(cam_ref_coord)  # Return robot base frame coordinates

        text = "x: " + str(robot_base_frame_coord[0][0]) + ", y: " + str(robot_base_frame_coord[1][0])
        print(f'{t}:{text}')  # Display time stamp and coordinates


if __name__ == '__main__':
    main()
