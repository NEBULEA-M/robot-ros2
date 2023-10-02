''' ####################
    Detect an object in a video stream and publish its coordinates
    from the perspective of the camera (i.e. camera reference frame)
    Note: You don't need a camera to run this node. This node just demonstrates how to create
    a publisher node in ROS2 (i.e. how to publish data to a topic in ROS2).
    -------
    Publish the coordinates of the centroid of an object to a topic:
      /pos_in_cam_frame – The position of the center of an object in centimeter coordinates
    #################### '''

import rclpy  # Import the ROS client library for Python
from rclpy.node import Node  # Enables the use of rclpy's Node class
from std_msgs.msg import Float64MultiArray  # Enable use of the std_msgs/Float64MultiArray message type
import numpy as np  # NumPy Python library
import random  # Python library to generate random numbers


class CameraPublisher(Node):
    """
  Create a CameraPublisher class, which is a subclass of the Node class.
  The class publishes the position of an object every 3 seconds.
  The position of the object are the x and y coordinates with respect to 
  the camera frame.
  """

    def __init__(self):
        """
    Class constructor to set up the node
    """

        # Initiate the Node class's constructor and give it a name
        super().__init__('camera_publisher')

        # Create publisher(s)

        # This node publishes the position of an object every 3 seconds.
        # Maximum queue size of 10.
        self.publisher_position_cam_frame = self.create_publisher(Float64MultiArray, '/pos_in_cam_frame', 10)

        # 3 seconds
        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.get_coordinates_of_object)
        self.i = 0  # Initialize a counter variable

        # Centimeter to pixel conversion factor
        # Assume we measure 36.0 cm across the width of the field of view of the camera.
        # Assume camera is 640 pixels in width and 480 pixels in height
        self.CM_TO_PIXEL = 36.0 / 640

    def get_coordinates_of_object(self):
        """
    Callback function.
    This function gets called every 3 seconds.
    We locate an object using the camera and then publish its coordinates to ROS2 topics.
    """
        # Center of the bounding box that encloses the detected object.
        # This is in pixel coordinates.
        # Since we don't have an actual camera and an object to detect,
        # we generate random pixel locations.
        # Assume x (width) can go from 0 to 640 pixels, and y (height) can go from 0 to 480 pixels
        x = random.randint(250, 450)  # Generate a random integer from 250 to 450 (inclusive)
        y = random.randint(250, 450)  # Generate a random integer from 250 to 450 (inclusive)

        # Calculate the center of the object in centimeter coordinates
        # instead of pixel coordinates
        x_cm = x * self.CM_TO_PIXEL
        y_cm = y * self.CM_TO_PIXEL

        # Store the position of the object in a NumPy array
        object_position = [x_cm, y_cm]

        # Publish the coordinates to the topic
        self.publish_coordinates(object_position)

        # Increment counter variable
        self.i += 1

    def publish_coordinates(self, position):
        """
    Publish the coordinates of the object to ROS2 topics
    :param: The position of the object in centimeter coordinates [x , y] 
    """
        msg = Float64MultiArray()  # Create a message of this type
        msg.data = position  # Store the x and y coordinates of the object
        self.publisher_position_cam_frame.publish(msg)  # Publish the position to the topic


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    camera_publisher = CameraPublisher()

    # Spin the node so the callback function is called.
    # Publish any pending messages to the topics.
    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy  # Import the ROS client library for Python
from rclpy.node import Node  # Enables the use of rclpy's Node class
from std_msgs.msg import Float64MultiArray  # Enable use of the std_msgs/Float64MultiArray message type
import numpy as np  # NumPy Python library
import random  # Python library to generate random numbers


class CameraPublisher(Node):
    """
  Create a CameraPublisher class, which is a subclass of the Node class.
  The class publishes the position of an object every 3 seconds.
  The position of the object are the x and y coordinates with respect to 
  the camera frame.
  """

    def __init__(self):
        """
    Class constructor to set up the node
    """

        # Initiate the Node class's constructor and give it a name
        super().__init__('camera_publisher')

        # Create publisher(s)

        # This node publishes the position of an object every 3 seconds.
        # Maximum queue size of 10.
        self.publisher_position_cam_frame = self.create_publisher(Float64MultiArray, '/pos_in_cam_frame', 10)

        # 3 seconds
        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.get_coordinates_of_object)
        self.i = 0  # Initialize a counter variable

        # Centimeter to pixel conversion factor
        # Assume we measure 36.0 cm across the width of the field of view of the camera.
        # Assume camera is 640 pixels in width and 480 pixels in height
        self.CM_TO_PIXEL = 36.0 / 640

    def get_coordinates_of_object(self):
        """
    Callback function.
    This function gets called every 3 seconds.
    We locate an object using the camera and then publish its coordinates to ROS2 topics.
    """
        # Center of the bounding box that encloses the detected object.
        # This is in pixel coordinates.
        # Since we don't have an actual camera and an object to detect,
        # we generate random pixel locations.
        # Assume x (width) can go from 0 to 640 pixels, and y (height) can go from 0 to 480 pixels
        x = random.randint(250, 450)  # Generate a random integer from 250 to 450 (inclusive)
        y = random.randint(250, 450)  # Generate a random integer from 250 to 450 (inclusive)

        # Calculate the center of the object in centimeter coordinates
        # instead of pixel coordinates
        x_cm = x * self.CM_TO_PIXEL
        y_cm = y * self.CM_TO_PIXEL

        # Store the position of the object in a NumPy array
        object_position = [x_cm, y_cm]

        # Publish the coordinates to the topic
        self.publish_coordinates(object_position)

        # Increment counter variable
        self.i += 1

    def publish_coordinates(self, position):
        """
    Publish the coordinates of the object to ROS2 topics
    :param: The position of the object in centimeter coordinates [x , y] 
    """
        msg = Float64MultiArray()  # Create a message of this type
        msg.data = position  # Store the x and y coordinates of the object
        self.publisher_position_cam_frame.publish(msg)  # Publish the position to the topic


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    camera_publisher = CameraPublisher()

    # Spin the node so the callback function is called.
    # Publish any pending messages to the topics.
    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()