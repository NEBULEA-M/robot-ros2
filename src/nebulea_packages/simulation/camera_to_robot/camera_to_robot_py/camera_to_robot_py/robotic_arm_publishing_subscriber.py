''' ####################
    Receive (i.e. subscribe) coordinates of an object in the camera reference frame, and
    publish those coordinates in the robotic arm base frame.
    -------
    Publish the coordinates of the centroid of an object to topic:
      /pos_in_robot_base_frame – The x and y position of the center of an object in centimeter coordinates
    #################### '''

import rclpy  # Import the ROS client library for Python
from rclpy.node import Node  # Enables the use of rclpy's Node class
from std_msgs.msg import Float64MultiArray  # Enable use of the std_msgs/Float64MultiArray message type
import numpy as np  # NumPy Python library
from .coordinate_transform import CameraToRobotBaseConversion


class PublishingSubscriber(Node):
    """
  Create a PublishingSubscriber class, which is a subclass of the Node class.
  This class subscribes to the x and y coordinates of an object in the camera reference frame.
  The class then publishes the x and y coordinates of the object in the robot base frame.  
  """

    def __init__(self):
        """
    Class constructor to set up the node
    """

        # Initiate the Node class's constructor and give it a name
        super().__init__('publishing_subscriber')

        # Create subscriber(s)

        # The node subscribes to messages of type std_msgs/Float64MultiArray, over a topic named:
        #   /pos_in_cam_frame
        # The callback function is called as soon as a message is received.
        # The maximum number of queued messages is 10.
        self.subscription_1 = self.create_subscription(
            Float64MultiArray,
            '/pos_in_cam_frame',
            self.pos_received,
            10)
        self.subscription_1  # prevent unused variable warning

        # Create publisher(s)

        # This node publishes the position in robot frame coordinates.
        # Maximum queue size of 10.
        self.publisher_pos_robot_frame = self.create_publisher(Float64MultiArray, '/pos_in_robot_base_frame', 10)

        # Define the displacement from frame base frame of robot to camera frame in centimeters
        x_disp = -17.8
        y_disp = 24.4
        z_disp = 0.0
        rot_angle = 180  # angle between axes in degrees

        # Create a CameraToRobotBaseConversion object
        self.cam_to_robo = CameraToRobotBaseConversion(rot_angle, x_disp, y_disp, z_disp)

    def pos_received(self, msg):
        """
    Callback function.
    This function gets called as soon as the position of the object is received.
    :param: msg is of type std_msgs/Float64MultiArray 
    """
        object_position = msg.data

        # Coordinates of the object in the camera reference frame in centimeters
        cam_ref_coord = np.array([[object_position[0]],
                                  [object_position[1]],
                                  [0.0],
                                  [1]])

        robot_base_frame_coord = self.cam_to_robo.convert(cam_ref_coord)  # Return robot base frame coordinates

        # Capture the object's desired position (x, y)
        object_position = [robot_base_frame_coord[0][0], robot_base_frame_coord[1][0]]

        # Publish the coordinates to the topics
        self.publish_position(object_position)

    def publish_position(self, object_position):
        """
    Publish the coordinates of the object with respect to the robot base frame.
    :param: object position [x, y] 
    """
        msg = Float64MultiArray()  # Create a message of this type
        msg.data = object_position  # Store the object's position
        self.publisher_pos_robot_frame.publish(msg)  # Publish the position to the topic


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    publishing_subscriber = PublishingSubscriber()

    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    # Publish any pending messages to the topics.
    rclpy.spin(publishing_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publishing_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
