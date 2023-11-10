[Create a tf Listener Using ROS 2](https://automaticaddison.com/how-to-create-a-tf-listener-using-ros-2-and-python/)

pip3 install transforms3d
pip3 install numpy

ros2 run tf2_ros tf2_echo map base_link

The syntax is:

```sh 
ros2 run tf2_ros tf2_echo [parent_frame] [child_frame]
```

Launch a robot

```sh 
ros2 launch two_wheeled_robot hospital_world_connect_to_charging_dock.launch.py
```

Open another terminal and run the transform listener.

```sh 
ros2 run two_wheeled_robot map_to_base_link_transform.py
```
The command above does the base_link -> map transform. If you want to see another transform (e.g. lidar_link -> map), you can type the following command:

```sh 
ros2 run two_wheeled_robot map_to_base_link_transform.py --ros-args -p target_frame:='lidar_link'
```

Open a new terminal window, and observe the data. 
```sh 
ros2 topic echo /map_to_base_link_pose2d
```

Here we can see the pose of the robot (i.e. base_link coordinate frame) with respect to the map frame in x, y, yaw (i.e. Euler angle) format…where x and y are in meters, and yaw is in radians.

