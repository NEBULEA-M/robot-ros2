run the following message to indicate the battery is at full charge.

```sh
ros2 topic pub /battery_status sensor_msgs/BatteryState '{voltage: 9.0, percentage: 1.0, power_supply_status: 3}' 
```

Open a new terminal and launch the robot.

```sh
ros2 launch two_wheeled_robot hospital_world_connect_to_charging_dock.launch.py
```

Select the **Nav2 Goal** button at the top of RViz and click somewhere on the map to command the robot to navigate to any reachable goal location.

While the robot is moving, stop the /battery_status publisher.
```sh
CTRL + C
```


Now run this command to indicate low battery:
```sh
ros2 topic pub /battery_status sensor_msgs/BatteryState '{voltage: 2.16, percentage: 0.24, power_supply_status: 3}' 
```

The robot will plan a path to the staging area and then move along that path.

Once the robot reaches the staging area, the robot will spin indefinitely. This spin, in a real-world application, would be navigating to a charging dock with the appropriate algorithm (using an infrared sensor, ARTag, AprilTag, etc.).

Let’s assume the robot has now reached the charging dock and is charging.

Press CTRL + C to stop the /battery_status publisher, and type:
```sh
ros2 topic pub /battery_status sensor_msgs/BatteryState '{voltage: 2.16, percentage: 0.24, power_supply_status: 1}' 
```
That 1 for the power_supply_status indicates the battery is CHARGING.