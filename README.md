Install dependent ROS2 packages from [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

Gazebo
```sh
sudo apt install ros-humble-gazebo-*
```

Cartographer
```sh
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```

Navigation2
```sh
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

Refer to [ros2_rust](https://github.com/ros2-rust/ros2_rust)

Install vcstool
```sh
sudo apt install python3-vcstool
```
or when that is not possible, fall back to pip:
```sh
sudo pip install -U vcstool
```

Install package for rust in ROS2
```sh
sudo apt install -y git libclang-dev python3-pip python3-vcstool # libclang-dev is required by bindgen
# Install these plugins for cargo and colcon:
cargo install --debug cargo-ament-build  # --debug is faster to install
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```
Change directory to ros2_rust, then run
```sh
vcs import src < src/ros2_rust/ros2_rust_humble.repos
. /opt/ros/humble/setup.sh
colcon build
```

Install rust packages for ros2
```sh
. ./install/setup.sh. ./install/setup.sh
```

sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations


https://automaticaddison.com/go-to-a-goal-location-upon-low-battery-ros-2-navigation/

sudo apt-get install ros-humble-robot-localization