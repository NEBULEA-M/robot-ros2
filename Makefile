.PHONY: all build run clean

build:
	colcon build --packages-select two_wheeled_robot

run:
	. ./install/setup.sh ; \
	export LIBGL_ALWAYS_SOFTWARE=1 ; \
	source /usr/share/gazebo/setup.sh ; \
	ros2 launch two_wheeled_robot hospital_world_connect_to_charging_dock.launch.py

run_charging_dock: run
	ros2 launch two_wheeled_robot hospital_world_connect_to_charging_dock.launch.py


clean:
	rm -rf build install log
