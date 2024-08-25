# This makefile allows to prepare ros enviromnent for Miabot RPi and VM (or PC)
# Make sure all your systems are ubuntu 22.04 LTS 

MAKEFLAGS += --no-print-directory

# to verify UBINTU_CODENAME out of makefile use `. /etc/os-release && echo $UBUNTU_CODENAME`
UBUNTU_CODE = jammy
ARCH = `dpkg --print-architecture`
SUDO_PASSWORD = 12345678

ROS2_WORKSPACE = ./ros2_workspace/
PROJECT_ROOT = ./

ROS_DISTRO=humble
NODES = nodes/
NODE_MIABOT = miabot_node/
NODE_EXPLORATION = exploration_algorithm/
NODE_PROJECT_BRINGUP = project_bringup/

SRC = src/
RESULTS = results/


.SILENT: perpare_robot prepare_pc prepare_ros2_workspace test

test:
	echo $(USER)
	echo $(ROS2_WORKSPACE)
	echo $(PROJECT_ROOT)
	echo $(UBUNTU_CODE)
	echo $(ARCH)
	echo $$(date +%s)

add_serial_port_privileges:
	echo $(SUDO_PASSWORD) | sudo -S chown $(USER) /dev/ttyACM0 /dev/ttyS0

install_ros2_humble:
	echo $(SUDO_PASSWORD) | sudo -S apt install software-properties-common -y
	echo $(SUDO_PASSWORD) | sudo -S add-apt-repository universe -y
	echo $(SUDO_PASSWORD) | sudo -S apt install curl -y
	echo $(SUDO_PASSWORD) | sudo -S curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(ARCH) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(UBUNTU_CODE) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	echo $(SUDO_PASSWORD) | sudo -S apt update && sudo apt upgrade -y && sudo apt autoremove
	echo $(SUDO_PASSWORD) | sudo -S apt install ros-humble-ros-base ros-dev-tools -y
	echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
	echo "ROS_DISTRO=$(ROS_DISTRO)" >> ~/.bashrc

install_ros2_nodes:
	echo $(SUDO_PASSWORD) | sudo -S apt update
	echo $(SUDO_PASSWORD) | sudo -S apt install -y ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp ros-humble-tf-transformations ros-humble-rqt ros-humble-turtlebot3*

prepare_python_dependencies:
	echo $(SUDO_PASSWORD) | sudo -S apt install python3-pip
	pip3 install setuptools==58.2.0 transforms3d serial pyserial

prepare_ros2_workspace:
	mkdir -p $(ROS2_WORKSPACE)$(SRC)
	mkdir -p $(ROS2_WORKSPACE)$(RESULTS)
	$(MAKE) prepare_urg2_node
	$(MAKE) copy_nodes
	$(MAKE) build_ros2_workspace

prepare_urg2_node:
	cd $(ROS2_WORKSPACE)$(SRC); \
	git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git; \
	echo $(SUDO_PASSWORD) | sudo -S rosdep init; \
	cat ../../nodes/config/urg_node2.launch.py > urg_node2/launch/urg_node2.launch.py; \
	cat ../../nodes/config/params_serial.yaml > urg_node2/config/params_serial.yaml; \
	rosdep update; \
	rosdep install -i --from-paths urg_node2

copy_nodes:
	cp -r $(PROJECT_ROOT)$(NODES)$(NODE_PROJECT_BRINGUP) $(PROJECT_ROOT)$(NODES)$(NODE_MIABOT) $(PROJECT_ROOT)$(NODES)$(NODE_EXPLORATION) $(ROS2_WORKSPACE)$(SRC)

build_ros2_workspace:
	cd $(ROS2_WORKSPACE); \
	colcon build --symlink-install

remove_ros2_workspace:
	rm -rf $(ROS2_WORKSPACE)


# these steps manually :)

# run_robot_nodes:
# 	cd $(ROS2_WORKSPACE)
# 	source install/setup.bash
# 	ros2 launch project_bringup robot_bringup.launch.py

# run_pc_nodes:
# 	cd $(ROS2_WORKSPACE)
# 	source install/setup.bash
# 	ros2 launch project_bringup pc_bringup.launch.py
#	ros2 launch nav2_bringup rviz_launch.py
#	ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file src/project_bringup/config/slam.yaml
#	ros2 launch nav2_bringup navigation_launch.py params_file:="src/project_bringup/config/nav2_params.yaml"

prepare_robot:
	# $(MAKE) install_ros2_humble
	$(MAKE) prepare_ros2_workspace

before_ros_run_robot:
	$(MAKE) add_serial_port_privilege

prepare_pc:
	# $(MAKE) install_ros2_humble
	$(MAKE) install_ros2_nodes
	$(MAKE) prepare_ros2_workspace

# tools
view_frames:
	cd $(ROS2_WORKSPACE)$(RESULTS); \
	ros2 run tf2_tools view_frames

save_map:
	ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: '$(RESULTS)map_$$(date +%s)'}"
	echo $$?
