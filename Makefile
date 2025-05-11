# This makefile allows to prepare ros enviromnent for Miabot RPi and VM (or PC)
# Make sure all your systems are ubuntu 22.04 LTS 

MAKEFLAGS += --no-print-directory

# to verify UBINTU_CODENAME out of makefile use `. /etc/os-release && echo $UBUNTU_CODENAME`
UBUNTU_CODE = jammy
ARCH = `dpkg --print-architecture`
SUDO_PASSWORD = 12345678

ROS2_WORKSPACE = ./ros2_workspace/
PROJECT_ROOT = ./

REQUIREMENTS = requirements

ROS_DISTRO=humble
NODES = nodes/
NODE_MIABOT = miabot_node/
NODE_EXPLORATION = exploration_algorithm/
NODE_PROJECT_BRINGUP = project_bringup/
CONFIG = config/

NODE_MAP_JSON = map_json_node/
NODE_MAP_PEDICTOR = map_predictor/
NODE_TIME_PREDICTOR = time_predictor/

SRC = src/
RESULTS = results/
CACHE = cache/

DOCKER_DEVELOPMENT = docker_development/
VSCODE = .vscode/

.SILENT: perpare_robot prepare_pc ros2_prepare_workspace test

test:
	@echo user:         $(USER)
	@echo workspace:    $(ROS2_WORKSPACE)
	@echo project root: $(PROJECT_ROOT)
	@echo ubuntu name:  $(UBUNTU_CODE)
	@echo architecture: $(ARCH)
	@echo current ts:   $$(date +%s)

serial_port_add_privileges:
	@echo $(SUDO_PASSWORD) | sudo -S chown $(USER) /dev/ttyACM0 /dev/ttyS0

ros2_install_humble:
	@echo $(SUDO_PASSWORD) | sudo -S apt install software-properties-common -y
	@echo $(SUDO_PASSWORD) | sudo -S add-apt-repository universe -y
	@echo $(SUDO_PASSWORD) | sudo -S apt install curl -y
	@echo $(SUDO_PASSWORD) | sudo -S curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	@echo "deb [arch=$(ARCH) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(UBUNTU_CODE) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	@echo $(SUDO_PASSWORD) | sudo -S apt update && sudo apt upgrade -y && sudo apt autoremove
	@echo $(SUDO_PASSWORD) | sudo -S apt install ros-humble-ros-base ros-dev-tools -y
	@echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
	@echo "ROS_DISTRO=$(ROS_DISTRO)" >> ~/.bashrc

ros2_install_nodes:
	@echo $(SUDO_PASSWORD) | sudo -S apt update
	@echo $(SUDO_PASSWORD) | sudo -S apt install -y ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp ros-humble-tf-transformations ros-humble-rqt ros-humble-turtlebot3*

python_prepare_dependencies:
	@echo $(SUDO_PASSWORD) | sudo -S apt install python3-pip python3-venv -y
	pip3 install setuptools==58.2.0 transforms3d serial pyserial

python_prepare_venv:
	python3 -m venv $(ROS2_WORKSPACE)venv/
	@echo "make sure to activate venv using source ./venv/bin/activate"

python_install_packages:
	pip install --upgrade pip
	pip install -r $(REQUIREMENTS)

ros2_prepare_workspace:
	mkdir -p $(ROS2_WORKSPACE)$(SRC)
	mkdir -p $(ROS2_WORKSPACE)$(RESULTS)
	$(MAKE) ros2_prepare_urg2_node
	$(MAKE) ros2_prepare_frontier_base_exploration_algorithm
	$(MAKE) ros2_copy_nodes
	$(MAKE) ros2_build_workspace

ros2_prepare_urg2_node:
	cd $(ROS2_WORKSPACE)$(SRC); \
	git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git; \
	@echo $(SUDO_PASSWORD) | sudo -S rosdep init; \
	cat ../../nodes/config/urg_node2.launch.py > urg_node2/launch/urg_node2.launch.py; \
	cat ../../nodes/config/params_serial.yaml > urg_node2/config/params_serial.yaml; \
	rosdep update; \
	rosdep install -i --from-paths urg_node2

ros2_prepare_frontier_base_exploration_algorithm:
	mkdir -p temporary/; \
	git clone --recursive https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot.git temporary/; \
	cp -r temporary/autonomous_exploration/ $(ROS2_WORKSPACE)$(SRC)
	rm -rf temporary/

ros2_copy_nodes:
	cp -r $(PROJECT_ROOT)$(NODES)$(NODE_PROJECT_BRINGUP) $(PROJECT_ROOT)$(NODES)$(NODE_MIABOT) $(PROJECT_ROOT)$(NODES)$(NODE_EXPLORATION) $(PROJECT_ROOT)$(NODES)$(NODE_MAP_JSON) $(PROJECT_ROOT)$(NODES)$(NODE_MAP_PEDICTOR) $(PROJECT_ROOT)$(NODES)$(NODE_TIME_PREDICTOR) $(PROJECT_ROOT)$(NODES)$(CONFIG) $(ROS2_WORKSPACE)$(SRC)

ros2_build_workspace:
	cd $(ROS2_WORKSPACE); \
	colcon build --symlink-install

ros2_remove_workspace:
	rm -rf $(ROS2_WORKSPACE)

prepare_robot:
	$(MAKE) ros2_install_humble
	$(MAKE) ros2_prepare_workspace

prepare_pc:
	$(MAKE) ros2_install_humble
	$(MAKE) ros2_install_nodes
	$(MAKE) ros2_prepare_workspace

# these steps manually :)

# run_robot_nodes:
# 	cd $(ROS2_WORKSPACE)
# 	source install/setup.bash
# 	ros2 launch project_bringup robot_bringup.launch.py

# run_pc_nodes:
# 	cd $(ROS2_WORKSPACE)
# 	source install/setup.bash
#	export TURTLEBOT3_MODEL=waffle && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# 	ros2 launch project_bringup pc_bringup.launch.py
#	ros2 launch nav2_bringup rviz_launch.py
#	ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file src/config/simulation/slam.yaml (or robot)
#	ros2 launch nav2_bringup navigation_launch.py params_file:="src/config/simulation/nav2_params.yaml" (or robot)
#	ros2 run exploration_algorithm random_direction_node


# tools
view_frames:
	cd $(ROS2_WORKSPACE)$(RESULTS); \
	ros2 run tf2_tools view_frames

save_map:
	ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: '$(RESULTS)map_$$(date +%s)'}"
	@echo $$?


# docker - https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html#id7
# indexing ROS2 in vs code - https://robotics.stackexchange.com/questions/113024/import-rclpy-could-not-be-resolved-pylance-reportmissingimports
docker_init_devcontainer:
	mkdir -p $(ROS2_WORKSPACE)$(SRC) $(ROS2_WORKSPACE)/.devcontainer
	mkdir -p $(ROS2_WORKSPACE)$(CACHE)$(ROS_DISTRO)/build $(ROS2_WORKSPACE)$(CACHE)$(ROS_DISTRO)/install $(ROS2_WORKSPACE)$(CACHE)$(ROS_DISTRO)/log

	cp $(DOCKER_DEVELOPMENT)devcontainer.json $(DOCKER_DEVELOPMENT)Dockerfile $(ROS2_WORKSPACE).devcontainer/

	cp Makefile $(ROS2_WORKSPACE)

	mkdir -p $(ROS2_WORKSPACE)$(VSCODE)
	cp $(DOCKER_DEVELOPMENT)settings.json $(ROS2_WORKSPACE)$(VSCODE)

	$(MAKE) ros2_copy_nodes
	cp $(REQUIREMENTS) $(ROS2_WORKSPACE)

	@echo "Remember to source source /opt/ros/$(ROS_DISTRO)/setup.bash before running docker setup"

docker_setup_devcontainer:
	@if [ "$(ROS2_WORKSPACE)" = "$(PROJECT_ROOT)" ]; then \
		echo "ROS2_WORKSPACE is equal to PROJECT_ROOT. Check passed"; \
	else \
		echo "ROS2_WORKSPACE is not equal to PROJECT_ROOT. Use only in docker workspace. Exiting."; \
		exit 1; \
	fi

	$(MAKE) python_prepare_dependencies
	$(MAKE) python_install_packages

	$(MAKE) ros2_install_nodes
	$(MAKE) ros2_prepare_frontier_base_exploration_algorithm
	$(MAKE) ros2_build_workspace

# node runners
run_reminder:
	@echo "make sure to run source install/setup.bash before that"

run_node_quick_simulation:
	$(MAKE) run_reminder
	export TURTLEBOT3_MODEL=waffle && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py & \
	ros2 launch project_bringup pc_bringup.launch.py & \
	ros2 launch nav2_bringup rviz_launch.py & \
	ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file src/config/simulation/slam.yaml & \
	ros2 launch nav2_bringup navigation_launch.py params_file:="src/config/simulation/nav2_params.yaml"

run_node_map_json_node:
	$(MAKE) run_reminder
	ros2 run map_json_node map_json_node

run_node_map_predictor:
	$(MAKE) run_reminder
	ros2 run map_predictor map_predictor
