# TEMPORARY

clear all cache: `rm -rf build/ log/ install/`
build only exploration node: `colcon build --packages-select exploration_algorithm`
run exploration: `ros2 run  exploration_algorithm exploration_algorithm_node`
