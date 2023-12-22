catkin build
source ~/catkin_ws/devel/setup.bash
roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library

altro terminale:
source ~/catkin_ws/devel/setup.bash
roslaunch tiago_iaslab_simulation navigation.launch

altro terminale: runnare nostro codice
source ~/catkin_ws/devel/setup.bash
rosrun group39_hw1 movement_node