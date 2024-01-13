#!/bin/bash
gnome-terminal --title="Simulation" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables"
sleep 8
gnome-terminal --title="AprilTag" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; roslaunch tiago_iaslab_simulation apriltag.launch"
sleep 5
gnome-terminal --title="Navigation" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; roslaunch tiago_iaslab_simulation navigation.launch"
sleep 5
gnome-terminal --title="Human Node" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; rosrun tiago_iaslab_simulation human_node"
sleep 5
gnome-terminal --title="Server" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; rosrun group39_hw1 server"
sleep 5
gnome-terminal --title="Detection" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; rosrun group39_hw2 detection"
sleep 5
gnome-terminal --title="Manipulation" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; rosrun group39_hw2 manipulation"
sleep 10
gnome-terminal --title="Brain" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; rosrun group39_hw2 brain"
#sleep 5
#gnome-terminal --title="DEBUG" -- bash -c "cd ~/catkin_ws; source devel/setup.bash; rosrun look_to_point look_to_point"