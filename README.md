# GROUP 39
- Alessio Cocco 		    alessio.cocco@studenti.unipd.it
- Andrea Valentinuzzi 		andrea.valentinuzzi@studenti.unipd.it
- Giovanni Brejc 		    giovanni.brejc@studenti.unipd.it

This repository contains the project folder and files like the readme and gitignore.

## Instructions HW1
#### Terminal 1:
	catkin build
	source ~/catkin_ws/devel/setup.bash
	roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library

#### Terminal 2:
	source ~/catkin_ws/devel/setup.bash
	roslaunch tiago_iaslab_simulation navigation.launch

#### Terminal 3: (server)
	source ~/catkin_ws/devel/setup.bash
	rosrun group39_hw1 server

#### Terminal 4: (client)
	source ~/catkin_ws/devel/setup.bash
	rosrun group39_hw1 client
	
#### Note that the client can be runned wihtout any command or with Pose_B coordinates (examples below):
	rosrun group39_hw1 client
	rosrun group39_hw1 client 10 0 90

## Instruction HW2
We provide a run_all.sh to run all following console instructions, in alternative we list here the detail to run it independently:
Note: if you run the run_all file make sure to do a catkin build and a source before.

#### Terminal 1:
	catkin build
	source ~/catkin_ws/devel/setup.bash
	roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables
#### Terminal2:
	roslaunch tiago_iaslab_simulation apriltag.launch
#### Terminal 3:
	roslaunch tiago_iaslab_simulation navigation.launch
#### Terminal 4:
	rosrun tiago_iaslab_simulation human_node
#### Terminal 5:
    rosrun group39_hw1 server
#### Terminal 6:
    rosrun group39_hw2 detection
#### Terminal 7:
	rosrun group39_hw2 manipulation
#### Terminal 8:
    rosrun group39_hw2 brain