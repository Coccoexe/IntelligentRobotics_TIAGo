#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <group39_hw1/MoveAction.h>
int main (int argc, char **argv)
{
    ros::init(argc, argv, "client");
    actionlib::SimpleActionClient<group39_hw1::MoveAction> ac("movement", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");
    group39_hw1::MoveGoal goal;
    goal.x = atof(argv[1]);
    goal.y = atof(argv[2]);
    goal.z = atof(argv[3]);
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
    return 0;
}