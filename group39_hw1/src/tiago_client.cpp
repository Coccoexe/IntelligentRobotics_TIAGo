// Authors: Group 39 (Alessio Cocco, Andrea Valentinuzzi, Giovanni Brejc)

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <group39_hw1/MoveAction.h>

void doneCb(const actionlib::SimpleClientGoalState& state,
            const group39_hw1::MoveResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    for (int i = 0; i < result->coordinates.size(); i++)
    {
        ROS_INFO("Movable obstacle %d: x = %f, y = %f", i, result->coordinates[i].x, result->coordinates[i].y);
    }
}

void activeCb()
{
    ROS_INFO("Start Moving");
}

void feedbackCb(const group39_hw1::MoveFeedbackConstPtr& feedback)
{
    ROS_INFO("%s", feedback->f.c_str());
}

int main (int argc, char **argv)
{
    // Constants
    const float GR39_TIMEOUT = 60.0;

    // Initialize ROS
    ros::init(argc, argv, "client");
    actionlib::SimpleActionClient<group39_hw1::MoveAction> ac("movement", true);
    
    // Wait for the action server to start
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // Send goal
    group39_hw1::MoveGoal goal;
    //parse arguments if any

    goal.x = 11;
    goal.y = 0;
    goal.theta = -70;

    if(argc < 4){
        ROS_INFO("Used default value for x,y,theta (%f,%f,%f)",goal.x,goal.y,goal.theta);
    }
    else{
        goal.x = atof(argv[1]);     
        goal.y = atof(argv[2]);     
        goal.theta = atof(argv[3]);
    }
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    
    // Wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(GR39_TIMEOUT));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    // Exit
    return 0;
}