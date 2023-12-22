// Authors: Group 39 (Alessio Cocco, Andrea Valentinuzzi, Giovanni Brejc)

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <group39_hw1/MoveAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Server
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<group39_hw1::MoveAction> as_;
    std::string action_name_;
    group39_hw1::MoveFeedback feedback_;
    group39_hw1::MoveResult result_;
public:
    Server(std::string name)
    : as_(nh_, name, boost::bind(&Server::executeCB, this, _1), false), action_name_(name)
    {
        // Start the action server
        as_.start();
    }
    ~Server(void){}
    void executeCB(const group39_hw1::MoveGoalConstPtr &goal)
    {
        // Flag to check if the goal was reached
        bool success = true;

        // Clear the feedback vector
        feedback_.sequence.clear();

        // Tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);
    
        // Wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    
        move_base_msgs::MoveBaseGoal goalPosition;
    
        // We'll send a goal to the robot to move 1 meter forward
        goalPosition.target_pose.header.frame_id = "map"; // "base_link" le coordinate si riferiscono a dove parte, mentre con map si riferiscono alla mappa generale
        goalPosition.target_pose.header.stamp = ros::Time::now();
    
        goalPosition.target_pose.pose.position.x = goal->x;    // 1.0;
        goalPosition.target_pose.pose.position.y = goal->y;    // 1.0;
        goalPosition.target_pose.pose.orientation.z = goal->z; // 1.0;
    
        ROS_INFO("Sending goal");
        ac.sendGoal(goalPosition);
    
        ac.waitForResult();
    
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            success = true;
        else
            success = false;

        if(success){
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "server");
    Server movement("movement");
    ros::spin();
    return 0;
}