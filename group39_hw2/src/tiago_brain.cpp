#include <ros/ros.h>
#include <tiago_iaslab_simulation/Objs.h>           // IDs
#include <group39_hw1/MoveAction.h>                 // HW1 TIAGo movement
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/PointHeadAction.h>           // Head movement
#include <group39_hw2/DetectAction.h>               // Detection
#include <group39_hw2/ManipulateAction.h>           // Manipulation

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define DEBUG false

class TiagoBrain
{
private:
    struct Gr39_Coordinates { float x, y, z; };
    struct Gr39_Task { Gr39_Coordinates pickup, delivery; };

    /**
     * @brief Perform movement, pickup and delivery tasks for a given object ID
     * 
     * @param id Object ID
     * @throws std::runtime_error if object ID is not recognized (i.e. not 1, 2 or 3)
     */
    void gr39_task(int id);

    /**
     * @brief Move TIAGo to a given position
     * 
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z angle (in degrees)
     * @throws std::runtime_error if movement server does not finish after 60 seconds
     */
    void gr39_move(float x, float y, float z);
    
    /**
     * @brief Perform pickup task for a given object ID
     * 
     * @param id Object ID
     * @throws std::runtime_error if no objects are detected, if object with given ID is not detected, or if actions take too long
     */
    void gr39_pickup(int id);

    void gr39_deliver(int id);

    /**
     * @brief Move TIAGo's head to a given angle
     * 
     * @param a Angle
     * @throws std::runtime_error if head movement server does not finish after 10 seconds
     */
    void gr39_move_head(float a);

    /**
     * @brief Move TIAGo's torso to a given angle
     * 
     * @param a Angle
     * @throws std::runtime_error if torso movement server does not finish after 10 seconds
     */
    void gr39_move_torso(float a);

public:
    /**
     * @brief Construct a new Tiago Brain object and perform all tasks for all IDs received from human_objects_srv
     *
     * @note Handles exceptions WITHOUT crashing the routine (i.e. if an exception is thrown, the routine will continue with the next ID)
     */
    TiagoBrain();
};

TiagoBrain::TiagoBrain()
{
    // Get objects IDs from human_objects_srv
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv");
    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;
    srv.request.all_objs = true;
    if (!client.call(srv)) { ROS_ERROR("ERROR | Failed to call service human_objects_srv"); return; }

    // Process objects
    for (int id : srv.response.ids)
    {
        ROS_INFO("\nProcessing object %d ..", id);
        try { gr39_task(id); }
        catch (const std::exception& e) { ROS_ERROR("ERROR | %s", e.what()); }
        if (DEBUG) break;
    }
}

void TiagoBrain::gr39_task(int id)
{
    if (id < 1 || id > 3) throw std::runtime_error("Object ID not recognized");

    // Constants
    const struct { float x, y; } BASE = {8.5, 0.5};
    const Gr39_Task TASKS[3] =
    {
        { {8.15, -2.1, -90}, {12.5, 0.6, -90} }, //{ {8.9, -2.7, -180}, {0.0, 0.0, -90} }, // Blue
        { {7.8, -4, 90},   {11.45, 0.6, -90} }, //{ {7.7, -4.1, 90},   {0.0, 0.0, -90} }, // Green
        { {7.45, -2, -70},  {10.5, 0.6, -90} }  //{ {7.2, -1.9, -50},  {0.0, 0.0, -90} }  // Red
    };

    // Position in front of object
    gr39_move(BASE.x, BASE.y, -90);
    if (id == 2) gr39_move(8.7, -4.2, -180); // Green is special
    gr39_move(TASKS[id - 1].pickup.x, TASKS[id - 1].pickup.y, TASKS[id - 1].pickup.z);
    
    // Pick up object
    gr39_pickup(id);

    // Position in front of delivery
    if (id == 2) gr39_move(8.7, -4.2, 90); // Green is special pt.2
    gr39_move(BASE.x, BASE.y, 0);
    gr39_move(TASKS[id - 1].delivery.x, TASKS[id - 1].delivery.y, TASKS[id - 1].delivery.z);

    // Deliver object
    gr39_deliver(id);
}

void TiagoBrain::gr39_move(float x, float y, float z)
{
    actionlib::SimpleActionClient<group39_hw1::MoveAction> ac("movement", true);
    ROS_INFO("START  | Waiting for movement server to start.");
    ac.waitForServer();
    ROS_INFO("DONE   | Movement server started, sending goal.");

    // Setting goal
    group39_hw1::MoveGoal goal;
    goal.x = x;
    goal.y = y;
    goal.theta = z;
    ac.sendGoal(goal);

    // Waiting for result
    if (!ac.waitForResult(ros::Duration(60.0))) throw std::runtime_error("Movement server did not finish after 60 seconds");
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("STATUS | Movement server finished with state %s", state.toString().c_str());
}

void TiagoBrain::gr39_pickup(int id)
{
    // Look down at the table
    const float HEAD_UP = 1.0, HEAD_DOWN = 0.6;
    gr39_move_head(HEAD_DOWN);
    gr39_move_torso(0.35);

    // Detect objects
    actionlib::SimpleActionClient<group39_hw2::DetectAction> ac("detection", true);
    ROS_INFO("START  | Waiting for detection server to start.");
    ac.waitForServer();
    ROS_INFO("DONE   | Detection server started, sending goal.");

    // Setting goal
    group39_hw2::DetectGoal goal;
    goal.ready = true;
    ac.sendGoal(goal);
    if (!ac.waitForResult(ros::Duration(10.0))) throw std::runtime_error("Detection server did not finish after 10 seconds");
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("STATUS | Detection server finished with state %s", state.toString().c_str());

    // Get and check results
    group39_hw2::DetectResultConstPtr result = ac.getResult();
    if (result->detectedObj.size() == 0) throw std::runtime_error("No objects detected");
    bool found = false;
    for (const auto& obj : result->detectedObj) if (obj.id == id) { found = true; break; }
    if (!found) throw std::runtime_error("Object with given ID not detected");

    // Manipulate object
    actionlib::SimpleActionClient<group39_hw2::ManipulateAction> ac2("manipulation", true);
    ROS_INFO("START  | Waiting for manipulation server to start.");
    ac2.waitForServer();
    ROS_INFO("DONE   | Manipulation server started, sending goal.");

    // Setting goal
    group39_hw2::ManipulateGoal goal2;
    goal2.attach = true;
    goal2.id = id;
    goal2.detectedObj = result->detectedObj;
    ac2.sendGoal(goal2);
    if (!ac2.waitForResult(ros::Duration(60.0))) throw std::runtime_error("Manipulation server did not finish after 60 seconds");

    // Move head up
    gr39_move_head(HEAD_UP);
}

void TiagoBrain::gr39_deliver(int id)
{   
    actionlib::SimpleActionClient<group39_hw2::ManipulateAction> ac("manipulation", true);
    ROS_INFO("START  | Waiting for manipulation server to start.");
    ac.waitForServer();
    ROS_INFO("DONE   | Manipulation server started, sending goal.");

    // Setting goal
    group39_hw2::ManipulateGoal goal;
    goal.attach = false;
    goal.id = id;
    ac.sendGoal(goal);
    if (!ac.waitForResult(ros::Duration(60.0))) throw std::runtime_error("Manipulation server did not finish after 60 seconds");
}

void TiagoBrain::gr39_move_head(float a)
{
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac("head_controller/point_head_action", true);
    ROS_INFO("START  | Waiting for head movement server to start.");
    ac.waitForServer();
    ROS_INFO("DONE   | Head movement server started, sending goal.");

    // Setting goal
    control_msgs::PointHeadGoal goal;
    goal.target.header.frame_id = "base_link";
    goal.target.point.x = 1.0;
    goal.target.point.y = 0.0;
    goal.target.point.z = a;
    goal.pointing_frame = "xtion_optical_frame";
    goal.max_velocity = 1.0;
    goal.min_duration = ros::Duration(1.0);
    ac.sendGoal(goal);

    // Waiting for result
    if (!ac.waitForResult(ros::Duration(10.0))) throw std::runtime_error("Head movement server did not finish after 10 seconds");
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("STATUS | Head movement server finished with state %s", state.toString().c_str());
}

void TiagoBrain::gr39_move_torso(float a)
{
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    goal.trajectory.points.resize(1);
    int i = 0;
    goal.trajectory.points[i].positions.resize(1);
    goal.trajectory.points[i].positions[0] = a;
    goal.trajectory.points[i].velocities.resize(1);
    goal.trajectory.points[i].velocities[0] = 1.0;
    goal.trajectory.points[i].time_from_start = ros::Duration(2.0);
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("torso_controller/follow_joint_trajectory", true);
    ROS_INFO("START  | Waiting for torso movement server to start.");
    ac.waitForServer();
    ROS_INFO("DONE   | Torso movement server started, sending goal.");
    ac.sendGoal(goal);

    // Waiting for result
    if (!ac.waitForResult(ros::Duration(10.0))) throw std::runtime_error("Torso movement server did not finish after 10 seconds");
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("STATUS | Torso movement server finished with state %s", state.toString().c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brain");
    TiagoBrain tb;
    return 0;
}