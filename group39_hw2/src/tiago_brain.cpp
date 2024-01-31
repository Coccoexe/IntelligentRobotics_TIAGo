#include <ros/ros.h>
#include <tiago_iaslab_simulation/Objs.h>           // IDs
#include <group39_hw1/MoveAction.h>                 // HW1 TIAGo movement
#include <group39_hw1/Coordinates.h>                 // HW1 TIAGo movement
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/PointHeadAction.h>           // Head movement
#include <group39_hw2/DetectAction.h>               // Detection
#include <group39_hw2/ManipulateAction.h>           // Manipulation

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <image_transport/image_transport.h>    // Extra point
#include <cv_bridge/cv_bridge.h>                // Extra point
#include <opencv2/opencv.hpp>                   // Extra point
#include <cmath>                                // Extra point

#define MANUAL_MODE false
#define ONLY_ONE false
#define EXTRA_POINT true

class TiagoBrain
{
private:
    /// @brief Structs for storing coordinates
    struct Gr39_Coordinates { float x, y, z; };

    /// @brief Struct for storing pickup and delivery coordinates (task)
    struct Gr39_Task { Gr39_Coordinates pickup, delivery; };

    ros::NodeHandle nh_;
    ros::ServiceClient client;
    actionlib::SimpleActionClient<group39_hw1::MoveAction> move;
    actionlib::SimpleActionClient<group39_hw2::DetectAction> detect;
    actionlib::SimpleActionClient<group39_hw2::ManipulateAction> manipulate;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> head;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso;
    image_transport::Subscriber sub;
    cv_bridge::CvImagePtr cv_ptr;

    const float HEAD_UP = 1.0, HEAD_DOWN = 0.6;

    Gr39_Coordinates deliveries[3];

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
     * @throws std::runtime_error if movement server does not succeed
     */
    void gr39_move(float x, float y, float z);

    /**
    * @brief Move TIAGo to a given position and return detected obstacles coordinates
    */
    void gr39_move_and_detect(float x, float y, float z, std::vector<group39_hw1::Coordinates>  &obstacles);
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief Perform pickup task for a given object ID
     * 
     * @param id Object ID
     * @throws std::runtime_error if no objects are detected, if object with given ID is not detected, or if actions do not succeed
     */
    void gr39_pickup(int id);

    /**
     * @brief Perform delivery task for a given object ID
     * 
     * @param id Object ID
     * @throws std::runtime_error if actions do not succeed
     */
    void gr39_deliver(int id);

    /**
     * @brief Move TIAGo's head to a given angle
     * 
     * @param a Angle
     * @throws std::runtime_error if head movement server does not succeed
     */
    void gr39_move_head(float a);

    /**
     * @brief Move TIAGo's torso to a given angle
     * 
     * @param a Angle
     * @throws std::runtime_error if torso movement server does not succeed
     */
    void gr39_move_torso(float a);

    /**
    * @brief Extra point task function, detect and indentify place table
    */
    void gr39_extra_point();

public:
    /**
     * @brief Construct a new Tiago Brain object and perform all tasks for all IDs received from human_objects_srv
     *
     * @note Handles exceptions WITHOUT crashing the routine (i.e. if an exception is thrown, the routine will continue with the next ID)
     */
    TiagoBrain(ros::NodeHandle nh);
};

TiagoBrain::TiagoBrain(ros::NodeHandle nh)
: nh_(nh), move("movement", true), detect("detection", true), manipulate("manipulation", true),
  head("head_controller/point_head_action", true), torso("torso_controller/follow_joint_trajectory", true)
{
    this->client = this->nh_.serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv");
    ROS_INFO("START  | Waiting for nodes to start.");
    this->move.waitForServer();
    ROS_INFO("DONE   | Movement server started.");
    this->detect.waitForServer();
    ROS_INFO("DONE   | Detect service started.");
    this->manipulate.waitForServer();
    ROS_INFO("DONE   | Manipulate service started.");
    this->head.waitForServer();
    ROS_INFO("DONE   | Head movement server started.");
    this->torso.waitForServer();
    ROS_INFO("DONE   | Torso movement server started.");

    image_transport::ImageTransport it(this->nh_);
    sub = it.subscribe("/xtion/rgb/image_color", 1, &TiagoBrain::imageCallback, this);

    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;
    srv.request.all_objs = true;
    if (!client.call(srv)) { ROS_ERROR("ERROR | Failed to call service human_objects_srv"); return; }

    // Extra point task
    ROS_INFO("STATUS | Performing extra point task");
    if (EXTRA_POINT) gr39_extra_point();
    ROS_INFO("DONE   | Extra point task completed");

    // Process objects
    if (MANUAL_MODE) srv.response.ids = {2};
    for (int id : srv.response.ids)
    {
        ROS_INFO("\nProcessing object %d ..", id);
        try { gr39_task(id); }
        catch (const std::exception& e) { ROS_ERROR("ERROR | %s", e.what()); }
        if (ONLY_ONE) break;
    }
}

void TiagoBrain::gr39_task(int id)
{
    if (id < 1 || id > 3) throw std::runtime_error("Object ID not recognized");

    // Constants
    const struct { float x, y; } BASE = {8.5, 0.5};
    Gr39_Task TASKS[3] =
    {
        { {8.1, -2.1, -90},  {12.5, 0.5, -90} }, // Blue
        { {7.8, -4.0, 90},   {11.4, 0.5, -90} }, // Green
        { {7.30, -2.0, -65}, {10.5, 0.5, -90} }  // Red
    };
    if(EXTRA_POINT)
    {
        TASKS[0].delivery = deliveries[0];
        TASKS[1].delivery = deliveries[1];
        TASKS[2].delivery = deliveries[2];
    }

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
    // Setting goal
    group39_hw1::MoveGoal goal;
    goal.x = x;
    goal.y = y;
    goal.theta = z;
    ROS_INFO("STATUS | Sending goal to movement server");
    move.sendGoalAndWait(goal);
    if (move.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("STATUS | Movement server finished with state %s", move.getState().toString().c_str());
    else
        throw std::runtime_error("Movement server did not finish successfully");
}

void TiagoBrain::gr39_move_and_detect(float x, float y, float z, std::vector<group39_hw1::Coordinates>  &obstacles)
{
    // Setting goal
    group39_hw1::MoveGoal goal;
    goal.x = x;
    goal.y = y;
    goal.theta = z;
    ROS_INFO("STATUS | Sending goal to movement server");
    move.sendGoalAndWait(goal);
    if (move.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("STATUS | Movement server finished with state %s", move.getState().toString().c_str());
    else
        throw std::runtime_error("Movement server did not finish successfully");

    // Get obstacles coordinates
    for (auto coord : move.getResult()->coordinates)
    {
        obstacles.push_back(coord);
    }
}

void TiagoBrain::gr39_pickup(int id)
{
    // Look down at the table
    gr39_move_head(this->HEAD_DOWN);
    gr39_move_torso(0.35);

    // Detect object
    // Setting goal
    group39_hw2::DetectGoal goal;
    goal.ready = true;
    detect.sendGoalAndWait(goal);
    if (detect.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("STATUS | Detection server finished with state %s", detect.getState().toString().c_str());
    else
        throw std::runtime_error("Detection server did not finish successfully");

    // Get and check results
    group39_hw2::DetectResultConstPtr result = detect.getResult();
    if (result->detectedObj.size() == 0) throw std::runtime_error("No objects detected");
    bool found = false;
    for (const auto& obj : result->detectedObj) if (obj.id == id) { found = true; break; }
    if (!found) throw std::runtime_error("Object with given ID not detected");

    // Manipulate object
    // Setting goal
    group39_hw2::ManipulateGoal goal2;
    goal2.attach = true;
    goal2.id = id;
    goal2.detectedObj = result->detectedObj;
    manipulate.sendGoalAndWait(goal2);
    if (manipulate.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("STATUS | Manipulation server finished with state %s", manipulate.getState().toString().c_str());
    else
        throw std::runtime_error("Manipulation server did not finish successfully");

    // Move head up
    gr39_move_head(this->HEAD_UP);
}

void TiagoBrain::gr39_deliver(int id)
{   
    // Setting goal
    group39_hw2::ManipulateGoal goal;
    goal.attach = false;
    goal.id = id;
    manipulate.sendGoalAndWait(goal);
    if (manipulate.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("STATUS | Manipulation server finished with state %s", manipulate.getState().toString().c_str());
    else
        throw std::runtime_error("Manipulation server did not finish successfully");
}

void TiagoBrain::gr39_move_head(float a)
{
    // Setting goal
    control_msgs::PointHeadGoal goal;
    goal.target.header.frame_id = "base_link";
    goal.target.point.x = 1.0;
    goal.target.point.y = 0.0;
    goal.target.point.z = a;
    goal.pointing_frame = "xtion_optical_frame";
    goal.max_velocity = 1.0;
    goal.min_duration = ros::Duration(1.0);
    head.sendGoalAndWait(goal);
    if (head.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("STATUS | Head movement server finished with state %s", head.getState().toString().c_str());
    else
        throw std::runtime_error("Head movement server did not finish successfully");
}

void TiagoBrain::gr39_move_torso(float a)
{
    // Setting goal
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
    torso.sendGoalAndWait(goal);
    if (torso.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("STATUS | Torso movement server finished with state %s", torso.getState().toString().c_str());
    else
        ROS_ERROR("ERROR  | Torso movement failed with error %s", torso.getState().toString().c_str());
}

void TiagoBrain::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try { this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void TiagoBrain::gr39_extra_point()
{
    // Move in front of all tables
    Gr39_Coordinates waypoint = {11.0, 0.7, -70}; // {11.4, 1.0, -90};
    std::vector<group39_hw1::Coordinates> tables;
    gr39_move_and_detect(waypoint.x, waypoint.y, waypoint.z, tables);

    int sum = 0;

    ROS_INFO("STATUS | Processing detected tables");
    for(auto coord : tables)
    {
        // Check table distance from tiago
        if ( std::pow(coord.x - waypoint.x, 2) + std::pow(coord.y - waypoint.y, 2) > 4.0 )
        {
            ROS_INFO("WARN   | Table too far away, skipping");
            continue;
        } 

        // Move in front of table
        gr39_move(coord.x, coord.y + 0.75, -90);

        // Move head down
        gr39_move_head(this->HEAD_DOWN);

        ros::Duration(1.0).sleep();
        ros::spinOnce();
        cv::Mat img = this->cv_ptr->image;

        cv::Scalar color = cv::mean(img);

        // Color detection
        float max = 0;
        int max_index = 0;
        for (int i = 0; i < 3; i++)
        {
            if (color[i] > max)
            {
                max = color[i];
                max_index = i;
            }
        }
        std::string color_name = "";
        switch (max_index)
        {
            case 0: color_name = "BLUE"; break;
            case 1: color_name = "GREEN"; break;
            case 2: color_name = "RED"; break;
        }

        // Store delivery coordinates
        deliveries[max_index].x = coord.x;
        deliveries[max_index].y = coord.y + 0.65 + 0.025 * max_index; // detection compensation
        deliveries[max_index].z = -90 - 7.5 * max_index;              // detection compensation

        ROS_INFO("STATUS | Table in coordinates (%.2f, %.2f) detected as %s", coord.x, coord.y, color_name.c_str());

        // Move head up
        gr39_move_head(this->HEAD_UP);

        // Increment sum
        sum += std::pow(max_index, 2) + 1;
    }

    // Check number and color of tables
    if (sum != 8)
    {
        ROS_ERROR("ERROR | Wrong number / color of tables detected");
        return;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brain");
    ros::NodeHandle nh;
    TiagoBrain tb(nh);
    return 0;
}