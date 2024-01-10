#include <ros/ros.h>
#include <tiago_iaslab_simulation/Objs.h>           // ids
#include <group39_hw1/MoveAction.h>                 // move
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/PointHeadAction.h>           // move_head
#include <group39_hw2/DetectAction.h>               // detect
#include <map>

struct Gr39_Coordinates
{
    float x, y, z;
};

void move_head()
{
    ROS_INFO("Moving head");
    // Move head to look at object
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac("head_controller/point_head_action", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    control_msgs::PointHeadGoal goal;
    goal.target.header.frame_id = "base_link";
    goal.target.point.x = 1.0;
    goal.target.point.y = 0.0;
    goal.target.point.z = 0.0;
    goal.pointing_frame = "xtion_optical_frame";
    goal.pointing_axis.x = 1;
    goal.max_velocity = 1.0;
    goal.min_duration = ros::Duration(1.0);
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

}

bool gr39_move(float x, float y, float z)
{
    // Move to x, y, z
    actionlib::SimpleActionClient<group39_hw1::MoveAction> ac("movement", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    
    group39_hw1::MoveGoal goal;
    goal.x = x;
    goal.y = y;
    goal.theta = z;
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

    return finished_before_timeout;
}

int gr39_pickup(int id)
{
    // 1. Detection
    actionlib::SimpleActionClient<group39_hw2::DetectAction> ac("detection", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    group39_hw2::DetectGoal goal;
    goal.ready = true;

    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
    if (!finished_before_timeout)
    {
        ROS_INFO("Action did not finish before the time out.");
        return -1;
    }
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    group39_hw2::DetectResultConstPtr result = ac.getResult();

    std::map<int, Gr39_Coordinates> detection;
    for (const auto& obj : result->detectedObj)
    {
        detection[obj.id] = {obj.x, obj.y, obj.z};
        ROS_INFO("Detected object %d at (%f, %f, %f)", obj.id, obj.x, obj.y, obj.z);
    }
    if (detection.find(id) == detection.end())
    {
        ROS_INFO("Object %d not detected", id);
        return -1;
    }
    
    // 2. Collisions


    // 3. Pickup


    return true;
}

void gr39_deliver(int id)
{
    // 1. Place object on table
    // 2. Open gripper
    // 3. Detach object from gripper
}

int main (int argc, char **argv)
{
    // Constants
    struct Gr39_Task
    {
        struct Gr39_Coordinates pickup, delivery;
    };
    const Gr39_Coordinates
        COORD_BASE = {8.5, 0.5, -45},
        COORD_G = {8.5, -4, -180};
    const Gr39_Task TASK[] = {
        {{8.7, -2.7, -180}, {0.0, 0.0, -90}},   // Blue
        {{7.5, -3.9, 90}, {0.0, 0.0, -90}},     // Green
        {{7.4, -2.1, -90}, {0.0, 0.0, -90}}};   // Red

    // ROS Initialization
    ros::init(argc, argv, "tiago_brain");

    // Get ids
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv");
    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;
    srv.request.all_objs = true;
    if (client.call(srv))
    {
        ROS_INFO("Ready to start");
        ROS_INFO("Object ID: %d", srv.response.ids[0]);
        ROS_INFO("Object ID: %d", srv.response.ids[1]);
        ROS_INFO("Object ID: %d", srv.response.ids[2]);
    }
    else
    {
        ROS_ERROR("Failed to call service human_objects_srv");
        return 1;
    }

    // bgr
    for (int i = 0; i < srv.response.ids.size(); i++)
    {
        int id = srv.response.ids[i];
        //gr39_pickup(id);
        move_head();
        break;

        // 1. Move to COORD_BASE
        if (!gr39_move(COORD_BASE.x, COORD_BASE.y, COORD_BASE.z))
        {
            ROS_INFO("TIAGo -> BASE did not finish before the time out.");
            return 0;
        }

        // 1.opt. Move to COORD_G
        if (id == 2)
        {
            if (!gr39_move(COORD_G.x, COORD_G.y, COORD_G.z))
            {
                ROS_INFO("TIAGo (Green) -> optional BASE did not finish before the time out.");
                return 0;
            }
        }

        // 2. Move to object id's pickup coordinates
        if (!gr39_move(TASK[id - 1].pickup.x, TASK[id - 1].pickup.y, TASK[id - 1].pickup.z))
        {
            ROS_INFO("TIAGo -> PICKUP point did not finish before the time out.");
            return 0;
        }

        // 3. Pick up object
        gr39_pickup(id);

        // 4.opt. Move to COORD_G
        if (id == 2)
        {
            if (!gr39_move(COORD_G.x, COORD_G.y, COORD_G.z))
            {
                ROS_INFO("TIAGo (Green) -> optional BASE did not finish before the time out.");
                return 0;
            }
        }

        // 4. Move to COORD_BASE
        if (!gr39_move(COORD_BASE.x, COORD_BASE.y, COORD_BASE.z))
        {
            ROS_INFO("TIAGo -> BASE did not finish before the time out.");
            return 0;
        }
        
        // 5. Move to delivery
        if (!gr39_move(TASK[id - 1].delivery.x, TASK[id - 1].delivery.y, TASK[id - 1].delivery.z))
        {
            ROS_INFO("TIAGo -> DELIVERY point did not finish before the time out.");
            return 0;
        }
        
        // 6. Place object
        gr39_deliver(id);
        
        // DEBUG only 1 object
        break;
    }
}