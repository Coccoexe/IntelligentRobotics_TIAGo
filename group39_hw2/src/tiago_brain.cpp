#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tiago_iaslab_simulation/Objs.h>

int main (int argc, char **argv)
{
    // Constants
    struct Gr39_Coordinates
    {
        float x, y, z;
    };
    struct Gr39_Task
    {
        struct Gr39_Coordinates pickup, delivery;
    };
    const Gr39_Coordinates
        COORD_BASE = {8.5, 0.5, -45},
        COORD_R = {8.5, -4, -180},
    const Gr39_Task
        TASK_R = {{7.4, -2.1, -90}, {0.0, 0.0, -90}},
        TASK_G = {{7.5, -3.9, 90}, {0.0, 0.0, -90}},
        TASK_B = {{8.7, -2.7, -180}, {0.0, 0.0, -90}};

    // ROS Initialization
    ros::init(argc, argv, "tiago_brain");
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
        ROS_ERROR("Failed to call service ids");
        return 1;
    }
    
}