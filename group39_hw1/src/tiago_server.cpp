// Authors: Group 39 (Alessio Cocco, Andrea Valentinuzzi, Giovanni Brejc)

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <group39_hw1/MoveAction.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#define PI 3.14159265
#define DEBUG false

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Server
{
protected:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_; // laser scanner
    ros::Subscriber pose_sub_; // robot pose
    actionlib::SimpleActionServer<group39_hw1::MoveAction> as_;
    std::string action_name_;
    group39_hw1::MoveFeedback feedback_;
    group39_hw1::MoveResult result_;

private:
    // Constants
    float TIAGO_X                      = 0.0;
    float TIAGO_Y                      = 0.0;
    float TIAGO_YAW                    = 0.0;
    static const int   NUM_DISTANCES   = 666;
    const int   LEGS_WIDTH             = 20;
    const float TIAGO_ANGLE_RANGE      = 1.9198600053787231;
    const float TIAGO_ANGLE_INCREMENT  = 0.005774015095084906;
    const float THRESHOLD_DISTANCE     = 0.5;
    const float THRESHOLD_ANGLE_AVG    = 0.04;

    // Store the distances from the laser scanner
    float dst[NUM_DISTANCES];

    // Helper struct to store coordinates
    struct Gr39_Coordinates
    {
        float x, y;
    };

    void distanceToCoordinates(float *d, Gr39_Coordinates *c)
    {   // Convert laser distance from TIAGo, knowing angle, to absolute coordinates
        for (int i = 0; i < NUM_DISTANCES; i++)
        {
            c[i].x = TIAGO_X + d[i] * cos(TIAGO_YAW - TIAGO_ANGLE_RANGE + i * TIAGO_ANGLE_INCREMENT);
            c[i].y = TIAGO_Y + d[i] * sin(TIAGO_YAW - TIAGO_ANGLE_RANGE + i * TIAGO_ANGLE_INCREMENT);
        }
    }
    
    void computeAngles(Gr39_Coordinates *c, float *a)
    {   // Compute angles between each poit an the next one
        for (int i = 0; i < NUM_DISTANCES - 1; i++)
        {
            a[i] = atan2(c[i + 1].y - c[i].y, c[i + 1].x - c[i].x);
        }
        a[NUM_DISTANCES - 1] = a[NUM_DISTANCES - 2];
    }

    void findObstacles()
    {
        // Check if we have laser data and pose data
        if (dst[0] == 0)
        {   // No laser data, skip
            ROS_INFO("No laser data, skipping");
            return;
        }
        if (TIAGO_X == 0 || TIAGO_Y == 0 || TIAGO_YAW == 0)
        {   // No pose data, skip
            ROS_INFO("No pose data, skipping");
            return;
        }
        ROS_INFO("TIAGo pose: (%f, %f, %f)\n\n\n", TIAGO_X, TIAGO_Y, TIAGO_YAW);

        // Convert relative distances to absolute coordinates
        Gr39_Coordinates crd[NUM_DISTANCES];
        distanceToCoordinates(&dst[0], &crd[0]);
        if (DEBUG) ROS_INFO("DEBUG: coordinates point(100) = (%f, %f)", crd[100].x, crd[100].y);

        // Compute angles between each point and the next one
        float angle[NUM_DISTANCES];
        computeAngles(&crd[0], &angle[0]);
        if (DEBUG) ROS_INFO("DEBUG: angle point(100) = %f", angle[100]);

        // Find POIs (Points Of Interest), defined as: distance change > THRESHOLD_DISTANCE, removing LEGS_WIDTH left and right readings, since they are obstructed by the robot's legs
        int prev_index = -1;
        for (int i = 0 + LEGS_WIDTH - 1; i < NUM_DISTANCES - LEGS_WIDTH + 1; i++)
        {
            // 0. This is the first check, to see if the distance change is significant
            if (abs(dst[i] - dst[i + 1]) <= THRESHOLD_DISTANCE)
            {   // Distance change is not significant, skip WITHOUT saving the index
                continue;
            }
            if (DEBUG) ROS_INFO("DEBUG: d[point(%d), point(%d)] = %f", i, i + 1, abs(dst[i] - dst[i + 1]));

            // 1. Now point(i) & point(i+1) create a separation between two different depths (to define if wall or obstacle)
            if (prev_index == -1)
            {   // First POI after first leg, just save the index (+1)
                prev_index = i + 1;
                continue;
            }

            // 2. Skip immediately if distance(point(prev_index), point(i)) > THRESHOLD_DISTANCE (i.e. the two POIs are too far apar, then they are not part of the same obstacle)
            float gap = sqrt(pow(crd[prev_index].x - crd[i].x, 2) + pow(crd[prev_index].y - crd[i].y, 2));
            if (gap > THRESHOLD_DISTANCE)
            {   // Too far apart, just save the index (+1)
                prev_index = i + 1;
            	continue;
            }
            if (DEBUG) ROS_INFO("DEBUG: gap[point(%d), point(%d)] = %f", prev_index, i, gap);

            // 3. Analyze the gap between point(prev_index) & point(i) to determine if it is an obstacle, then save the index (+1)
            float delta_angle_avg = 0;
            for (int j = prev_index; j < i; j++)
            {   // Compute average difference between consecutive angles (high if going around round obstacle)
            	delta_angle_avg += abs(angle[j] - angle[j + 1]);
            }
            delta_angle_avg /= (i - prev_index);
            if (delta_angle_avg <= THRESHOLD_ANGLE_AVG)
            {   // Most likely flat surface(s), just save the index (+1)
                prev_index = i + 1;
                continue;
            }
            
            // 4. Here we're left with a range [prev_index, i] that is most likely an obstacle, so we can compute its center
            float m1 = - 1.0 / angle[prev_index - 1];
            float m2 = -1.0 / angle[i];
            if (DEBUG) ROS_INFO("DEBUG: m1 = %f, m2 = %f", m1, m2);
            if (m1 == m2)
            {   // Hope this never happens :)
                prev_index = i + 1;
                continue;
            }
            Gr39_Coordinates intersection;
            intersection.x = (m1 * crd[prev_index].x - m2 * crd[i].x + crd[i].y - crd[prev_index].y) / (m1 - m2);
            intersection.y = m1 * (intersection.x - crd[prev_index].x) + crd[prev_index].y;
            prev_index = i + 1;

            // 5. Obstacle confirmed at Gr39_Coordinates intersection
            ROS_INFO("Obstacle found at (%f, %f)", intersection.x, intersection.y);
        }            
    }

public:
    Server(std::string name)
    : as_(nh_, name, boost::bind(&Server::executeCB, this, _1), false), action_name_(name)
    {
        // Start the action server
        as_.start();
    }
    ~Server(void){}
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // Add laser distance data to the dst array
        for(int i = 0; i < msg->ranges.size(); i++){
            float range = msg->ranges[i];
            dst[i] = range;
            ROS_INFO("%f", range);
        }

        // Find obstacles
        findObstacles();
    }
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // Here you can access the pose data
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        geometry_msgs::Quaternion q_msg = msg->pose.pose.orientation;
        tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // Save the pose
        TIAGO_X = float(x);
        TIAGO_Y = float(y);
        TIAGO_YAW = float(yaw);
    }
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
        double degree = goal->theta;
        //to radians
        double rad = degree * PI / 180;
        tf2::Quaternion q;
        q.setRPY(0, 0, rad);
        goalPosition.target_pose.pose.orientation = tf2::toMsg(q.normalize()); 

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

            geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg2 = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", ros::Duration(10));
            if(msg2 != nullptr)
                poseCallback(msg2);

            sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(10));
            ros::Subscriber sub = nh_.subscribe("/amcl_pose", 1000, &Server::poseCallback, this);
            if(msg != nullptr)
                laserCallback(msg);
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "server");
    Server movement("movement");
    ros::spin();
    return 0;
}