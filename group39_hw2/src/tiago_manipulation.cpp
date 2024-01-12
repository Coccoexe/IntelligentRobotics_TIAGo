#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <group39_hw2/ManipulateAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

#define DEBUG true

class Manipulation
{
protected:
    ros::NodeHandle nh_;
    std::string action_name_;
    ros::Subscriber tag_sub_;
    actionlib::SimpleActionServer<group39_hw2::ManipulateAction> as_;
    
private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    void makeCollisionObject(const group39_hw2::ManipulateGoalConstPtr &goal)
    {
        for (const auto& obj : goal->detectedObj)
        {
            ROS_INFO("Object %d at (%f, %f, %f)", obj.id, obj.x, obj.y, obj.z);

            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = "map";
            collision_object.id = std::to_string(obj.id);

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.05;
            primitive.dimensions[1] = 0.05;
            primitive.dimensions[2] = 0.05;

            geometry_msgs::Pose box_pose;
            box_pose.position.x = obj.x;
            box_pose.position.y = obj.y;
            box_pose.position.z = obj.z;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            planning_scene_interface_.applyCollisionObject(collision_object);
        }

        //collision of table
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "map";
        collision_object.id = "table";
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.8;
        primitive.dimensions[1] = 0.8;
        primitive.dimensions[2] = 0.8;
        geometry_msgs::Pose box_pose;
        box_pose.position.x = 7.825;
        box_pose.position.y = -2.983;
        box_pose.position.z = 0.38;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        planning_scene_interface_.applyCollisionObject(collision_object);
    }
    void attachCallback()
    {
        
    }
    void detachCallback()
    {
        
    }
public:
    Manipulation(std::string name)
    : as_(nh_, name, boost::bind(&Manipulation::executeCB, this, _1), false), action_name_(name), move_group_("arm_torso")
    {
        as_.start();
    }
    ~Manipulation(void) {}
    void executeCB(const group39_hw2::ManipulateGoalConstPtr &goal)
    {
        //print goal
        ROS_INFO("Received goal");
        ROS_INFO("Attach: %d", goal->attach);
        ROS_INFO("ID: %d", goal->id);
        
        //make collision object
        makeCollisionObject(goal);

        //if goal attach
        if (goal->attach)
        {

            geometry_msgs::PoseStamped target_pose;

            //take position of object
            for (const auto& obj : goal->detectedObj)
            {
                if (obj.id == goal->id)
                {
                    target_pose.pose.position.x = obj.x;
                    target_pose.pose.position.y = obj.y;
                    target_pose.pose.position.z = obj.z + 0.4;
                }
            }

            move_group_.setPoseReferenceFrame("map");
            move_group_.setPoseTarget(target_pose);
            move_group_.setPlanningTime(10.0);
            move_group_.move();

        }

        //if goal detach

        as_.setSucceeded();

        //delete planning scene
        //planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manipulation");
    Manipulation manipulation("manipulation");
    ros::spin();
    return 0;
}