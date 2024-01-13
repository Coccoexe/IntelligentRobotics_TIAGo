#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <group39_hw2/ManipulateAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

#include <moveit_msgs/CollisionObject.h> // new
#include <control_msgs/FollowJointTrajectoryAction.h> // new
#include <gazebo_ros_link_attacher/Attach.h> // new
#include <tf2_ros/transform_listener.h> // new
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // new

#define DEBUG true

class Manipulation
{
protected:
    ros::NodeHandle nh_;
    std::string action_name_;
    ros::Subscriber tag_sub_;
    actionlib::SimpleActionServer<group39_hw2::ManipulateAction> as_;

    ros::AsyncSpinner spinner_; // new
    std::vector<moveit_msgs::CollisionObject> collisionObjects_; // new
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> gripper_client_; // new
    ros::ServiceClient attachService_; // new
    ros::ServiceClient detachService_; // new

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface move_group_;

    void addStaticCollisionObjects()
    {        
        // List of immovable object data pick_table
        moveit_msgs::CollisionObject pickTable;

        pickTable.id = "pick_table";
        pickTable.header.frame_id = "map";

        pickTable.primitives.resize(1);
        pickTable.primitives[0].type = pickTable.primitives[0].BOX;
        pickTable.primitives[0].dimensions.resize(3);
        pickTable.primitives[0].dimensions[0] = 0.91 + 0.1; // x + error
        pickTable.primitives[0].dimensions[1] = 0.92 + 0.1; // y + error
        pickTable.primitives[0].dimensions[2] = 0.775; // z

        pickTable.primitive_poses.resize(1);
        pickTable.primitive_poses[0].position.x = 7.8;
        pickTable.primitive_poses[0].position.y = -2.95;
        pickTable.primitive_poses[0].position.z = 0.3875;

        pickTable.operation = pickTable.ADD;

        collisionObjects_.push_back(pickTable);

        // place_table_b
        moveit_msgs::CollisionObject placeTableB;
        placeTableB.id = "place_table_b";
        placeTableB.header.frame_id = "map";

        placeTableB.primitives.resize(1);
        placeTableB.primitives[0].type = placeTableB.primitives[0].CYLINDER;
        placeTableB.primitives[0].dimensions.resize(2);
        placeTableB.primitives[0].dimensions[0] = 0.69; // Height
        placeTableB.primitives[0].dimensions[1] = 0.21; // Radious

        placeTableB.primitive_poses.resize(1);
        placeTableB.primitive_poses[0].position.x = 12.52;
        placeTableB.primitive_poses[0].position.y = -0.35;
        placeTableB.primitive_poses[0].position.z = 0.345000;

        placeTableB.operation = placeTableB.ADD;

        collisionObjects_.push_back(placeTableB);

        // place_table_g
        moveit_msgs::CollisionObject placeTableG;
        placeTableG.id = "place_table_g";
        placeTableG.header.frame_id = "map";

        placeTableG.primitives.resize(1);
        placeTableG.primitives[0].type = placeTableG.primitives[0].CYLINDER;
        placeTableG.primitives[0].dimensions.resize(2);
        placeTableG.primitives[0].dimensions[0] = 0.69; // Height
        placeTableG.primitives[0].dimensions[1] = 0.21; // Radious

        placeTableG.primitive_poses.resize(1);
        placeTableG.primitive_poses[0].position.x = 11.52;
        placeTableG.primitive_poses[0].position.y = -0.35;
        placeTableG.primitive_poses[0].position.z = 0.345000;

        placeTableG.operation = placeTableG.ADD;

        collisionObjects_.push_back(placeTableG);

        // place_table_r
        moveit_msgs::CollisionObject placeTableR;
        placeTableR.id = "place_table_r";
        placeTableR.header.frame_id = "map";

        placeTableR.primitives.resize(1);
        placeTableR.primitives[0].type = placeTableR.primitives[0].CYLINDER;
        placeTableR.primitives[0].dimensions.resize(2);
        placeTableR.primitives[0].dimensions[0] = 0.69; // Height
        placeTableR.primitives[0].dimensions[1] = 0.21; // Radious

        placeTableR.primitive_poses.resize(1);
        placeTableR.primitive_poses[0].position.x = 10.52;
        placeTableR.primitive_poses[0].position.y = -0.34;
        placeTableR.primitive_poses[0].position.z = 0.345000;
        
        placeTableR.operation = placeTableR.ADD;

        collisionObjects_.push_back(placeTableR);

        // construction_cone_1
        moveit_msgs::CollisionObject constructionCone1;
        constructionCone1.id = "construction_cone_1";
        constructionCone1.header.frame_id = "map";

        constructionCone1.primitives.resize(1);
        constructionCone1.primitives[0].type = constructionCone1.primitives[0].CYLINDER;
        constructionCone1.primitives[0].dimensions.resize(2);
        constructionCone1.primitives[0].dimensions[0] = 1.457100; // Height
        constructionCone1.primitives[0].dimensions[1] = 0.183096; // Radious

        constructionCone1.primitive_poses.resize(1);
        constructionCone1.primitive_poses[0].position.x = 11.725;
        constructionCone1.primitive_poses[0].position.y = -3.07;
        constructionCone1.primitive_poses[0].position.z = 0.728548;

        constructionCone1.operation = constructionCone1.ADD;

        collisionObjects_.push_back(constructionCone1);

        // construction_cone_4
        moveit_msgs::CollisionObject constructionCone4;
        constructionCone4.id = "construction_cone_4";
        constructionCone4.header.frame_id = "map";

        constructionCone4.primitives.resize(1);
        constructionCone4.primitives[0].type = constructionCone4.primitives[0].CYLINDER;
        constructionCone4.primitives[0].dimensions.resize(2);
        constructionCone4.primitives[0].dimensions[0] = 1.457100; // height
        constructionCone4.primitives[0].dimensions[1] = 0.183096 + 0.01; // radious + error

        constructionCone4.primitive_poses.resize(1);
        constructionCone4.primitive_poses[0].position.x = 7.35;
        constructionCone4.primitive_poses[0].position.y = -0.72;
        constructionCone4.primitive_poses[0].position.z = 0.728548;

        constructionCone4.operation = constructionCone4.ADD;

        collisionObjects_.push_back(constructionCone4);

        planning_scene_interface_.applyCollisionObjects(collisionObjects_); // Add collision objects to planing scene
    }
    void addCollisionObject(const group39_hw2::ManipulateGoalConstPtr &goal)
    {
        for (const auto& obj : goal->detectedObj)
        {
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = "base_footprint";
            collision_object.id = std::to_string(obj.id);
            collision_object.primitives.resize(1);

            if (obj.id == 1) // blue
            {
                collision_object.primitives[0].type = collision_object.primitives[0].CYLINDER;
                collision_object.primitives[0].dimensions.resize(2);
                collision_object.primitives[0].dimensions[0] = 0.1; // Height
                collision_object.primitives[0].dimensions[1] = 0.03; // Radious
            }
            else if (obj.id == 2) // green
            {
                collision_object.primitives[0].type = collision_object.primitives[0].BOX;
                collision_object.primitives[0].dimensions.resize(3);
                collision_object.primitives[0].dimensions[0] = 0.07; // x
                collision_object.primitives[0].dimensions[1] = 0.05; // y
                collision_object.primitives[0].dimensions[2] = 0.05; // z
            }
            else if (obj.id == 3) // red
            {
                collision_object.primitives[0].type = collision_object.primitives[0].BOX;
                collision_object.primitives[0].dimensions.resize(3);
                collision_object.primitives[0].dimensions[0] = 0.05; // x
                collision_object.primitives[0].dimensions[1] = 0.05; // y
                collision_object.primitives[0].dimensions[2] = 0.05; // z
            }
            else if (obj.id > 3 && obj.id < 8)
            {
                collision_object.primitives[0].type = collision_object.primitives[0].CYLINDER;
                collision_object.primitives[0].dimensions.resize(2);
                collision_object.primitives[0].dimensions[0] = 0.2; // Height
                collision_object.primitives[0].dimensions[1] = 0.05; // Radious
            }

            collision_object.primitive_poses.resize(1);
            collision_object.primitive_poses[0] = obj.pose;
            collision_object.primitive_poses[0].position.z -= collision_object.primitives[0].dimensions[0] / 2;
            collision_object.operation = collision_object.ADD;
            collisionObjects_.push_back(collision_object);
        }
        planning_scene_interface_.applyCollisionObjects(collisionObjects_); // Add collision objects to planing scene
    }
    void removeCollisionObjects()
    {
        planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());
    }
    void removeCollisionById(int id)
    {
        std::string id_str = std::to_string(id);
        std::vector<std::string> ids;
        for (const auto& obj : collisionObjects_)
        {
            if (obj.id == id_str)
            {
                ids.push_back(obj.id);
            }
        }
        planning_scene_interface_.removeCollisionObjects(ids);
    }
    // ARM
    geometry_msgs::Quaternion gripperOrientation(double roll, double pitch, double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();
        return tf2::toMsg(q);
    }
    bool moveArm(geometry_msgs::Point point, geometry_msgs::Quaternion orientation)
    {
        ROS_INFO("Moving arm to point (%f, %f, %f)", point.x, point.y, point.z);

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "base_footprint";
        target_pose.pose.position = point;
        target_pose.pose.orientation = orientation;

        //set target
        move_group_.setPoseTarget(target_pose);
        move_group_.setStartStateToCurrentState();
        move_group_.setMaxVelocityScalingFactor(1.0);
        move_group_.setPlanningTime(6.0);
        move_group_.setGoalJointTolerance(0.05);

        //planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        //execute
        if (!success)
        {
            ROS_ERROR("Failed to plan");
            return false;
        }

        ROS_INFO("Moving arm");
        move_group_.move();
        return true;
    }
    // approach object from above with gripper facing down
    bool approachFromAbove(geometry_msgs::Point point, double dist)
    {
        point.z += dist;
        return moveArm(point, gripperOrientation(0, M_PI_2, 0));
    }
    bool safePosition()
    {
        geometry_msgs::Point point;
        point.x = 0.2;
        point.y = 0.2;
        point.z = 1.6;
        geometry_msgs::Quaternion orientation = gripperOrientation(0, 0, 4.71);
        return moveArm(point, orientation);
    }

    // GRIPPER
    bool moveGripper(double finger_1, double finger_2)
    {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
        goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
        goal.trajectory.points.resize(1);
        int i = 0;
        goal.trajectory.points[i].positions.resize(2);
        goal.trajectory.points[i].positions[0] = finger_1;
        goal.trajectory.points[i].positions[1] = finger_2;
        goal.trajectory.points[i].velocities.resize(2);
        for (int j = 0; j < 2; ++j)
        {
            goal.trajectory.points[i].velocities[j] = 1.0;
        }
        goal.trajectory.points[i].time_from_start = ros::Duration(2.0);

        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        gripper_client_->sendGoalAndWait(goal);

        if (gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Gripper moved");
            return true;
        }
        else
        {
            ROS_INFO("Gripper failed to move");
            return false;
        }
    }
    bool openGripper()
    {
        return moveGripper(0.04, 0.04);
    }
    bool closeGripper()
    {
        return moveGripper(0.0, 0.0);
    }

    // PICK AND PLACE
    bool pickObject(int id)
    {
        gazebo_ros_link_attacher::Attach attach_srv_msg;
        attach_srv_msg.request.model_name_1 = "tiago";
        attach_srv_msg.request.link_name_1 = "arm_7_link";
        switch (id)
        {
            case 1:
                attach_srv_msg.request.model_name_2 = "Hexagon";
                attach_srv_msg.request.link_name_2 = "Hexagon_link";
                break;
            case 2:
                attach_srv_msg.request.model_name_2 = "Triangle";
                attach_srv_msg.request.link_name_2 = "Triangle_link";
                break;
            case 3:
                attach_srv_msg.request.model_name_2 = "Cube";
                attach_srv_msg.request.link_name_2 = "Cube_link";
                break;
            default:
                ROS_ERROR("Object with id %d not supported", id);
                return false;
        }
        if (attachService_.call(attach_srv_msg))
        {
            ROS_INFO("Attached object");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to attach object");
            return false;
        }
    }
    bool placeObject(int id)
    {
        gazebo_ros_link_attacher::Attach attach_srv_msg;
        attach_srv_msg.request.model_name_1 = "tiago";
        attach_srv_msg.request.link_name_1 = "arm_7_link";
        switch (id)
        {
            case 1:
                attach_srv_msg.request.model_name_2 = "Hexagon";
                attach_srv_msg.request.link_name_2 = "Hexagon_link";
                break;
            case 2:
                attach_srv_msg.request.model_name_2 = "Triangle";
                attach_srv_msg.request.link_name_2 = "Triangle_link";
                break;
            case 3:
                attach_srv_msg.request.model_name_2 = "Cube";
                attach_srv_msg.request.link_name_2 = "Cube_link";
                break;
            default:
                ROS_ERROR("Object with id %d not supported", id);
                return false;
        }
        if (detachService_.call(attach_srv_msg))
        {
            ROS_INFO("Detached object");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to detach object");
            return false;
        }
    }

public:
    Manipulation(std::string name)
    : as_(nh_, name, boost::bind(&Manipulation::executeCB, this, _1), false), action_name_(name), move_group_("arm_torso"), spinner_(1)
    {
        gripper_client_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/gripper_controller/follow_joint_trajectory", true)); // new
        while (!gripper_client_->waitForServer(ros::Duration(5.0))) // new
        {
            ROS_INFO("Waiting for the gripper action server to come up");
        }
        attachService_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach"); // new
        attachService_.waitForExistence(); // new
        detachService_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach"); // new
        detachService_.waitForExistence(); // new
        spinner_.start(); // new

        as_.start();
    }
    ~Manipulation(void) { spinner_.stop(); }
    void executeCB(const group39_hw2::ManipulateGoalConstPtr &goal)
    {
        //print goal
        ROS_INFO("Received goal");
        ROS_INFO("Attach: %d", goal->attach);
        ROS_INFO("ID: %d", goal->id);
        
        //make collision object
        addStaticCollisionObjects();

        //if goal attach
        if (goal->attach)
        {
            addCollisionObject(goal);

            //object to attach
            geometry_msgs::Pose object_pose;
            for (const auto& obj : goal->detectedObj)
            {
                if (obj.id == goal->id)
                {
                    object_pose = obj.pose;
                    break;
                }
            }

            openGripper();

            //define distances to the object
            double approach_dist, target_dist;
            switch(goal->id)
            {
                case 1: // blue
                    approach_dist = 0.31;
                    target_dist = 0.20;
                    break;
                case 2: // green
                    approach_dist = 0.31;
                    target_dist = 0.24;
                    break;
                case 3: // red
                    approach_dist = 0.25;
                    target_dist = 0.20;
                    break;
                default: // other
                    ROS_ERROR("Object with id %d not supported", goal->id);
                    break;
            }

            approachFromAbove(object_pose.position, approach_dist);
            approachFromAbove(object_pose.position, target_dist);

            //remove collision of object to pick
            removeCollisionById(goal->id);

            pickObject(goal->id);
            closeGripper();

            //move arm up
            approachFromAbove(object_pose.position, approach_dist);

            //move arm to safe position
            safePosition();
        }
        else
        {
            geometry_msgs::Point point;
            point.x = 0.8;
            point.y = 0.0;
            point.z = 1.0;
            moveArm(point, gripperOrientation(0, M_PI_2, 0));

            openGripper();

            placeObject(goal->id);

            safePosition();
        }

        //remove collision objects
        removeCollisionObjects();

        //if goal detach

        group39_hw2::ManipulateResult result;
        result.success = true;
        as_.setSucceeded(result);

        //delete planning scene
        //planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manipulation");
    Manipulation manipulation("manipulation");
    ros::waitForShutdown();
    return 0;
}