#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <group39_hw2/DetectAction.h>
#include <group39_hw2/DetectedObj.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <map>

#define DEBUG true

/**
 * @brief Class for detecting objects in the environment, using AprilTags
 */
class Detection
{
protected:
    ros::NodeHandle nh_;
    std::string action_name_;
    ros::Subscriber tag_sub_;
    actionlib::SimpleActionServer<group39_hw2::DetectAction> as_;
private:
    struct Gr39_Coordinates
    {
        float x, y, z;
    };

    /// Map of detected objects <id, global coordinates>
    std::map<int, Gr39_Coordinates> detection_map;
    
    /**
     * @brief Callback function for the AprilTag detection
     * @param msg AprilTag detection message
     */
    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
    {
        ROS_INFO("Received tag detection");
        // Scan for objects
        for (const auto& detection : msg->detections)
        {   // 1. Get the id of the detected object
            int id = detection.id[0];

            // 2. Check if the object has already been detected, if so, skip
            if (detection_map.find(id) != detection_map.end()) continue;

            // 3. Get the relative position of the detected object
            geometry_msgs::PoseWithCovarianceStamped pose = detection.pose;
            geometry_msgs::Point p = pose.pose.pose.position;

            // 4. Transform the relative position to the global position
            tf::TransformListener listener;
            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("/map", pose.header.frame_id, ros::Time(0), ros::Duration(2.0));
                listener.lookupTransform("/map", pose.header.frame_id, ros::Time(0), transform);
                double x = transform.getOrigin().x();
                double y = transform.getOrigin().y();
                double z = transform.getOrigin().z();
                tf::Vector3 global_point = transform * tf::Vector3(p.x, p.y, p.z); 

                // 5. Add the detected object to the map
                detection_map[id] = {(float)global_point.x(), (float)global_point.y(), (float)global_point.z()};
                ROS_INFO("Detected object %d at (%f, %f, %f)", id, global_point.x(), global_point.y(), global_point.z());
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }
    }
public:
    Detection(std::string name)
    : as_(nh_, name, boost::bind(&Detection::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }
    ~Detection(void) {}
    void executeCB(const group39_hw2::DetectGoalConstPtr &goal)
    {
        tag_sub_ = nh_.subscribe("/tag_detections", 1, &Detection::tagCallback, this);
        ROS_INFO("Detecting objects");
        //tagCallback(ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections"));
        ros::Duration(4.0).sleep();
        group39_hw2::DetectResult result;
        for (const auto& detection : detection_map)
        {
            group39_hw2::DetectedObj obj;
            obj.id = detection.first;
            obj.x = detection.second.x;
            obj.y = detection.second.y;
            obj.z = detection.second.z;
            result.detectedObj.push_back(obj);
        }
        tag_sub_.shutdown();
        as_.setSucceeded(result);
        ROS_INFO("Detected %d objects", (int)result.detectedObj.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection");
    Detection detection("detection");
    ros::spin();
    return 0;
}