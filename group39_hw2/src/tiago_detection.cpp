#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <group39_hw2/DetectAction.h>
#include <group39_hw2/DetectedObj.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Detection
{
protected:
    ros::NodeHandle nh_;
    std::string action_name_;
    ros::Subscriber tag_sub_;
    actionlib::SimpleActionServer<group39_hw2::DetectAction> as_;
    std::vector<group39_hw2::DetectedObj> objs;
    geometry_msgs::TransformStamped transform;

private:
    /**
     * @brief convert apriltag_ros::AprilTagDetectionArray to std::vector<group39_hw2::DetectedObj>
     * 
     * @param msg input message
     * @return converted std::vector<group39_hw2::DetectedObj>
     */
    std::vector<group39_hw2::DetectedObj> apriltag2obj(const apriltag_ros::AprilTagDetectionArray& msg)
    {
        std::vector<group39_hw2::DetectedObj> objs;
        const auto& detections = msg.detections;
        for (const auto& detection : detections)
        {
            group39_hw2::DetectedObj obj;
            obj.id = detection.id[0];
            geometry_msgs::PoseWithCovarianceStamped pose = detection.pose;
            geometry_msgs::PoseWithCovariance pose_cov = pose.pose;
            geometry_msgs::Pose pose_ = pose_cov.pose;
            obj.pose = pose_;
            objs.push_back(obj);
        }
        return objs;
    }

public:
    /**
     * @brief Construct a new Detection object
     * 
     * @param name name of the action
     */
    Detection(std::string name)
    : as_(nh_, name, boost::bind(&Detection::executeCB, this, _1), false), action_name_(name)
    { as_.start(); }

    /**
     * @brief Destroy the Detection object
     */
    ~Detection(void) {}

    /**
     * @brief Callback function for the action
     * 
     * @param goal goal of the action
     */
    void executeCB(const group39_hw2::DetectGoalConstPtr &goal)
    {
        ROS_INFO("START  | Detecting objects.");
        boost::shared_ptr<const apriltag_ros::AprilTagDetectionArray> spAprilTagDetections;
        apriltag_ros::AprilTagDetectionArray aprilTagDetections;

        // Wait for apriltag detections
        spAprilTagDetections = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
        if (spAprilTagDetections != NULL) aprilTagDetections = *spAprilTagDetections;
        objs = apriltag2obj(aprilTagDetections);
        ROS_INFO("DONE   | Detected %d objects", (int)objs.size());

        // Conversion to robot frame
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        try
        {
            transform = tfBuffer.lookupTransform
            (
                "base_footprint",          // Target frame
                "xtion_rgb_optical_frame", // Source frame
                ros::Time(0),              // Get the tf at first available time
                ros::Duration(5.0)         // Wait for 5 second
            );
        }
        catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); exit(EXIT_FAILURE); }

        // Send result
        group39_hw2::DetectResult result;
        for (const auto& obj : objs)
        {
            group39_hw2::DetectedObj obj_robot_frame;
            obj_robot_frame.id = obj.id;
            geometry_msgs::Pose new_pose;
            tf2::doTransform(obj.pose, new_pose, transform);
            obj_robot_frame.pose = new_pose;
            result.detectedObj.push_back(obj_robot_frame);
        }
        as_.setSucceeded(result);
        ROS_INFO("STATUS | Detected %d objects", (int)objs.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection");
    Detection detection("detection");
    ros::spin();
    return 0;
}