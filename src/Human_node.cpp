#include <ros/ros.h>
#include <tiago_iaslab_simulation/Human.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "human_node");
  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  Human human(nh_ptr);

  ros::spin();

  return 0;
}
