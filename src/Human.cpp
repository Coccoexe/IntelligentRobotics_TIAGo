#include <tiago_iaslab_simulation/Human.h>

Human::Human(std::shared_ptr<ros::NodeHandle> nh_ptr)
: nh_ptr_(nh_ptr)
{
  start();
}

void Human::start(){
  objects_server_ = nh_ptr_->advertiseService("/human_objects_srv", &Human::objService, this);
  ROS_INFO_STREAM("Service done!");
}

int Human::randomNumber(){
  int range = MAX_ - MIN_ + 1;
  std::srand(time(NULL));
  int num = std::rand() % range + MIN_;
  return num;
}

bool Human::objService(tiago_iaslab_simulation::Objs::Request &req, tiago_iaslab_simulation::Objs::Response &res){
    if(!req.ready){
        ROS_ERROR_STREAM("Error! Ready is false. It must be TRUE");
        ros::shutdown();
        return false;
    }
    std::vector<int> ids_vec;
    if(req.all_objs){
      while(ids_vec.size() < 3){
        int num = randomNumber();
        if(std::find(ids_vec.begin(), ids_vec.end(), num) == ids_vec.end()){
          ids_vec.push_back(num);
        }
      }
    }
    else{
      ids_vec.push_back(randomNumber());
    }
    res.ids = ids_vec;
    return true;
}
