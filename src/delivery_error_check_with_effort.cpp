#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <vector>
#include <stdlib.h>
#include <limits>
using namespace std;

class JointEffortMonitor{
public:
  JointEffortMonitor():n_("~"){

    if (!n_.getParam("threshold", threshold_))
      {
        ROS_WARN("~threshold_ is not specified");
        threshold_ = 4;
      }

    sub_ = n_.subscribe("input", 1, &JointEffortMonitor::joint_state_cb, this);
    first_time_ = true;
  };

  void joint_state_cb(const sensor_msgs::JointState js){
    //    ROS_INFO("callback");
    //first time
    if(first_time_){
      first_time_ = false;
      for(int i = 0; i < js.name.size(); i++){
        //skip not concerned
        if(js.name[i].find("caster") != string::npos || js.name[i].find("tilt") != string::npos || js.name[i].find("head") != string::npos){
          continue;
        }
        joint_and_effort_.insert( map< std::string, double >::value_type( js.name[i], js.effort[i]) );

        if(js.effort[i] == 0.0){
          joint_and_zero_appear_.insert( map< std::string, int >::value_type( js.name[i], 1) );
        }else{
          joint_and_zero_appear_.insert( map< std::string, int >::value_type( js.name[i], 0) );
        }
      }
    }
    //not First time
    else{

      for(int i = 0; i < js.name.size(); i++){
        if(js.name[i].find("caster") != string::npos || js.name[i].find("tilt") != string::npos || js.name[i].find("head") != string::npos){
          continue;
        }

        double log_effort;
        if(js.effort[i] == 0.0){
          if(joint_and_zero_appear_.find(js.name[i])->second > 0){
            log_effort = log(0.000001);
          }else{
            joint_and_zero_appear_.find(js.name[i])->second = 1;
            continue;
          }
        }else{
          joint_and_zero_appear_.find(js.name[i])->second = 0;
          log_effort = log(js.effort[i]);
        }

        std::map<std::string, double>::iterator joint_and_effort_target_ = joint_and_effort_.find(js.name[i]);
        double diff = joint_and_effort_target_->second - log_effort;
        //diff
        if(abs(diff) > threshold_){
          ROS_INFO("Difference Appear %s : diff %lf   %lf -> %lf(%lf)", js.name[i].c_str(), diff, joint_and_effort_target_->second,  log_effort, js.effort[i]);
        }

        //update
        joint_and_effort_target_->second = log_effort;
      }

    }
  }

  void run(){
    while(ros::ok()){
      ros::spin();
    }
  }

  ros::Subscriber sub_;
  ros::NodeHandle n_;
  bool first_time_;

  double threshold_;
  map<std::string, double> joint_and_effort_;
  map<std::string, int> joint_and_zero_appear_;
};


int main(int argc, char* argv[]){
  ros::init(argc, argv, "joint_effort_monitor");
  JointEffortMonitor jem;
  jem.run();
}
