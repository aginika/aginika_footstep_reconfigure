#include "aginika_footprint_reconfigure/arms_aginika_footprint_reconfigure.h"


using namespace arms_aginika_footprint_reconfigure;

ArmsObjectManipulation::ArmsObjectManipulation():n_("~"),right_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),left_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),counter_(0){

  if (!n_.getParam("target_frame_id", target_frame_id_))
    {
      ROS_WARN("~base_link is not specified");
      target_frame_id_ = "base_link";
    }

  right_arms_sub_ =  n_.subscribe("right_cloud_", 1, &ArmsObjectManipulation::right_arm_cloud_cb, this);
  left_arms_sub_ = n_.subscribe("left_cloud_", 1, &ArmsObjectManipulation::left_arm_cloud_cb, this);

}


void ArmsObjectManipulation::right_arm_cloud_cb(const sensor_msgs::PointCloud2& cloud_msg)
{
  convert_to_pcl(cloud_msg, right_cloud_);
}

void ArmsObjectManipulation::left_arm_cloud_cb(const sensor_msgs::PointCloud2& cloud_msg)
{
  convert_to_pcl(cloud_msg, left_cloud_);
}

void ArmsObjectManipulation::convert_to_pcl(const sensor_msgs::PointCloud2& cloud_msg, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    pcl::PCLPointCloud2 pcl_pc;
    std::vector<int> indices;
    pcl_conversions::toPCL(cloud_msg, pcl_pc);
    pcl::fromPCLPointCloud2 (pcl_pc, *cloud);
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
}

void ArmsObjectManipulation::update_robot_foot_print()
{
  
}

void ArmsObjectManipulation::run(){
  //  ros::Rate loop_rate(2);
  while(ros::ok()){
    update_robot_foot_print();
    ros::spinOnce();
    //   loop_rate.sleep();
    counter_++;
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "arms_object_maniputlation");

  ArmsObjectManipulation a;
  a.run();
  return 0;
}
