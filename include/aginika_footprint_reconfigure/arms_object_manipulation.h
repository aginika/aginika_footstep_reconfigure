// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto Inagaki and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef __ARMS_OBJECT_MANIPULATION_H__
#define __ARMS_OBJECT_MANIPULATION_H__

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <vector>
#include <Eigen/Core>
#include <boost/foreach.hpp>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <jsk_pcl_ros/SetFilterMode.h>
#include <jsk_pcl_ros/SetFrameId.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <limits>
#include <jsk_debug_tools/jsk_debug_tools.h>



namespace arms_aginika_footprint_reconfigure
{
  class ArmsObjectManipulation{
  public:
    ArmsObjectManipulation();
    void run();
    void right_arm_cloud_cb(const sensor_msgs::PointCloud2& cloud_msg);
    void left_arm_cloud_cb(const sensor_msgs::PointCloud2& cloud_msg);
    void convert_to_pcl(const sensor_msgs::PointCloud2& cloud_msg, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
    void update_robot_foot_print();

  public:
    ros::Publisher pub_;
    ros::Subscriber right_arms_sub_;
    ros::Subscriber left_arms_sub_;
    ros::NodeHandle n_;
    int counter_;

    std::string target_frame_id_;

    //point clouds
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr right_cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr left_cloud_;

  };
}


#endif
