// Copyright (C) 2012, 2019 Austin Robot Technology, Jack O'Quin, Joshua Whitley, Sebastian Pütz
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef VELODYNE_POINTCLOUD_POINTCLOUDXYZIRSAFE_H
#define VELODYNE_POINTCLOUD_POINTCLOUDXYZIRSAFE_H

#include <velodyne_pointcloud/datacontainerbase.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/SafeNodeConfig.h>
//#include <ctime>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::time_point<std::chrono::system_clock> CTime;

namespace velodyne_pointcloud
{
class PointcloudXYZIRSafe : public velodyne_rawdata::DataContainerBase
{
public:
  PointcloudXYZIRSafe(const double max_range, const double min_range, const std::string& target_frame,
                  const std::string& fixed_frame, const unsigned int scans_per_block,
                  boost::shared_ptr<tf::TransformListener> tf_ptr = boost::shared_ptr<tf::TransformListener>());

  virtual void finish();

  virtual void newLine();

  virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg);

  virtual void addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth,
                        const float distance, const float intensity, const float time);

  void addPixel(uint16_t azimuth, float distance, float ring);

  void callback(velodyne_pointcloud::SafeNodeConfig &config, uint32_t level);

  sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity, iter_time;
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;

private:
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::SafeNodeConfig> > srv_;
  ros::Publisher safe_pub_;
  ros::Publisher img_pub_;
  float min_x_;
  float min_y_;
  float min_z_;
  float max_x_;
  float max_y_;
  float max_z_;
  boost::mutex mtx;
  bool init;
  bool haspublish;
  double publish_rate;
  CTime start;
  cv::Mat img;
};
}  // namespace velodyne_pointcloud

#endif  // VELODYNE_POINTCLOUD_POINTCLOUDXYZIRSAFE_H
