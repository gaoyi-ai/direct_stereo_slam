// Copyright (C) <2020> <Jiawei Mo, Junaed Sattar>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <chrono>

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include "FrontEnd.h"
#include "loop_closure/pangolin_viewer/PangolinLoopViewer.h"
#include "SLAMNode.h"

using namespace dso;

int main(int argc, char **argv) {
  ros::init(argc, argv, "direct_stereo_slam");
  ros::NodeHandle nhPriv("~");

  /* *********************** required parameters ************************ */
  // stereo camera parameters
  std::vector<double> tfm_stereo;
  std::string topic0, topic1, calib0, calib1, info0, info1;
  if (!nhPriv.getParam("T_stereo/data", tfm_stereo) ||
      !nhPriv.getParam("topic0", topic0) ||
      !nhPriv.getParam("topic1", topic1) ||
      !nhPriv.getParam("info0", info0) ||
      !nhPriv.getParam("info1", info1) ||
      !nhPriv.getParam("calib0", calib0) ||
      !nhPriv.getParam("calib1", calib1)) {
    ROS_INFO("Fail to get sensor topics/params, exit.");
    return -1;
  }
  std::string vignette0, vignette1, gamma0, gamma1;
  nhPriv.param<std::string>("vignette0", vignette0, "");
  nhPriv.param<std::string>("vignette1", vignette1, "");
  nhPriv.param<std::string>("gamma0", gamma0, "");
  nhPriv.param<std::string>("gamma1", gamma1, "");

  /* *********************** optional parameters ************************ */
  // DSO settings
  bool nomt, debugSaveImages;
  int preset, mode;
  nhPriv.param("preset", preset, 0);
  nhPriv.param("mode", mode, 1);
  nhPriv.param("quiet", setting_debugout_runquiet, true);
  nhPriv.param("nogui", disableAllDisplay, false);
  nhPriv.param("nomt", nomt, false);
  nhPriv.param("save", debugSaveImages, false);


  // scale optimization accept threshold
  // set to -1 to disable scale optimization, i.e., dso mode
  float scale_opt_thres;
  nhPriv.param("scale_opt_thres", scale_opt_thres, 15.0f);

  // loop closure parameters
  float lidar_range; // set to -1 to disable loop closure
  float scan_context_thres;
  nhPriv.param("lidar_range", lidar_range, 40.0f);
  nhPriv.param("scan_context_thres", scan_context_thres, 0.33f);

  // read from a bag file
  std::string bag_path;
  nhPriv.param<std::string>("bag", bag_path, "");

  /* ******************************************************************** */

  SLAMNode slam_node(tfm_stereo, calib0, calib1, vignette0, vignette1, gamma0,
                     gamma1, nomt, preset, mode, scale_opt_thres, lidar_range,
                     scan_context_thres);

  if (!bag_path.empty()) {

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector <std::string> topics = {topic0, topic1, info0, info1};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    sensor_msgs::ImageConstPtr img0, img1;
    sensor_msgs::CameraInfoConstPtr l_info_msg, r_info_msg;
    bool img0_updated(false), img1_updated(false);
    bool info0_updated(false), info1_updated(false);
    bool info_inited(false);
    BOOST_FOREACH(rosbag::MessageInstance
    const m, view) {
      if (m.getTopic() == topic0) {
        img0 = m.instantiate<sensor_msgs::Image>();
        img0_updated = true;
      }
      if (m.getTopic() == topic1) {
        img1 = m.instantiate<sensor_msgs::Image>();
        img1_updated = true;
      }
      if (img0_updated && img1_updated) {
        assert(fabs(img0->header.stamp.toSec() - img1->header.stamp.toSec()) <
               0.1);
        slam_node.imageMessageCallback(img0, img1);
        img0_updated = img1_updated = false;
      }
      if (m.getTopic() == info0) {
        l_info_msg = m.instantiate<sensor_msgs::CameraInfo>();
        info0_updated = true;
      }
      if (m.getTopic() == info1) {
        r_info_msg = m.instantiate<sensor_msgs::CameraInfo>();
        info1_updated = true;
      }
      if (info0_updated && info1_updated && !info_inited) {
        assert(fabs(l_info_msg->header.stamp.toSec() - r_info_msg->header.stamp.toSec()) <
               0.1);
        slam_node.imgInfoMsgCB(l_info_msg, r_info_msg);
        info_inited = true;
      }
    }
    bag.close();
  } else {

    // ROS subscribe to stereo images
    ros::NodeHandle nh;
    auto *cam0_sub =
        new message_filters::Subscriber<sensor_msgs::Image>(nh, topic0, 10000);
    auto *cam1_sub =
        new message_filters::Subscriber<sensor_msgs::Image>(nh, topic1, 10000);
    auto *sync = new message_filters::Synchronizer <
    message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
        sensor_msgs::Image>>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            sensor_msgs::Image>(10),
        *cam0_sub, *cam1_sub);
    sync->registerCallback(
        boost::bind(&SLAMNode::imageMessageCallback, &slam_node, _1, _2));
    ros::spin();
  }

  return 0;
}
