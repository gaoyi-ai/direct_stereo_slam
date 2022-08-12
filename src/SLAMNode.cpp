//
// Created by yi on 8/10/22.
//


#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include "SLAMNode.h"
#include "loop_closure/pangolin_viewer/PangolinLoopViewer.h"

void SLAMNode::settingsDefault(int preset, int mode) {
  printf("\n=============== PRESET Settings: ===============\n");
  if (preset == 1 || preset == 3) {
    printf("preset=%d is not supported", preset);
    exit(1);
  }
  if (preset == 0) {
    printf("DEFAULT settings:\n"
           "- 2000 active points\n"
           "- 5-7 active frames\n"
           "- 1-6 LM iteration each KF\n"
           "- original image resolution\n");

    dso::setting_desiredImmatureDensity = 1500;
    dso::setting_desiredPointDensity = 2000;
    dso::setting_minFrames = 5;
    dso::setting_maxFrames = 7;
    dso::setting_maxOptIterations = 6;
    dso::setting_minOptIterations = 1;
  }

  if (preset == 2) {
    printf("FAST settings:\n"
           "- 800 active points\n"
           "- 4-6 active frames\n"
           "- 1-4 LM iteration each KF\n"
           "- 424 x 320 image resolution\n");

    dso::setting_desiredImmatureDensity = 600;
    dso::setting_desiredPointDensity = 800;
    dso::setting_minFrames = 4;
    dso::setting_maxFrames = 6;
    dso::setting_maxOptIterations = 4;
    dso::setting_minOptIterations = 1;

    dso::benchmarkSetting_width = 424;
    dso::benchmarkSetting_height = 320;
  }

  if (mode == 0) {
    printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
  }
  if (mode == 1) {
    printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
    dso::setting_photometricCalibration = 0;
    dso::setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    dso::setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
  }
  if (mode == 2) {
    printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
    dso::setting_photometricCalibration = 0;
    dso::setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
    dso::setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
    dso::setting_minGradHistAdd = 3;
  }

  printf("==============================================\n");
}

SLAMNode::SLAMNode(const std::vector<double> &tfm_stereo,
                   const std::string &calib0, const std::string &calib1,
                   const std::string &vignette0, const std::string &vignette1,
                   const std::string &gamma0, const std::string &gamma1,
                   bool nomt, int preset, int mode, float scale_opt_thres,
                   float lidar_range, float scan_context_thres)
    : tfm_stereo_(tfm_stereo), scale_opt_thres_(scale_opt_thres) {
  // DSO front end
  settingsDefault(preset, mode);

  dso::multiThreading = !nomt;

  undistorter0_ = dso::Undistort::getUndistorterForFile(calib0, gamma0, vignette0);
  undistorter1_ = dso::Undistort::getUndistorterForFile(calib1, gamma1, vignette1);
  assert((int) undistorter0_->getSize()[0] == (int) undistorter1_->getSize()[0]);
  assert((int) undistorter0_->getSize()[1] == (int) undistorter1_->getSize()[1]);

  dso::setGlobalCalib((int) undistorter0_->getSize()[0],
                      (int) undistorter0_->getSize()[1],
                      undistorter0_->getK().cast<float>());

  front_end_ = new dso::FrontEnd(tfm_stereo_, undistorter1_->getK().cast<float>(),
                                 scale_opt_thres_);
  if (undistorter0_->photometricUndist != 0)
    front_end_->setGammaFunction(undistorter0_->photometricUndist->getG());

  // Loop closure
  dso::IOWrap::PangolinLoopViewer *pangolinViewer = 0;
  if (!dso::disableAllDisplay) {
    pangolinViewer = new dso::IOWrap::PangolinLoopViewer(
        (int) undistorter0_->getSize()[0], (int) undistorter0_->getSize()[1]);
    front_end_->output_wrapper_.push_back(pangolinViewer);
  }
  loop_handler_ =
      new dso::LoopHandler(lidar_range, scan_context_thres, pangolinViewer);
  // setLoopHandler is called even if loop closure is disabled
  // because results are recordered by LoopHandler
  // but loop closure is disabled internally if loop closure is disabled
  front_end_->setLoopHandler(loop_handler_);

  incomingId = 0;
  currentTimeStamp = -1.0;

  elas_wrapper_ = new ELASWrapper(pangolinViewer);
  front_end_->setElasWrapper(elas_wrapper_);
}

SLAMNode::~SLAMNode() {
  loop_handler_->savePose();

  printf("\n\n************** Statistics (ms) ***************\n");
  std::cout << "===========VO Time============ " << std::endl;
  print_average("feature_detect", front_end_->feature_detect_time_);
  print_average("scale_opt", front_end_->scale_opt_time_);
  print_average("dso_opt", front_end_->opt_time_);

  std::cout << "======Loop Closure Time======= " << std::endl;
  print_average("pts_generation", loop_handler_->pts_generation_time_);
  print_average("sc_generation", loop_handler_->sc_generation_time_);
  print_average("search_ringkey", loop_handler_->search_ringkey_time_);
  print_average("search_sc", loop_handler_->search_sc_time_);
  print_average("direct_est", loop_handler_->direct_est_time_);
  print_average("icp", loop_handler_->icp_time_);
  print_average("pose_graph_opt", loop_handler_->opt_time_);
  printf("loop_count: %d (direct) + %d (icp)\n",
         loop_handler_->direct_loop_count_, loop_handler_->icp_loop_count_);

  std::cout << "============================== " << std::endl;
  print_average("per_frame", frame_tt_);

  printf("**********************************************\n\n\n");

  delete undistorter0_;
  delete undistorter1_;
  delete loop_handler_;
  for (auto &ow: front_end_->output_wrapper_) {
    delete ow;
  }
  delete front_end_;

  delete elas_wrapper_;
}

void SLAMNode::imageMessageCallback(const sensor_msgs::ImageConstPtr &msg0,
                                    const sensor_msgs::ImageConstPtr &msg1) {
  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // detect if a new sequence is received, restart if so
  if (currentTimeStamp > 0 &&
      fabs(msg0->header.stamp.toSec() - currentTimeStamp) > 10) {
    front_end_->is_lost_ = true;
  }
  currentTimeStamp = msg0->header.stamp.toSec();

  // reinitialize if necessary
  if (front_end_->init_failed_ || front_end_->is_lost_) {
    auto lastPose = front_end_->cur_pose_;
    int existing_kf_size = front_end_->getTotalKFSize();
    std::vector<dso::IOWrap::Output3DWrapper *> wraps = front_end_->output_wrapper_;
    delete front_end_;

    printf("Reinitializing\n");
    front_end_ = new dso::FrontEnd(tfm_stereo_, undistorter1_->getK().cast<float>(),
                                   scale_opt_thres_, existing_kf_size);
    if (undistorter0_->photometricUndist != 0)
      front_end_->setGammaFunction(undistorter0_->photometricUndist->getG());
    front_end_->setLoopHandler(loop_handler_);
    front_end_->output_wrapper_ = wraps;
    front_end_->cur_pose_ = lastPose;
    // setting_fullResetRequested=false;

    front_end_->setElasWrapper(elas_wrapper_);
  }

  dso::MinimalImageB minImg0((int) img0.cols, (int) img0.rows,
                             (unsigned char *) img0.data);
  dso::ImageAndExposure *undistImg0 =
      undistorter0_->undistort<unsigned char>(&minImg0, 1, 0, 1.0f);
  undistImg0->timestamp = msg0->header.stamp.toSec();
  dso::MinimalImageB minImg1((int) img1.cols, (int) img1.rows,
                             (unsigned char *) img1.data);
  dso::ImageAndExposure *undistImg1 =
      undistorter1_->undistort<unsigned char>(&minImg1, 1, 0, 1.0f);

  auto t0 = std::chrono::steady_clock::now();
  // vo entry
  front_end_->addActiveStereoFrame(undistImg0, undistImg1, incomingId);
  auto t1 = std::chrono::steady_clock::now();
  frame_tt_.push_back(t1 - t0);

  // matching entry
  elas_wrapper_->process(msg0, msg1);

  incomingId++;
  delete undistImg0;
  delete undistImg1;
}

void SLAMNode::imgInfoMsgCB(const sensor_msgs::CameraInfoConstPtr &l_info_msg,
                            const sensor_msgs::CameraInfoConstPtr &r_info_msg) {
  elas_wrapper_->update_stereo_model(l_info_msg, r_info_msg);
}
