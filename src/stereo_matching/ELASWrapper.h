//
// Created by yi on 8/11/22.
//

#ifndef DIRECT_STEREO_SLAM_ELASWRAPPER_H
#define DIRECT_STEREO_SLAM_ELASWRAPPER_H

#include <queue>

#include <libelas/elas.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/stereo_camera_model.h>
#include <message_filters/subscriber.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include "loop_closure/pangolin_viewer/PangolinLoopViewer.h"

namespace dso {
  class FrameHessian;
  class CalibHessian;
  class FrameShell;
  namespace IOWrap {
    class Output3DWrapper;
  }
}

struct Elas3DFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int kf_id;                      // kF id, for pose graph and visualization
  int incoming_id;                // increasing id, for ground truth
  g2o::SE3Quat tfm_w_c;           // coordinate in pose graph
  Eigen::Vector3d trans_w_c_orig; // original pose for logging

  dso::FrameHessian *fh0;
  dso::FrameHessian *fh1;
  std::vector<float> cam;
  float ab_exposure;

  std::vector<Eigen::Vector3d> pts;

  Elas3DFrame(dso::FrameHessian *fh0, dso::FrameHessian *fh1,
            const std::vector<float> &cam)
      : fh0(fh0), fh1(fh1) ,kf_id(fh0->frameID),
        incoming_id(fh0->shell->incoming_id),
        tfm_w_c(g2o::SE3Quat(fh0->shell->camToWorld.rotationMatrix(),
                             fh0->shell->camToWorld.translation())),
        trans_w_c_orig(tfm_w_c.translation()), cam(cam),
        ab_exposure(fh0->ab_exposure){}

  ~Elas3DFrame() {
  }
};

class ELASWrapper {

public:
  ELASWrapper();

  ~ELASWrapper();

  ELASWrapper(dso::IOWrap::PangolinLoopViewer *pangolin_viewer);

  void process(const sensor_msgs::ImageConstPtr &l_image_msg,
               const sensor_msgs::ImageConstPtr &r_image_msg);

  void publish_keyframes(dso::FrameHessian *fh0, dso::FrameHessian *fh1, dso::CalibHessian *h_calib);

  void generate_pc(const sensor_msgs::ImageConstPtr &l_image_msg, float *l_disp_data,
                  const std::vector<int32_t> &inliers,
                  int32_t l_width, int32_t l_height);

  void update_stereo_model(const sensor_msgs::CameraInfoConstPtr &l_info_msg,
                           const sensor_msgs::CameraInfoConstPtr &r_info_msg);

  void run();

  void join();

  void save_pc();

private:

  boost::shared_ptr<Elas> elas_;
  int queue_size_;

  // Struct parameters
  int disp_min;
  int disp_max;
  double support_threshold;
  int support_texture;
  int candidate_stepsize;
  int incon_window_size;
  int incon_threshold;
  int incon_min_support;
  bool add_corners;
  int grid_size;
  double beta;
  double gamma;
  double sigma;
  double sradius;
  int match_texture;
  int lr_threshold;
  double speckle_sim_threshold;
  int speckle_size;
  int ipol_gap_width;
  bool filter_median;
  bool filter_adaptive_mean;
  bool postprocess_only_left;
  bool subsampling;

  ros::NodeHandle nh_;

  image_geometry::StereoCameraModel model_;
  boost::scoped_ptr<Elas::parameters> param_;

  int cur_id_;
  bool running_;
  boost::thread run_thread_;

  std::unordered_map<int, Eigen::Matrix<double, 6, 1>> id_pose_wc_;
  std::vector<std::pair<int, Eigen::Vector3d>> pts_nearby_;
  std::vector<Eigen::Vector3d> pts_;

  boost::mutex elas3d_frame_queue_mutex_;
  std::queue<Elas3DFrame *> elas3d_frame_queue_;
  std::vector<Elas3DFrame *> elas3d_frames_;

  dso::IOWrap::PangolinLoopViewer *pangolin_viewer_;

};


#endif //DIRECT_STEREO_SLAM_ELASWRAPPER_H
