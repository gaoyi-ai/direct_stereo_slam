//
// Created by yi on 8/11/22.
//
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ELASWrapper.h"

ELASWrapper::ELASWrapper() : cur_id_(-1) {
  ros::NodeHandle local_nh("~");

  local_nh.param<int>("disp_min", disp_min, 0);
  local_nh.param<int>("disp_max", disp_max, 255);
  local_nh.param<double>("support_threshold", support_threshold, 0.95);
  local_nh.param<int>("support_texture", support_texture, 10);
  local_nh.param<int>("candidate_stepsize", candidate_stepsize, 5);
  local_nh.param<int>("incon_window_size", incon_window_size, 5);
  local_nh.param<int>("incon_threshold", incon_threshold, 5);
  local_nh.param<int>("incon_min_support", incon_min_support, 5);
  local_nh.param<bool>("add_corners", add_corners, 0);
  local_nh.param<int>("grid_size", grid_size, 20);
  local_nh.param<double>("beta", beta, 0.02);
  local_nh.param<double>("gamma", gamma, 3);
  local_nh.param<double>("sigma", sigma, 1);
  local_nh.param<double>("sradius", sradius, 2);
  local_nh.param<int>("match_texture", match_texture, 1);
  local_nh.param<int>("lr_threshold", lr_threshold, 2);
  local_nh.param<double>("speckle_sim_threshold", speckle_sim_threshold, 1);
  local_nh.param<int>("speckle_size", speckle_size, 200);
  local_nh.param<int>("ipol_gap_width", ipol_gap_width, 300);
  local_nh.param<bool>("filter_median", filter_median, 0);
  local_nh.param<bool>("filter_adaptive_mean", filter_adaptive_mean, 1);
  local_nh.param<bool>("postprocess_only_left", postprocess_only_left, 1);
  local_nh.param<bool>("subsampling", subsampling, 0);

  // Create the elas processing class
  //param.reset(new Elas::parameters(Elas::MIDDLEBURY));
  //param.reset(new Elas::parameters(Elas::ROBOTICS));
  param_.reset(new Elas::parameters);

  /* Parameters tunned*/
  param_->disp_min = disp_min;
  param_->disp_max = disp_max;
  param_->support_threshold = support_threshold;
  param_->support_texture = support_texture;
  param_->candidate_stepsize = candidate_stepsize;
  param_->incon_window_size = incon_window_size;
  param_->incon_threshold = incon_threshold;
  param_->incon_min_support = incon_min_support;
  param_->add_corners = add_corners;
  param_->grid_size = grid_size;
  param_->beta = beta;
  param_->gamma = gamma;
  param_->sigma = sigma;
  param_->sradius = sradius;
  param_->match_texture = match_texture;
  param_->lr_threshold = lr_threshold;
  param_->speckle_sim_threshold = speckle_sim_threshold;
  param_->speckle_size = speckle_size;
  param_->ipol_gap_width = ipol_gap_width;
  param_->filter_median = filter_median;
  param_->filter_adaptive_mean = filter_adaptive_mean;
  param_->postprocess_only_left = postprocess_only_left;
  param_->subsampling = subsampling;

  //param_->match_texture = 1;
  //param_->postprocess_only_left = 1;
  //param_->ipol_gap_width = 2;
#ifdef DOWN_SAMPLE
  param_->subsampling = true;
#endif
  elas_.reset(new Elas(*param_));

  running_ = true;
  run_thread_ = boost::thread(&ELASWrapper::run, this);
}

ELASWrapper::ELASWrapper(dso::IOWrap::PangolinLoopViewer *pangolin_viewer)
    : pangolin_viewer_(pangolin_viewer) {
  new(this) ELASWrapper();
}

ELASWrapper::~ELASWrapper() {
  running_ = false;
  run_thread_.join();
}

void ELASWrapper::join() {
  run_thread_.join();
  printf("JOINED ELASWrapper thread!\n");
}

void ELASWrapper::run() {
  std::cout << "Start ELASWrapper Thread" << std::endl;
  while (running_) {
    boost::unique_lock<boost::mutex> lk_elas3dfq(elas3d_frame_queue_mutex_);
    if (elas3d_frame_queue_.empty()) {
      lk_elas3dfq.unlock();
      usleep(5000);
      continue;
    }
    Elas3DFrame *cur_frame = elas3d_frame_queue_.front();
    elas3d_frame_queue_.pop();

    // for Pangolin visualization
    std::vector<Eigen::Vector3d> elas3d_pts3d = cur_frame->pts3d;
    auto elas3d_pts4d = cur_frame->pts4d;
    auto elas3d_pts4d_color = cur_frame->pts4d_color;

    elas3d_frames_.emplace_back(cur_frame);
    // Connection to previous keyframe
    if (elas3d_frames_.size() > 1) {
      auto prv_frame = elas3d_frames_[elas3d_frames_.size() - 2];
      dso::SE3 tfm_prv_cur =
          (prv_frame->tfm_w_c.inverse() * cur_frame->tfm_w_c);
    }

    std::vector<std::pair<int, Eigen::Vector3d>> pts3d_color;

    for (int i = 0; i < elas3d_pts4d_color.size(); i++) {
      Eigen::Vector3d p_g = cur_frame->tfm_w_c.matrix3x4() * elas3d_pts4d_color[i].second;
      pts_nearby_.emplace_back(std::pair<int, Eigen::Vector3d>(cur_frame->kf_id, p_g));
      std::cout<<elas3d_pts4d_color[i].first<<std::endl;
      pts3d_color.emplace_back(std::pair<int, Eigen::Vector3d>(elas3d_pts4d_color[i].first, p_g));
    }

    if (pangolin_viewer_) {
      pangolin_viewer_->refreshElasPtsData(pts3d_color,
                                           pts3d_color.size());
    }
    delete cur_frame->fh0;
    delete cur_frame->fh1;
    usleep(5000);
  }
  std::cout << "Finished Loop Thread" << std::endl;
}

void ELASWrapper::publish_keyframes(dso::FrameHessian *fh0, dso::FrameHessian *fh1, dso::CalibHessian *h_calib) {
  int prv_id = cur_id_;

  // keep id increasing
  if (prv_id >= fh0->frameID) {
    return;
  }

  cur_id_ = fh0->frameID;
  dso::SE3 cur_wc = fh0->shell->camToWorld;
  float fx = h_calib->fxl();
  float fy = h_calib->fyl();
  float cx = h_calib->cxl();
  float cy = h_calib->cyl();

  /* ====================== Extract points ================================ */
  for (dso::PointHessian *p: fh0->pointHessiansMarginalized) {
    Eigen::Vector4d p_l((p->u - cx) / fx / p->idepth_scaled,
                        (p->v - cy) / fy / p->idepth_scaled,
                        1 / p->idepth_scaled, 1);
    Eigen::Vector3d p_g = cur_wc.matrix3x4() * p_l;
    pts_nearby_.emplace_back(std::pair<int, Eigen::Vector3d>(cur_id_, p_g));
    
      std::cout << p_g << std::endl;
  }  
  id_pose_wc_[cur_id_] = cur_wc.log();

  auto *cur_frame = new Elas3DFrame(fh0, fh1, {fx, fy, cx, cy});
  boost::unique_lock<boost::mutex> lk_elas3dfq(elas3d_frame_queue_mutex_);
  elas3d_frame_queue_.emplace(cur_frame);
}

void ELASWrapper::update_stereo_model(const sensor_msgs::CameraInfoConstPtr &l_info_msg,
                                      const sensor_msgs::CameraInfoConstPtr &r_info_msg) {
  model_.fromCameraInfo(l_info_msg, r_info_msg);
}

void ELASWrapper::preprocess(Eigen::Vector3f *dI,
                             const uint8_t *out_image_data,
                             int w, int h) {
  cv::Mat imgf3(h, w, CV_32FC3, dI);
  std::vector<cv::Mat> imgf;
  cv::split(imgf3, imgf);
  cv::Mat img(h, w, CV_8UC1);
  imgf[0].convertTo(img, CV_8UC1);
  out_image_data = img.data;
}

void
ELASWrapper::process(Eigen::Vector3f *dI0,
                     Eigen::Vector3f *dI1,
                     std::vector<Eigen::Vector4d> &pts,
                     std::vector<std::pair<int, Eigen::Vector4d>> &pts_color) {
  int w = dso::wG[0], h = dso::hG[0];
  uint8_t *l_image_data = new uint8_t[w * h];
  uint8_t *r_image_data = new uint8_t[w * h];
  preprocess(dI0, l_image_data, w, h);
  preprocess(dI1, r_image_data, w, h);

  // Allocate
  const int32_t dims[3] = {w, h, static_cast<int32_t>(sizeof(uint8_t) * w)};
  float *l_disp_data = new float[w * h * sizeof(float)];
  float *r_disp_data = new float[w * h * sizeof(float)];

  // Process
  elas_->process(l_image_data, r_image_data, l_disp_data, r_disp_data, dims);

  std::vector<int32_t> inliers;
  for (int32_t i = 0; i < w * h; i++) {
    if (l_disp_data[i] > 0) // 记录视差为有效值
      inliers.push_back(i);
  }

  compute_pc(l_image_data, l_disp_data, pts, pts_color, inliers, w, h);
}

inline void ELASWrapper::compute_pc(uint8_t *l_image_data, float *l_disp_data,
                                    std::vector<Eigen::Vector4d> &pts,
                                    std::vector<std::pair<int, Eigen::Vector4d>> &pts_color,
                                    const std::vector<int32_t> &inliers,
                                    int32_t l_width, int32_t l_height) {

  for (size_t i = 0; i < inliers.size(); i++) {
    cv::Point2d left_uv;
    int32_t index = inliers[i];
#ifdef DOWN_SAMPLE
    left_uv.x = (index % l_width) * 2;
    left_uv.y = (index / l_width) * 2;
#else
    left_uv.x = index % l_width;
    left_uv.y = index / l_width;
#endif
    cv::Point3d point;
    model_.projectDisparityTo3d(left_uv, l_disp_data[index], point);

    Eigen::Vector4d v;
    v << point.x, point.y, point.z, 1;
    pts.push_back(v);
    std::pair<int, Eigen::Vector4d> v_color;
    v_color.first = l_image_data[index];
    v_color.second = v;
    std::cout<< v_color.first << v_color.second << std::endl;
    pts_color.push_back(v_color);
  }
}

void
ELASWrapper::process(const sensor_msgs::ImageConstPtr &l_image_msg,
                     const sensor_msgs::ImageConstPtr &r_image_msg) {
  // Have a synchronised pair of images, now to process using elas
  // convert images if necessary
  uint8_t *l_image_data, *r_image_data;
  int32_t l_step, r_step;
  cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
  if (l_image_msg->encoding == sensor_msgs::image_encodings::MONO8) {
    l_image_data = const_cast<uint8_t *>(&(l_image_msg->data[0]));
    l_step = l_image_msg->step;
  } else {
    l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    l_image_data = l_cv_ptr->image.data;
    l_step = l_cv_ptr->image.step[0];
  }
  if (r_image_msg->encoding == sensor_msgs::image_encodings::MONO8) {
    r_image_data = const_cast<uint8_t *>(&(r_image_msg->data[0]));
  } else {
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    r_image_data = r_cv_ptr->image.data;
  }

#ifdef DOWN_SAMPLE
  int32_t width = l_image_msg->width / 2;
    int32_t height = l_image_msg->height / 2;
#else
  int32_t width = l_image_msg->width;
  int32_t height = l_image_msg->height;
#endif

  // Allocate
  const int32_t dims[3] = {static_cast<int32_t>(l_image_msg->width), static_cast<int32_t>(l_image_msg->height), l_step};
  float *l_disp_data = new float[width * height * sizeof(float)];
  float *r_disp_data = new float[width * height * sizeof(float)];

  // Process
  elas_->process(l_image_data, r_image_data, l_disp_data, r_disp_data, dims);

  std::vector<int32_t> inliers;
  for (int32_t i = 0; i < width * height; i++) {
    if (l_disp_data[i] > 0) // 记录视差为有效值
      inliers.push_back(i);
  }

  generate_pc(l_image_msg, l_disp_data, inliers, width, height);
}

void ELASWrapper::generate_pc(const sensor_msgs::ImageConstPtr &l_image_msg,
                              float *l_disp_data,
                              const std::vector<int32_t> &inliers,
                              int32_t l_width, int32_t l_height) {

  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
  // Copy into the data
  for (int32_t u = 0; u < l_width; u++) {
    for (int32_t v = 0; v < l_height; v++) {
      int index = v * l_width + u;
#ifdef DOWN_SAMPLE
      cv::Vec3b col = cv_ptr->image.at<cv::Vec3b>(v * 2, u * 2);
#else
      cv::Vec3b col = cv_ptr->image.at<cv::Vec3b>(v, u);
#endif
    }
  }

  std::vector<Eigen::Vector3d> pts;

  for (size_t i = 0; i < inliers.size(); i++) {
    cv::Point2d left_uv;
    int32_t index = inliers[i];
#ifdef DOWN_SAMPLE
    left_uv.x = (index % l_width) * 2;
    left_uv.y = (index / l_width) * 2;
#else
    left_uv.x = index % l_width;
    left_uv.y = index / l_width;
#endif
    cv::Point3d point;
    model_.projectDisparityTo3d(left_uv, l_disp_data[index], point);

    Eigen::Matrix<double, 3, 1> v;
    v << point.x, point.y, point.z;
    pts.push_back(v);
  }
}

void ELASWrapper::save_pc() {

}

void ELASWrapper::extract_pts(Elas3DFrame *elas3D_frame) {
  std::vector<Eigen::Vector4d> pts;
  std::vector<std::pair<int, Eigen::Vector4d>> pts_color;
  process(elas3D_frame->fh0->dI, elas3D_frame->fh1->dI, pts, pts_color);
  elas3D_frame->pts4d = pts;
  elas3D_frame->pts4d_color = pts_color;
}
