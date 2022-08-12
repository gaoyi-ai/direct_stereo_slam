//
// Created by yi on 8/10/22.
//

#ifndef SRC_SLAMNODE_H
#define SRC_SLAMNODE_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Image.h>
#include <chrono>
#include "FrontEnd.h"
#include "loop_closure/LoopHandler.h"
#include "stereo_matching/ELASWrapper.h"

using namespace dso;

/*
 * 实现功能拓展
 * */
class SLAMNode {
private:
  double currentTimeStamp;
  int incomingId;
  FrontEnd *front_end_;
  Undistort *undistorter0_;
  Undistort *undistorter1_;

  // scale optimizer
  std::vector<double> tfm_stereo_;
  float scale_opt_thres_; // set to -1 to disable scale optimization

  // loop closure
  LoopHandler *loop_handler_;

  // stereo matching
  ELASWrapper *elas_wrapper_;


private:

  // statistics: time for each frame
  std::vector<std::chrono::duration<long, std::ratio<1, 1000000000>>>
      frame_tt_;

  void settingsDefault(int preset, int mode);

public:
  SLAMNode(const std::vector<double> &tfm_stereo, const std::string &calib0,
           const std::string &calib1, const std::string &vignette0,
           const std::string &vignette1, const std::string &gamma0,
           const std::string &gamma1, bool nomt, int preset, int mode,
           float scale_opt_thres, float lidar_range, float scan_context_thres);

  ~SLAMNode();

  void imageMessageCallback(const boost::shared_ptr<const ::sensor_msgs::Image> &msg0,
                            const boost::shared_ptr<const ::sensor_msgs::Image> &msg1);

  void imgInfoMsgCB(const sensor_msgs::CameraInfoConstPtr &l_info_msg,
                    const sensor_msgs::CameraInfoConstPtr &r_info_msg);

  void setElasWrapper(ELASWrapper *elasWrapper) {
    elas_wrapper_ = elasWrapper;
  }
};

inline void print_average(const std::string &name, const TimeVector &tm_vec) {
  float sum = 0;
  for (size_t i = 0; i < tm_vec.size(); i++) {
    sum += std::chrono::duration_cast<std::chrono::duration<double >>(tm_vec[i])
        .count();
  }

  printf("%s %.2f x %lu\n", name.c_str(), 1000 * sum / tm_vec.size(),
         tm_vec.size());
}

#endif //SRC_SLAMNODE_H
