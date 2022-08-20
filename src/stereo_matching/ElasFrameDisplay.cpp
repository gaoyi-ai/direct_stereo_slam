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

// This file is modified from <https://github.com/JakobEngel/dso>

#include "util/settings.h"
#include <stdio.h>

//#include <GL/glx.h>
//#include <GL/gl.h>
//#include <GL/glu.h>

#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"
#include "ElasFrameDisplay.h"
#include "util/FrameShell.h"
#include <pangolin/pangolin.h>

ElasFrameDisplay::ElasFrameDisplay() {
  original_input_ = 0;
  num_buffer_size_ = 0;
  num_points_ = 0;

  id_ = 0;
  active_ = true;
  tfm_w_c_ = dso::SE3();

  need_refresh_ = true;

  my_scaled_th_ = 0.001;
  my_abs_th_ = 0.001;
  my_displayMode_ = 1;
  my_min_rel_bs_ = 0.1;
  my_sparsify_factor_ = 1;

  num_gl_buffer_points_ = 0;
  buffer_valid_ = false;
}

void ElasFrameDisplay::setFromKF(Elas3DFrame *fh) {
  id_ = fh->kf_id;
  width_ = dso::wG[0];
  height_ = dso::hG[0];

  // add all traces, inlier and outlier points.
  int npoints = fh->pts4d_color.size();

  if (num_buffer_size_ < npoints) {
    if (original_input_ != 0)
      delete original_input_;
    num_buffer_size_ = npoints + 100;
    original_input_ =
        new dso::IOWrap::InputPoint<1>[num_buffer_size_];
  }

  dso::IOWrap::InputPoint<1> *pc = original_input_;
  num_points_ = 0;
  for (const auto &p_color: fh->pts4d_color){
    pc[num_points_].color[0] = p_color.first;
    pc[num_points_].x = p_color.second[0];
    pc[num_points_].y = p_color.second[1];
    pc[num_points_].z = p_color.second[2];
    num_points_++;
  }
  assert(num_points_ <= npoints);

  tfm_w_c_ = fh->tfm_w_c;
  need_refresh_ = true;
}

ElasFrameDisplay::~ElasFrameDisplay() {
  if (original_input_ != 0)
    delete[] original_input_;
}

void ElasFrameDisplay::refreshPC() {
  if (!need_refresh_)
    return;
  need_refresh_ = false;

  // if there are no vertices, done!
  if (num_points_ == 0)
    return;

  // make data
  dso::Vec3f *tmpVertexBuffer = new dso::Vec3f[num_points_];
  dso::Vec3b *tmpColorBuffer = new dso::Vec3b[num_points_];
  int vertexBufferNumPoints = 0;

  for (int i = 0; i < num_points_; i++) {
    /* display modes:
     * my_displayMode_==0 - all pts, color-coded
     * my_displayMode_==1 - normal points
     * my_displayMode_==2 - active only
     * my_displayMode_==3 - nothing
     */

//    if (my_displayMode_ == 1 && original_input_[i].status != 1 &&
//        original_input_[i].status != 2)
//      continue;
    if (my_displayMode_ == 2 && original_input_[i].status != 1)
      continue;
    if (my_displayMode_ > 2)
      continue;

    if (original_input_[i].z < 0)
      continue;

    float depth = original_input_[i].z;
    float depth4 = depth * depth;
    depth4 *= depth4;

    if (original_input_[i].relObsBaseline < my_min_rel_bs_)
      continue;

    for (int pnt = 0; pnt < 1; pnt++) {

      if (my_sparsify_factor_ > 1 && rand() % my_sparsify_factor_ != 0)
        continue;

      tmpVertexBuffer[vertexBufferNumPoints][0] = original_input_[i].x;
      tmpVertexBuffer[vertexBufferNumPoints][1] = original_input_[i].y;
      tmpVertexBuffer[vertexBufferNumPoints][2] = original_input_[i].z;

      if (my_displayMode_ == 0) {
        if (original_input_[i].status == 0) {
          tmpColorBuffer[vertexBufferNumPoints][0] = 0;
          tmpColorBuffer[vertexBufferNumPoints][1] = 255;
          tmpColorBuffer[vertexBufferNumPoints][2] = 255;
        } else if (original_input_[i].status == 1) {
          tmpColorBuffer[vertexBufferNumPoints][0] = 0;
          tmpColorBuffer[vertexBufferNumPoints][1] = 255;
          tmpColorBuffer[vertexBufferNumPoints][2] = 0;
        } else if (original_input_[i].status == 2) {
          tmpColorBuffer[vertexBufferNumPoints][0] = 0;
          tmpColorBuffer[vertexBufferNumPoints][1] = 0;
          tmpColorBuffer[vertexBufferNumPoints][2] = 255;
        } else if (original_input_[i].status == 3) {
          tmpColorBuffer[vertexBufferNumPoints][0] = 255;
          tmpColorBuffer[vertexBufferNumPoints][1] = 0;
          tmpColorBuffer[vertexBufferNumPoints][2] = 0;
        } else {
          tmpColorBuffer[vertexBufferNumPoints][0] = 255;
          tmpColorBuffer[vertexBufferNumPoints][1] = 255;
          tmpColorBuffer[vertexBufferNumPoints][2] = 255;
        }

      } else {
        tmpColorBuffer[vertexBufferNumPoints][0] =
            original_input_[i].color[pnt];
        tmpColorBuffer[vertexBufferNumPoints][1] =
            original_input_[i].color[pnt];
        tmpColorBuffer[vertexBufferNumPoints][2] =
            original_input_[i].color[pnt];
      }
      vertexBufferNumPoints++;

      assert(vertexBufferNumPoints <= num_points_);
    }
  }

  if (vertexBufferNumPoints == 0) {
    delete[] tmpColorBuffer;
    delete[] tmpVertexBuffer;
    return;
  }

  num_gl_buffer_good_points_ = vertexBufferNumPoints;
  if (num_gl_buffer_good_points_ > num_gl_buffer_points_) {
    num_gl_buffer_points_ = vertexBufferNumPoints * 1.3;
    vertex_buffer_.Reinitialise(pangolin::GlArrayBuffer, num_gl_buffer_points_,
                                GL_FLOAT, 3, GL_DYNAMIC_DRAW);
    color_buffer_.Reinitialise(pangolin::GlArrayBuffer, num_gl_buffer_points_,
                               GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);
  }
  vertex_buffer_.Upload(tmpVertexBuffer,
                        sizeof(float) * 3 * num_gl_buffer_good_points_, 0);
  color_buffer_.Upload(tmpColorBuffer,
                       sizeof(unsigned char) * 3 * num_gl_buffer_good_points_,
                       0);
  buffer_valid_ = true;
  delete[] tmpColorBuffer;
  delete[] tmpVertexBuffer;
}

void ElasFrameDisplay::drawPC(float pointSize) {

  if (!buffer_valid_ || num_gl_buffer_good_points_ == 0)
    return;

  glDisable(GL_LIGHTING);

  glPushMatrix();

  Sophus::Matrix4f m = tfm_w_c_.matrix().cast<float>();
  glMultMatrixf((GLfloat *) m.data());

  glPointSize(pointSize);

  color_buffer_.Bind();
  glColorPointer(color_buffer_.count_per_element, color_buffer_.datatype, 0, 0);
  glEnableClientState(GL_COLOR_ARRAY);

  vertex_buffer_.Bind();
  glVertexPointer(vertex_buffer_.count_per_element, vertex_buffer_.datatype, 0,
                  0);
  glEnableClientState(GL_VERTEX_ARRAY);
  glDrawArrays(GL_POINTS, 0, num_gl_buffer_good_points_);
  glDisableClientState(GL_VERTEX_ARRAY);
  vertex_buffer_.Unbind();

  glDisableClientState(GL_COLOR_ARRAY);
  color_buffer_.Unbind();

  glPopMatrix();
}
