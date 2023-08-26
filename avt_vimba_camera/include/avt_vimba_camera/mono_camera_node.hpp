/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef MONO_CAMERA_H
#define MONO_CAMERA_H

#include <string>
#include <memory>

#include <dds/pub/ddspub.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/domain/ddsdomain.hpp> 

#include <sensor_msgs/msg/ImagePlugin.hpp>
#include <spdlog/logger.h>
#include <yaml-cpp/yaml.h>

#include "avt_vimba_camera/avt_vimba_camera.hpp"
#include "avt_vimba_camera/avt_vimba_api.hpp"


namespace avt_vimba_camera
{
class MonoCameraNode
{
public:
  explicit MonoCameraNode(size_t domain_id, const std::string &param_fp);
  ~MonoCameraNode();
  void start();

private:
  AvtVimbaApi api_;
  AvtVimbaCamera cam_;

  std::string ip_;
  std::string guid_;
  std::string camera_info_url_;
  std::string frame_id_;
  bool use_measurement_time_;
  int32_t ptp_offset_;

  dds::domain::DomainParticipant participant_;
  dds::topic::Topic<sensor_msgs::msg::dds_::Image_> data_topic_;
  dds::pub::DataWriter<sensor_msgs::msg::dds_::Image_> data_wr_;

  std::shared_ptr<spdlog::logger> console_;
  YAML::Node allparms_;

  void loadParams(const std::string &param_fp);
  void frameCallback(const FramePtr& vimba_frame_ptr);
};
}  // namespace avt_vimba_camera
#endif
