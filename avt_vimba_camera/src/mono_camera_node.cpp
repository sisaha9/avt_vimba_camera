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

#include <thread>

#include <avt_vimba_camera/mono_camera_node.hpp>

#include "spdlog/sinks/stdout_color_sinks.h"

using namespace std::placeholders;

namespace avt_vimba_camera
{
MonoCameraNode::MonoCameraNode(size_t domain_id, const std::string &param_fp) : api_(), cam_(), participant_(dds::core::null), data_topic_(dds::core::null), data_wr_(dds::core::null)
{
  // Set the frame callback
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCameraNode::frameCallback, this, _1));
  console_ = spdlog::stdout_color_mt("console");
  loadParams(param_fp);
  participant_ = dds::domain::DomainParticipant(domain_id);
  dds::pub::qos::DataWriterQos writer_qos;
  writer_qos << dds::core::policy::Reliability::BestEffort();
  data_topic_ = dds::topic::Topic<sensor_msgs::msg::dds_::Image_>(participant_, "rt/" + frame_id_ + "/image");
  data_wr_ = dds::pub::DataWriter<sensor_msgs::msg::dds_::Image_>(dds::pub::Publisher(participant_), data_topic_, writer_qos);
}

MonoCameraNode::~MonoCameraNode()
{
  cam_.stop();
}

void MonoCameraNode::loadParams(const std::string &param_fp)
{
  allparms_ = YAML::LoadFile(param_fp);
  ip_ = allparms_["ip"].as<std::string>();
  guid_ = allparms_["guid"].as<std::string>();
  // camera_info_url_ = allparms_["camera_info_url"].as<std::string>();
  frame_id_ = allparms_["frame_id"].as<std::string>();
  use_measurement_time_ = allparms_["use_measurement_time"].as<bool>();
  ptp_offset_ = allparms_["ptp_offset"].as<int>();
  spdlog::get("console")->info("Loading parameters from file: {}");
}

void MonoCameraNode::start()
{
  // Start Vimba & list all available cameras
  api_.start();

  // Start camera
  cam_.start(ip_, guid_, frame_id_, camera_info_url_);
  cam_.startImaging();
}

void MonoCameraNode::frameCallback(const FramePtr& vimba_frame_ptr)
{
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);

  // need to see if there is an RTI equivalent
  // Need to add back something to do with camera info
  // if (camera_info_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::msg::dds_::Image_ img;
    if (api_.frameToImage(vimba_frame_ptr, img))
    {
      if (use_measurement_time_)
      {
        VmbUint64_t frame_timestamp;
        vimba_frame_ptr->GetTimestamp(frame_timestamp);
        img.header().stamp().sec(cam_.getTimestampRealTime(frame_timestamp) + ptp_offset_);
      }
      else
      {
        img.header().stamp().nanosec(ts.tv_nsec);
        img.header().stamp().sec((int32_t)ts.tv_sec);
      }

      img.header().frame_id(frame_id_);
      data_wr_.write(img);
    }
    else
    {
      spdlog::get("console")->warn("Function frameToImage returned 0. No image published.");
    }
  }
}
}  // namespace avt_vimba_camera
