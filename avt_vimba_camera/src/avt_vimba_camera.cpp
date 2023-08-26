/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearican Islands
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
///       Systems, Robotics and Vision Group, Univ. of the Balearican Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearican Islands nor the names of its contributors may be used
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

#include "avt_vimba_camera/avt_vimba_camera.hpp"
#include "VimbaC/Include/VimbaC.h"
#include "avt_vimba_camera/avt_vimba_api.hpp"

#include <signal.h>
#include <yaml-cpp/yaml.h>
#include <thread>

namespace avt_vimba_camera
{
static volatile int keepRunning = 1;

AvtVimbaCamera::AvtVimbaCamera()
  : api_()
{
  opened_ = false;     // camera connected to the api
  streaming_ = false;  // capturing frames
  on_init_ = true;     // on initialization phase
  on_init_config_ = false;
  force_stopped_ = false;
  camera_state_ = OPENING;
}

void AvtVimbaCamera::start(const std::string& ip_str, const std::string& guid_str, const std::string& frame_id,
                           const std::string& camera_info_url)
{
  if (opened_)
    return;

  frame_id_ = frame_id;
  spdlog::get("console")->info("Starting device with IP: {} or GUID: {}", ip_str, guid_str);

  // Determine which camera to use. Try IP first
  if (!ip_str.empty())
  {
    spdlog::get("console")->info("Trying to open camera by IP: {}", ip_str);
    vimba_camera_ptr_ = openCamera(ip_str);
    if (!vimba_camera_ptr_)
    {
      spdlog::get("console")->warn("Camera pointer is empty. Returning...");
      return;
    }
    guid_ = ip_str;
    // If both guid and IP are available, open by IP and check guid
    if (!guid_str.empty())
    {
      std::string cam_guid_str;
      vimba_camera_ptr_->GetSerialNumber(cam_guid_str);
      if (!vimba_camera_ptr_)
      {
        spdlog::get("console")->warn("Camera pointer is empty. Returning...");
        return;
      }
      assert(cam_guid_str == guid_str);
      guid_ = guid_str;
      spdlog::get("console")->info("GUID {} matches for camera with IP: {}", cam_guid_str, ip_str);
    }
  }
  else if (!guid_str.empty())
  {
    // Only guid available
    spdlog::get("console")->info("Trying to open camera by ID: {}", guid_str);
    vimba_camera_ptr_ = openCamera(guid_str);
    guid_ = guid_str;
  }
  else
  {
    // No identifying info (GUID and IP) are available
    spdlog::get("console")->error("Can't connect to the camera: at least GUID or IP need to be set.");
    camera_state_ = ERROR;
  }

  getFeatureValue("GevTimestampTickFrequency", vimba_timestamp_tick_freq_);

  // From the SynchronousGrab API example:
  // TODO Set the GeV packet size to the highest possible value
  VmbInterfaceType cam_int_type;
  vimba_camera_ptr_->GetInterfaceType(cam_int_type);
  if (cam_int_type == VmbInterfaceEthernet)
  {
    runCommand("GVSPAdjustPacketSize");
  }

  std::string trigger_source;
  getFeatureValue("TriggerSource", trigger_source);

  SP_SET(frame_obs_ptr_,
          new FrameObserver(vimba_camera_ptr_,
                            std::bind(&avt_vimba_camera::AvtVimbaCamera::frameCallback, this, std::placeholders::_1)));
  spdlog::get("console")->info("Ready to receive frames triggered by {}", trigger_source);
  camera_state_ = IDLE;
  initConfig();
}

void AvtVimbaCamera::stop()
{
  if (!opened_)
    return;
  vimba_camera_ptr_->Close();
  opened_ = false;
}

void AvtVimbaCamera::startImaging()
{
  if (!streaming_)
  {
    // Start streaming
    VmbErrorType err = vimba_camera_ptr_->StartContinuousImageAcquisition(3, IFrameObserverPtr(frame_obs_ptr_));
    if (err == VmbErrorSuccess)
    {
      spdlog::get("console")->info("Starting continuous image acquisition ...");
      streaming_ = true;
      camera_state_ = OK;
    }
    else
    {
      spdlog::get("console")->error("Could not start continuous image acquisition. Error: {}",
                                    api_.errorCodeToMessage(err));
      camera_state_ = ERROR;
    }
  }
  else
  {
    spdlog::get("console")->warn("Start imaging called, but the camera is already imaging.");
  }
}

void AvtVimbaCamera::stopImaging()
{
  if (streaming_ || on_init_)
  {
    VmbErrorType err = vimba_camera_ptr_->StopContinuousImageAcquisition();
    if (err == VmbErrorSuccess)
    {
      spdlog::get("console")->info("Acquisition stoppped ...");
      streaming_ = false;
      camera_state_ = IDLE;
    }
    else
    {
      spdlog::get("console")->error("Could not stop image acquisition. Error: {}", api_.errorCodeToMessage(err));
      camera_state_ = ERROR;
    }
  }
  else
  {
    spdlog::get("console")->warn("Stop imaging called, but the camera is already stopped.");
  }
}

CameraPtr AvtVimbaCamera::openCamera(const std::string& id_str)
{
  // Details:   The ID might be one of the following:
  //            "IP:169.254.12.13",
  //            "MAC:000f31000001",
  //            or a plain serial number: "1234567890".

  CameraPtr camera;
  VimbaSystem& vimba_system(VimbaSystem::GetInstance());

  // get camera
  VmbErrorType err = vimba_system.GetCameraByID(id_str.c_str(), camera);
  while (err != VmbErrorSuccess)
  {
    if (keepRunning)
    {
      spdlog::get("console")->warn("Could not find camera using {}. Retrying every two seconds ...", id_str);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      err = vimba_system.GetCameraByID(id_str.c_str(), camera);
    }
    else
    {
      spdlog::get("console")->error("Could not find camera using {}. Error: {}", id_str,
                                    api_.errorCodeToMessage(err));
      camera_state_ = CAMERA_NOT_FOUND;
      return camera;
    }
  }

  // open camera
  err = camera->Open(VmbAccessModeFull);
  while (err != VmbErrorSuccess && keepRunning)
  {
    if (keepRunning)
    {
      spdlog::get("console")->warn("Could not open camera. Retrying every two seconds ...");
      err = camera->Open(VmbAccessModeFull);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else
    {
      spdlog::get("console")->error("Could not open camera. Error: {}", api_.errorCodeToMessage(err));
      camera_state_ = CAMERA_NOT_FOUND;
      return camera;
    }
  }

  std::string cam_id, cam_name;
  camera->GetID(cam_id);
  camera->GetName(cam_name);
  spdlog::get("console")->info("Opened connection to camera named {} with ID {}", cam_name, cam_id);

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  opened_ = true;
  camera_state_ = IDLE;
  return camera;
}

void AvtVimbaCamera::frameCallback(const FramePtr vimba_frame_ptr)
{
  std::unique_lock<std::mutex> lock(config_mutex_);
  camera_state_ = OK;

  // Call the callback implemented by other classes
  std::thread thread_callback = std::thread(userFrameCallback, vimba_frame_ptr);
  thread_callback.join();
}

CameraState AvtVimbaCamera::getCameraState() const
{
  return camera_state_;
}

double AvtVimbaCamera::getTimestamp()
{
  double timestamp = -1.0;
  if (runCommand("GevTimestampControlLatch"))
  {
    VmbInt64_t freq, ticks;
    getFeatureValue("GevTimestampTickFrequency", freq);
    getFeatureValue("GevTimestampValue", ticks);
    timestamp = static_cast<double>(ticks) / static_cast<double>(freq);
  }
  return timestamp;
}

double AvtVimbaCamera::getDeviceTemp()
{
  double temp = -1.0;
  if (setFeatureValue("DeviceTemperatureSelector", "Main") == VmbErrorSuccess)
  {
    getFeatureValue("DeviceTemperature", temp);
  }
  return temp;
}

int AvtVimbaCamera::getImageWidth()
{
  int width = -1;
  getFeatureValue("Width", width);
  return width;
}

int AvtVimbaCamera::getImageHeight()
{
  int height = -1;
  getFeatureValue("Height", height);
  return height;
}

int AvtVimbaCamera::getSensorWidth()
{
  int sensor_width = -1;
  getFeatureValue("SensorWidth", sensor_width);
  return sensor_width;
}

int AvtVimbaCamera::getSensorHeight()
{
  int sensor_height = -1;
  getFeatureValue("SensorHeight", sensor_height);
  return sensor_height;
}

int AvtVimbaCamera::getBinningOrDecimationX()
{
  int binning = -1;
  int decimation = -1;
  getFeatureValue("BinningHorizontal", binning);
  getFeatureValue("DecimationHorizontal", decimation);

  return std::max(binning, decimation);
}

int AvtVimbaCamera::getBinningOrDecimationY()
{
  int binning = -1;
  int decimation = -1;
  getFeatureValue("BinningVertical", binning);
  getFeatureValue("DecimationVertical", decimation);

  return std::max(binning, decimation);
}

double AvtVimbaCamera::getTimestampRealTime(VmbUint64_t timestamp_ticks)
{
  return (static_cast<double>(timestamp_ticks)) / (static_cast<double>(vimba_timestamp_tick_freq_));
}

// Template function to SET a feature value from the camera
template <typename T>
VmbErrorType AvtVimbaCamera::setFeatureValue(const std::string& feature_str, const T& val)
{
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(), vimba_feature_ptr);
  if (err == VmbErrorSuccess)
  {
    bool writable;
    err = vimba_feature_ptr->IsWritable(writable);
    if (err == VmbErrorSuccess)
    {
      if (writable)
      {
        spdlog::get("console")->debug("Setting feature {} value {}", feature_str, val);
        VmbFeatureDataType data_type;
        err = vimba_feature_ptr->GetDataType(data_type);
        if (err == VmbErrorSuccess)
        {
          if (data_type == VmbFeatureDataEnum)
          {
            bool available;
            err = vimba_feature_ptr->IsValueAvailable(val, available);
            if (err == VmbErrorSuccess)
            {
              if (available)
              {
                err = vimba_feature_ptr->SetValue(val);
              }
              else
              {
                spdlog::get("console")->warn("Feature {} is available now", feature_str);
              }
            }
            else
            {
              spdlog::get("console")->warn("Feature: value unavailable\n\tERROR {}", feature_str, api_.errorCodeToMessage(err));
            }
          }
          else
          {
            err = vimba_feature_ptr->SetValue(val);
          }
        }
        else
        {
          spdlog::get("console")->warn("Feature {}: Bad data type\n\tERROR {}", feature_str, api_.errorCodeToMessage(err));
        }
      }
      else
      {
        spdlog::get("console")->warn("Feature {} is not writable.", feature_str);
      }
    }
    else
    {
      spdlog::get("console")->warn("Feature {}: ERROR {}", feature_str, api_.errorCodeToMessage(err));
    }
  }
  else
  {
    spdlog::get("console")->warn("Could not get feature {}, your camera probably doesn't support it.", feature_str);
  }
  return err;
}

// Template function to GET a feature value from the camera
template <typename T>
bool AvtVimbaCamera::getFeatureValue(const std::string& feature_str, T& val)
{
  spdlog::get("console")->debug("Asking for feature {}", feature_str);
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(), vimba_feature_ptr);
  if (err == VmbErrorSuccess)
  {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable)
    {
      vimba_feature_ptr->GetDataType(data_type);
      if (err != VmbErrorSuccess)
      {
        spdlog::get("console")->error("[Could not get feature Data Type. Error code: {}]", err);
      }
      else
      {
        switch (data_type)
        {
          case VmbFeatureDataBool:
            bool bValue;
            err = vimba_feature_ptr->GetValue(bValue);
            if (err == VmbErrorSuccess)
            {
              val = static_cast<T>(bValue);
            }
            break;
          case VmbFeatureDataFloat:
            double fValue;
            err = vimba_feature_ptr->GetValue(fValue);
            if (err == VmbErrorSuccess)
            {
              val = static_cast<T>(fValue);
            }
            break;
          case VmbFeatureDataInt:
            VmbInt64_t nValue;
            err = vimba_feature_ptr->GetValue(nValue);
            if (err == VmbErrorSuccess)
            {
              val = static_cast<T>(nValue);
            }
            break;
          default:
            break;
        }
        if (err != VmbErrorSuccess)
        {
          spdlog::get("console")->warn("Could not get feature value. Error code: {}", api_.errorCodeToMessage(err));
        }
      }
    }
    else
    {
      spdlog::get("console")->warn("Feature {} is not readable.", feature_str);
    }
  }
  else
  {
    spdlog::get("console")->warn("Could not get feature {}", feature_str);
  }
  return (err == VmbErrorSuccess);
}

// Function to GET a feature value from the camera, overloaded to strings
bool AvtVimbaCamera::getFeatureValue(const std::string& feature_str, std::string& val)
{
  spdlog::get("console")->debug("Asking for feature {}", feature_str);
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(), vimba_feature_ptr);
  if (err == VmbErrorSuccess)
  {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable)
    {
      vimba_feature_ptr->GetDataType(data_type);
      if (err != VmbErrorSuccess)
      {
        spdlog::get("console")->error("[Could not get feature Data Type. Error code: {}]", err);
      }
      else
      {
        std::string strValue;
        switch (data_type)
        {
          case VmbFeatureDataEnum:
            err = vimba_feature_ptr->GetValue(strValue);
            if (err == VmbErrorSuccess)
            {
              val = strValue;
            }
            break;
          case VmbFeatureDataString:
            err = vimba_feature_ptr->GetValue(strValue);
            if (err == VmbErrorSuccess)
            {
              val = strValue;
            }
            break;
          default:
            break;
        }
        if (err != VmbErrorSuccess)
        {
          spdlog::get("console")->warn("Could not get feature value. Error code: {}", api_.errorCodeToMessage(err));
        }
      }
    }
    else
    {
      spdlog::get("console")->warn("Feature {} is not readable.", feature_str);
    }
  }
  else
  {
    spdlog::get("console")->warn("Could not get feature {}", feature_str);
  }
  return (err == VmbErrorSuccess);
}

// Tries to configure a camera feature.
// Updates the config value passed in if a different config is in use by the camera.
template <typename Vimba_Type>
void AvtVimbaCamera::configureFeature(const std::string& feature_str, const Vimba_Type& val_in)
{
  Vimba_Type actual_value;

  VmbErrorType return_value = setFeatureValue(feature_str, val_in);
  if (return_value == VmbErrorSuccess || return_value == VmbErrorInvalidValue)
  {
    getFeatureValue(feature_str, actual_value);
    if (val_in == actual_value)
    {
      spdlog::get("console")->info(" - {} set to {}", feature_str, actual_value);
    }
    else
    {
      spdlog::get("console")->warn(" - Tried to set {} to {} but the camera used {} instead", feature_str, val_in,
                                   actual_value);
    }
  }
  else
  {
    spdlog::get("console")->error(" - Failed to set {} to {}", feature_str, actual_value);
  }
}

// Overloaded for strings, template specialization doesn't currently compile with GCC
void AvtVimbaCamera::configureFeature(const std::string& feature_str, const std::string& val_in)
{
  std::string actual_value;

  VmbErrorType return_value = setFeatureValue(feature_str, val_in.c_str());
  if (return_value == VmbErrorSuccess || return_value == VmbErrorInvalidValue)
  {
    getFeatureValue(feature_str, actual_value);
    if (val_in == actual_value)
    {
      spdlog::get("console")->info(" - {} set to {}", feature_str, actual_value);
    }
    else
    {
      spdlog::get("console")->warn(" - Tried to set {} to {} but the camera used {} instead", feature_str, val_in,
                                   actual_value);
    }
  }
  else
  {
    spdlog::get("console")->error(" - Failed to set {} to {}", feature_str, actual_value);
  }
}

// Template function to RUN a command
bool AvtVimbaCamera::runCommand(const std::string& command_str)
{
  FeaturePtr feature_ptr;
  VmbErrorType err = vimba_camera_ptr_->GetFeatureByName(command_str.c_str(), feature_ptr);
  if (err == VmbErrorSuccess)
  {
    err = feature_ptr->RunCommand();
    if (err == VmbErrorSuccess)
    {
      bool is_command_done = false;
      do
      {
        err = feature_ptr->IsCommandDone(is_command_done);
        if (err != VmbErrorSuccess)
        {
          break;
        }
        spdlog::get("console")->debug("Waiting for command {}...", command_str);;
      } while (false == is_command_done);
      spdlog::get("console")->debug("Command {} done!", command_str);
      return true;
    }
    else
    {
      spdlog::get("console")->warn("Could not run command {}. Error: {}", command_str, api_.errorCodeToMessage(err));
      return false;
    }
  }
  else
  {
    spdlog::get("console")->warn("Could not get feature command {}. Error: {}", command_str,
                                 api_.errorCodeToMessage(err));
    return false;
  }
}

void AvtVimbaCamera::initConfig()
{
  if (!opened_)
  {
    spdlog::get("console")->error("Can't configure camera. It needs to be opened first");
    return;
  }

  VmbErrorType err;
  uint32_t writable_count = 0;
  on_init_config_ = true;
}


bool AvtVimbaCamera::setParams(const YAML::Node& params)
{
  FeaturePtrVector features;
  VmbErrorType err;
  err = vimba_camera_ptr_->GetFeatures(features);
  if (err == VmbErrorSuccess)
  {
    on_init_config_ = true;
    spdlog::get("console")->info("Configuring camera:");
    // Query all camera features to translate into params
    for (const FeaturePtr feature : features)
    {
      std::string feature_name = "";
      bool is_writable = false;
      err = feature->GetName(feature_name);
      if (err != VmbErrorSuccess)
      {
        spdlog::get("console")->error("[Could not get feature Name. Error code: {}]", err);
        return false;
      }

      err = feature->IsWritable(is_writable);
      if (err != VmbErrorSuccess)
      {
        spdlog::get("console")->error("[Could not get write access. Error code: {}]", err);
        return false;
      }
      writable_features_[feature_name] = is_writable;

      VmbFeatureDataType type;
      err = feature->GetDataType(type);
      if (err != VmbErrorSuccess)
      {
        spdlog::get("console")->error("[Could not get feature Data Type. Error code: {}]", err);
        return false;
      }
      else
      {
        switch (type)
        {
          case VmbFeatureDataBool: {
            configureFeature(feature_name, params[PARAM_NAMESPACE + feature_name].as<bool>());
            break;
          }
          case VmbFeatureDataInt: {
            configureFeature(feature_name, params[PARAM_NAMESPACE + feature_name].as<int>());
            break;
          }
          case VmbFeatureDataFloat: {
            configureFeature(feature_name, params[PARAM_NAMESPACE + feature_name].as<double>());
            break;
          }
          case VmbFeatureDataString: {
            configureFeature(feature_name, params[PARAM_NAMESPACE + feature_name].as<std::string>());
            break;
          }
          case VmbFeatureDataEnum: {
            configureFeature(feature_name, params[PARAM_NAMESPACE + feature_name].as<std::string>());
            break;
          }
          default: {
            break;
          }
        }
      }
    }
  }
  else
  {
    spdlog::get("console")->error("Could not get features. Error code: {}", api_.errorCodeToMessage(err));
    return false;
  }
  return true;
}

bool AvtVimbaCamera::loadCameraSettings(const std::string& filename)
{
  stopImaging();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  vimba_camera_ptr_->LoadSaveSettingsSetup(VmbFeaturePersistNoLUT, 5, 4);
  auto err = vimba_camera_ptr_->LoadCameraSettings(filename);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  startImaging();

  if (err != VmbErrorSuccess)
  {
    spdlog::get("console")->error("Failed to load camera settings from {}", filename);
    return false;
  }
  spdlog::get("console")->info("Loaded camera settings from {}", filename);
  return true;
}

bool AvtVimbaCamera::saveCameraSettings(const std::string& filename)
{
  stopImaging();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  vimba_camera_ptr_->LoadSaveSettingsSetup(VmbFeaturePersistNoLUT, 5, 4);
  auto err = vimba_camera_ptr_->SaveCameraSettings(filename);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  startImaging();

  if (err != VmbErrorSuccess)
  {
    spdlog::get("console")->error("Failed to save camera settings to {}", filename);
    return false;
  }
  spdlog::get("console")->info("Saved camera settings to {}", filename);
  return true;
}

}  // namespace avt_vimba_camera
