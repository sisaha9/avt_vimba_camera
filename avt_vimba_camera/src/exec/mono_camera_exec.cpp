#include "avt_vimba_camera/mono_camera_node.hpp"

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::string yaml_fp = "/home/autera-admin/ART/race_common/src/external/drivers/vimba_driver/avt_vimba_camera/config/test_params.yaml";
  rclcpp::NodeOptions options{};
  options.use_global_arguments(false);
  options.enable_rosout(false);
  options.enable_topic_statistics(false);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.enable_logger_service(false);
  options.append_parameter_override("start_type_description_service", false);
  std::shared_ptr<avt_vimba_camera::MonoCameraNode> node = std::make_shared<avt_vimba_camera::MonoCameraNode>(options, yaml_fp);
  node->start();
  rclcpp::spin(node);
  return 0;
}
