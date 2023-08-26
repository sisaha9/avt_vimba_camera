#include "avt_vimba_camera/mono_camera_node.hpp"
#include <rti/util/util.hpp>

#include <memory>

void start_infinite_loop()
{
  for (size_t i = 0; 1; i++) {
    rti::util::sleep(dds::core::Duration(0, 10000000));
  }
}

int main(int argc, char** argv)
{
  if (argc >= 3) {
    size_t domain_id = atoi(argv[1]);
    std::string yaml_fp = argv[2];
    std::shared_ptr<avt_vimba_camera::MonoCameraNode> node = std::make_shared<avt_vimba_camera::MonoCameraNode>(domain_id, yaml_fp);
    node->start();
    start_infinite_loop();
    return 0;
  }
  return -1;
}
