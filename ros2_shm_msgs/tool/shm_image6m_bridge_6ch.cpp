// six_channel_bridge.cpp
#include "shm_image_bridge.hpp"

using Topic = shm_msgs::msg::Image6m;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  
  // 创建 6 路 bridge 节点，每路一组 pub/sub
  auto node = std::make_shared<rclcpp::Node>("shm_image_6ch_bridge");
  
  // 摄像头位置名称，与 cam_geac/cfg/camera_position.json 保持一致
  static const char* cam_position_names[] = {
      "fisheye_back",
      "fisheye_right",
      "fisheye_left",
      "fisheye_front",
      "pinhole_back",
      "pinhole_front",
  };
  const int num_cams = sizeof(cam_position_names) / sizeof(cam_position_names[0]);

  std::vector<rclcpp::Subscription<Topic>::SharedPtr> subs;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pubs;
  
  for (int i = 0; i < num_cams; i++) {
    std::string cam_name = cam_position_names[i];
    std::string sub_topic = "/camera/" + cam_name + "/shm_image_6m";
    std::string pub_topic = "/camera/" + cam_name + "/sensor_image_6m";
    
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    auto pub = node->create_publisher<sensor_msgs::msg::Image>(pub_topic, qos);
    // auto pub = node->create_publisher<sensor_msgs::msg::Image>(pub_topic, 10);
    
    auto callback = [node, pub](const typename Topic::SharedPtr msg_in) -> void {
      auto last_cvimage = shm_msgs::toCvShare(msg_in);
      auto msg_out = cv_bridge::CvImage(
        last_cvimage->header, 
        last_cvimage->encoding, 
        last_cvimage->image
      ).toImageMsg();
      pub->publish(std::move(*msg_out));
    };
    
    auto sub = node->create_subscription<Topic>(sub_topic, 10, callback);
    subs.push_back(sub);
    pubs.push_back(pub);
    
    RCLCPP_INFO(node->get_logger(), "Bridge: %s -> %s", sub_topic.c_str(), pub_topic.c_str());
  }
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}