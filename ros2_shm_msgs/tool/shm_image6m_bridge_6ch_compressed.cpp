// shm_image6m_bridge_6ch_compressed.cpp
// 将 shm_msgs::msg::Image6m 降采样为低帧率 sensor_msgs::msg::CompressedImage。
// 采用共享绝对时间栅格抽帧：时间轴按 interval 切槽，每路每槽仅保留首个进入的帧。
// 输出时间戳对齐到槽位边界 (slot * interval)，6 路同一槽位的输出时间戳完全相同，
// 便于下游按时间戳配对融合。低速场景下作为基本时间对齐手段。
//
// 直接使用 OpenCV (cvtColor + imencode) 完成 YUV422->BGR->JPEG，
// 不经过 cv_bridge::CvImage::toCompressedImageMsg，以避免与 ROS 自带
// cv_bridge 链接的不同版本 OpenCV 产生符号冲突 (setSize 断言)。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using Topic = shm_msgs::msg::Image6m;

// 每路摄像头的状态：记录上次保留帧所在的栅格槽位号。
// 6 路共用同一把绝对刻度尺（槽位 = stamp / interval），保证对应帧时间戳一致。
struct PerCamState {
  std::int64_t last_slot{-1};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("shm_image_6ch_bridge_compressed");

  const int target_fps = node->declare_parameter<int>("target_fps", 10);
  if (target_fps <= 0 || target_fps > 1000) {
    RCLCPP_FATAL(node->get_logger(), "target_fps must be in (0, 1000]");
    rclcpp::shutdown();
    return 1;
  }
  const std::int64_t interval_ns = 1000000000LL / target_fps;

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
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> pubs;

  for (int i = 0; i < num_cams; i++) {
    std::string cam_name = cam_position_names[i];
    std::string sub_topic = "/camera/" + cam_name + "/shm_image_6m";
    std::string pub_topic = "/camera/" + cam_name + "/compressed";

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    auto pub = node->create_publisher<sensor_msgs::msg::CompressedImage>(pub_topic, qos);

    auto state = std::make_shared<PerCamState>();

    auto callback = [node, pub, state, interval_ns, cam_name](const typename Topic::SharedPtr msg_in) -> void {
      auto last_cvimage = shm_msgs::toCvShare(msg_in);
      const cv::Mat &src = last_cvimage->image;
      const std::string &enc = last_cvimage->encoding;

      // 用采集时间戳计算绝对栅格槽位，6 路共用同一刻度尺。
      // round-to-nearest（而非 floor）：边界敏感区从槽位边缘挪到中心 ±interval/2，
      // 两路对应帧源时间戳差 < interval/2 时数学上保证落入同一槽位。
      const std::int64_t stamp_ns =
        static_cast<std::int64_t>(last_cvimage->header.stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(last_cvimage->header.stamp.nanosec);
      const std::int64_t slot = (stamp_ns + interval_ns / 2) / interval_ns;
      if (slot == state->last_slot) {
        return;  // 本槽位已保留过帧，丢弃
      }
      state->last_slot = slot;

      cv::Mat bgr;
      if (enc == "yuv422_yuy2") {
        cv::cvtColor(src, bgr, cv::COLOR_YUV2BGR_YUY2);
      } else if (enc == "yuv422") {
        cv::cvtColor(src, bgr, cv::COLOR_YUV2BGR_UYVY);
      } else if (enc == "rgb8") {
        cv::cvtColor(src, bgr, cv::COLOR_RGB2BGR);
      } else if (enc == "rgba8") {
        cv::cvtColor(src, bgr, cv::COLOR_RGBA2BGR);
      } else if (enc == "bgra8") {
        cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
      } else if (enc == "mono8" || enc == "bgr8") {
        bgr = src;
      } else {
        bgr = src;
      }

      sensor_msgs::msg::CompressedImage msg_out;
      // 输出时间戳对齐到槽位边界，6 路同一槽位时间戳完全相同，便于配对
      const std::int64_t aligned_ns = slot * interval_ns;
      msg_out.header.stamp.sec = static_cast<std::int32_t>(aligned_ns / 1000000000LL);
      msg_out.header.stamp.nanosec = static_cast<std::uint32_t>(aligned_ns % 1000000000LL);
      msg_out.header.frame_id = last_cvimage->header.frame_id;
      msg_out.format = "jpeg";
      if (!cv::imencode(".jpg", bgr, msg_out.data)) {
        RCLCPP_WARN(node->get_logger(), "JPEG encode failed for encoding '%s'",
                    enc.c_str());
        return;
      }
      pub->publish(std::move(msg_out));

      // 打印源时间戳与对齐后输出时间戳，便于测试时核对 6 路是否对齐到同一时刻
      RCLCPP_INFO(
        node->get_logger(),
        "[%s] slot=%ld src=%ld.%09u out=%ld.%09u (delta=%ldns)",
        cam_name.c_str(), slot,
        static_cast<long>(last_cvimage->header.stamp.sec),
        last_cvimage->header.stamp.nanosec,
        static_cast<long>(aligned_ns / 1000000000LL),
        static_cast<unsigned>(aligned_ns % 1000000000LL),
        stamp_ns - aligned_ns);
    };

    auto sub = node->create_subscription<Topic>(sub_topic, 10, callback);
    subs.push_back(sub);
    pubs.push_back(pub);

    RCLCPP_INFO(node->get_logger(), "Bridge: %s -> %s (jpeg, %dfps, aligned stamp)",
                sub_topic.c_str(), pub_topic.c_str(), target_fps);
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
