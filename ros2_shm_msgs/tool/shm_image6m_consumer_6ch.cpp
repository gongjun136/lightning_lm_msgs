// shm_image6m_consumer_6ch.cpp
//
// 六路摄像头 SHM 订阅参考示例 —— 供遥操推流模块调用
//
// 设计要点：
//   1) 通过 shm_msgs::toCvShare() 获取帧数据：
//      - ICE/ZEROCOPY 传输开启时 → 零拷贝共享共享内存中的 cv::Mat（不可修改）
//      - 普通 DDS 传输时         → 内部自动拷贝，行为同 toCvCopy()
//      即"开启 SHM 零拷贝、未开启也能正常订阅到数据"。
//   2) 若下游需要可写副本，改用 shm_msgs::toCvCopy() 深拷贝。
//   3) 回调内避免重 I/O；如需写盘/编码，应投递到独立线程。
//
// 编译依赖（见 CMakeLists.txt）：
//   rclcpp  shm_msgs_image  opencv_core  opencv_imgproc  opencv_imgcodecs
//
// Topic 命名规则（与 cam_geac cfg/camera_position.json 一致）：
//   /camera/{cam_position_name}/shm_image_6m
//
// 摄像头位置列表：
//   fisheye_back, fisheye_right, fisheye_left,
//   fisheye_front, pinhole_back, pinhole_front
//
// 使用方式（二选一）：
//   A) 直接运行此可执行文件，每帧到达时打印日志。
//   B) 将 ShmImage6mConsumer 类嵌入自己的节点，注入 FrameCallback。

#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using Topic = shm_msgs::msg::Image6m;

//==========================================================================
// 帧消费回调类型 —— 遥操推流模块注册此回调，每帧到达时被调用。
//   cv_ptr      toCvShare() 返回的 CvImageConstPtr（含 header / encoding / image）
//               开启 SHM 零拷贝时 cv::Mat 直接引用共享内存；
//               未开启时内部已拷贝，数据安全。
//   cam_name    摄像头位置名，用于区分通道
//   frame_count 该通道累计帧号（可用于丢帧检测）
using FrameCallback = std::function<void(
    const shm_msgs::CvImageConstPtr &cv_ptr,
    const std::string &cam_name,
    uint64_t frame_count)>;

//==========================================================================
// 六路摄像头 SHM 消费者
class ShmImage6mConsumer : public rclcpp::Node
{
public:
  explicit ShmImage6mConsumer(
      const rclcpp::NodeOptions &options,
      FrameCallback user_cb = nullptr)
    : Node("shm_image6m_consumer_6ch", options)
    , m_user_cb(std::move(user_cb))
  {
    // 摄像头位置列表，与 cam_geac/cfg/camera_position.json 保持一致
    static const char* kCamNames[] = {
        "fisheye_back",
        "fisheye_right",
        "fisheye_left",
        "fisheye_front",
        "pinhole_back",
        "pinhole_front",
    };
    constexpr int kNumCams = sizeof(kCamNames) / sizeof(kCamNames[0]);

    m_cam_infos.resize(kNumCams);

    for (int i = 0; i < kNumCams; i++) {
      auto &info = m_cam_infos[i];
      info.cam_name   = kCamNames[i];
      info.topic_name = "/camera/" + info.cam_name + "/shm_image_6m";
      info.frame_count = 0;

      // Best-effort + KeepLast(1)：不反压发布端，只取最新帧
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.best_effort();
      qos.durability_volatile();

      auto callback = [this, i](const Topic::SharedPtr msg) {
        onFrame(i, msg);
      };

      info.sub = create_subscription<Topic>(info.topic_name, qos, callback);

      RCLCPP_INFO(get_logger(), "Subscribed to %s", info.topic_name.c_str());
    }
  }

private:
  struct CamInfo {
    std::string cam_name;
    std::string topic_name;
    rclcpp::Subscription<Topic>::SharedPtr sub;
    uint64_t frame_count{0};
  };

  std::vector<CamInfo> m_cam_infos;
  FrameCallback m_user_cb;

  void onFrame(int idx, const Topic::SharedPtr msg)
  {
    auto &info = m_cam_infos[idx];

    //======================================================================
    // 【关键】shm_msgs::toCvShare(msg) 语义：
    //   - ICE/zero-copy 传输激活时 → 零拷贝，cv::Mat 直接指向共享内存
    //     数据不可修改（const）。
    //   - 普通 DDS 传输时 → 内部深拷贝，行为透明，数据安全。
    //   无论是哪种情况，toCvShare 都会返回一个包含完整图像数据的 CvImage，
    //   调用方无需区分传输方式。
    //
    //   如需可写副本，改用 shm_msgs::toCvCopy(msg)。
    //======================================================================
    auto cv_ptr = shm_msgs::toCvShare(msg);

    RCLCPP_DEBUG(get_logger(), "[%s] frame=%lu, enc=%s, %dx%d",
                 info.cam_name.c_str(), info.frame_count,
                 cv_ptr->encoding.c_str(),
                 cv_ptr->image.cols, cv_ptr->image.rows);

    if (m_user_cb) {
      m_user_cb(cv_ptr, info.cam_name, info.frame_count);
    }

    info.frame_count++;
  }
};

//==========================================================================
// main —— 可执行入口，直接启动并打印帧信息
//
// 遥操推流模块按照下方 main() 中 my_callback 的模式使用即可：
//   1. 在 my_callback 内获取 cv_ptr->image（cv::Mat）
//   2. 根据需要 YUV422 → BGR 转换
//   3. 送入 H265 编码器 → RTP 推流
//
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // ---- 用户自定义帧处理（推流/编码/显示） ----
  FrameCallback my_callback =
      [](const shm_msgs::CvImageConstPtr &cv_ptr,
         const std::string &cam_name,
         uint64_t frame_count)
  {
    (void)frame_count;

    // 1) 获取图像数据（零拷贝 / 自动 fallback）
    const cv::Mat &frame = cv_ptr->image;

    // 2) YUV422(YUYV) → BGR（推流编码器通常需要 BGR 或 RGB 输入）
    cv::Mat bgr;
    if (cv_ptr->encoding == "yuv422_yuy2") {
      cv::cvtColor(frame, bgr, cv::COLOR_YUV2BGR_YUY2);
    } else if (cv_ptr->encoding == "yuv422") {
      cv::cvtColor(frame, bgr, cv::COLOR_YUV2BGR_UYVY);
    } else {
      bgr = frame;  // 已是 BGR/MONO
    }

    // 3) 将 bgr 送入推流编码器
    //    encodeAndPush(cam_name, bgr);

    // 如需可写副本，使用 toCvCopy：
    //    auto copy = shm_msgs::toCvCopy(msg);

    RCLCPP_INFO(rclcpp::get_logger("shm_image6m_consumer"),
                "[%s] got frame %lu, enc=%s, %dx%d",
                cam_name.c_str(), frame_count,
                cv_ptr->encoding.c_str(),
                bgr.cols, bgr.rows);
  };

  rclcpp::NodeOptions options;
  auto node = std::make_shared<ShmImage6mConsumer>(options, my_callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
