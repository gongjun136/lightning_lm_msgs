// shm_image6m_subscriber_6ch.cpp
#include <cstring>
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>
#include <errno.h>

#include "rclcpp/rclcpp.hpp"

#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

#include <opencv2/opencv.hpp>

using namespace shm_msgs;

class MultiCamListener : public rclcpp::Node
{
private:
  using Topic = shm_msgs::msg::Image6m;

  // 落盘任务：callback 组装，saver 线程消费
  struct SaveTask {
    cv::Mat image;          // toCvCopy 已深拷贝出 SHM，Mat 引用计数管理
    std::string raw_path;
    std::string jpg_path;
    std::string cam_name;
    uint64_t frame_count;
  };

  struct CamSubscriber {
    rclcpp::Subscription<Topic>::SharedPtr sub;
    rclcpp::Time last_image_ts{0, 0, RCL_ROS_TIME};
    std::string cam_name;
    uint64_t frame_count{0};
    std::string save_dir;

    // 落盘队列与消费线程
    std::queue<SaveTask> save_queue;
    std::mutex queue_mtx;
    std::condition_variable queue_cv;
    std::thread saver_thread;
    std::atomic<bool> running{true};
    std::atomic<uint64_t> dropped_count{0};
  };

  static constexpr size_t kMaxQueue = 2;   // 每路待落盘任务上限，超过则丢最旧

  std::vector<std::unique_ptr<CamSubscriber>> m_cams;
  std::string m_base_dir;

  // 将 timespec 转换为可读的时间字符串，用于文件名
  std::string timestampToStr(const builtin_interfaces::msg::Time &stamp) {
    std::time_t sec = stamp.sec;
    std::tm *tm_info = std::gmtime(&sec);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", tm_info);
    return std::string(buf) + "_" + std::to_string(stamp.nanosec / 1000000);
  }

  // 保存 YUYV 原始数据为 .raw 文件
  void saveRawData(const std::string &filepath, const cv::Mat &image) {
    std::ofstream file(filepath, std::ios::binary);
    if (file.is_open()) {
      file.write(reinterpret_cast<const char*>(image.data), image.total() * image.elemSize());
      file.close();
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open file for writing: %s", filepath.c_str());
    }
  }

  // 保存为 JPEG 图像（YUYV -> BGR 后编码）
  void saveAsJpeg(const std::string &filepath, const cv::Mat &image) {
    cv::Mat bgr;
    if (image.channels() == 2) {
      // YUYV 格式转 BGR
      cv::cvtColor(image, bgr, cv::COLOR_YUV2BGR_YUYV);
    } else if (image.channels() == 3) {
      bgr = image;
    } else {
      bgr = image;
    }
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 90};
    cv::imwrite(filepath, bgr, params);
  }

  // 落盘消费线程：从队列取任务写盘
  void saverLoop(CamSubscriber *cam) {
    while (cam->running.load() || !cam->save_queue.empty()) {
      SaveTask task;
      {
        std::unique_lock<std::mutex> lk(cam->queue_mtx);
        cam->queue_cv.wait(lk, [&]{
          return !cam->save_queue.empty() || !cam->running.load();
        });
        if (cam->save_queue.empty() && !cam->running.load()) {
          break;
        }
        task = std::move(cam->save_queue.front());
        cam->save_queue.pop();
      }
      saveRawData(task.raw_path, task.image);
      saveAsJpeg(task.jpg_path, task.image);
      RCLCPP_INFO(get_logger(), "[%s] Saved frame %lu: %s",
                  task.cam_name.c_str(), task.frame_count, task.raw_path.c_str());
    }
  }

public:
  explicit MultiCamListener(const rclcpp::NodeOptions &options)
      : Node("shm_image6m_subscriber_6ch", options)
  {
    // 获取保存路径参数，默认为当前目录下的 captured_images
    m_base_dir = declare_parameter<std::string>("save_dir", "./captured_images");

    // 创建基础目录
    struct stat st;
    if (stat(m_base_dir.c_str(), &st) != 0) {
      if (mkdir(m_base_dir.c_str(), 0755) != 0) {
        RCLCPP_ERROR(get_logger(), "Failed to create base directory: %s", m_base_dir.c_str());
      }
    }

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
    m_cams.reserve(num_cams);

    // 订阅摄像头
    for (int i = 0; i < num_cams; i++) {
      auto cam_ptr = std::make_unique<CamSubscriber>();
      cam_ptr->cam_name = cam_position_names[i];

      // 为每路相机创建子目录
      cam_ptr->save_dir = m_base_dir + "/" + cam_ptr->cam_name;
      if (stat(cam_ptr->save_dir.c_str(), &st) != 0) {
        if (mkdir(cam_ptr->save_dir.c_str(), 0755) != 0) {
          RCLCPP_ERROR(get_logger(), "Failed to create directory: %s", cam_ptr->save_dir.c_str());
        }
      }

      // 启动落盘消费线程
      cam_ptr->saver_thread = std::thread(&MultiCamListener::saverLoop, this, cam_ptr.get());

      std::string topic = "/camera/" + cam_ptr->cam_name + "/shm_image_6m";
      int idx = i;

      auto callback = [this, idx](const Topic::SharedPtr msg) -> void {
        auto &cam = *m_cams[idx];

        // 先把 header 拷出来，后面要 reset(msg) 归还 SHM chunk
        auto stamp = msg->header.stamp;

        RCLCPP_INFO(this->get_logger(), "[%s] Received frame %lu...",
                    cam.cam_name.c_str(), cam.frame_count);

        // 用 toCvCopy 深拷贝一份，不占用共享内存
        // msg 为 const SharedPtr，无法 reset；回调内无重 I/O，
        // 到回调结束 msg 自然析构即归还 SHM chunk（耗时亚毫秒级，可接受）
        auto cv_copy = shm_msgs::toCvCopy(msg);

        auto time_offset_ns = (now() - cv_copy->header.stamp).nanoseconds();
        auto timestamp_offset_ns = (rclcpp::Time(stamp) - cam.last_image_ts).nanoseconds();
        auto time_offset_ms = time_offset_ns / 1000000.0F;
        auto timestamp_offset_ms = timestamp_offset_ns / 1000000.0F;

        RCLCPP_INFO(get_logger(), "[%s] get-image6m-transport-time: %.3f ms",
                    cam.cam_name.c_str(), time_offset_ms);

        if (cam.last_image_ts.nanoseconds() > 0.0) {
          RCLCPP_INFO(get_logger(), "[%s] get-image6m-timestamp_offset-time: %.3f ms",
                      cam.cam_name.c_str(), timestamp_offset_ms);
        }

        cam.last_image_ts = stamp;

        // 组装落盘任务（路径与文件名在 callback 里算好，落盘线程只做 I/O）
        SaveTask task;
        task.image = cv_copy->image;   // Mat 浅拷贝，引用计数保活数据
        task.cam_name = cam.cam_name;
        task.frame_count = cam.frame_count;
        std::string timestamp_str = timestampToStr(stamp);
        task.raw_path = cam.save_dir + "/" + timestamp_str + "_" +
                        std::to_string(cam.frame_count) + ".raw";
        task.jpg_path = cam.save_dir + "/" + timestamp_str + "_" +
                        std::to_string(cam.frame_count) + ".jpg";

        // 入队（有界）；满则丢弃最旧任务，避免反压与延迟累积
        {
          std::lock_guard<std::mutex> lk(cam.queue_mtx);
          if (cam.save_queue.size() >= kMaxQueue) {
            cam.save_queue.pop();
            uint64_t dropped = cam.dropped_count.fetch_add(1) + 1;
            RCLCPP_WARN(get_logger(),
                        "[%s] Save queue full, dropped oldest (total dropped: %lu)",
                        cam.cam_name.c_str(), dropped);
          }
          cam.save_queue.push(std::move(task));
        }
        cam.queue_cv.notify_one();

        cam.frame_count++;
      };

      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.best_effort();           // 关键：解除 RELIABLE 反压
      qos.durability_volatile();
      cam_ptr->sub = create_subscription<Topic>(topic, qos, callback);

      CamSubscriber *raw = cam_ptr.get();
      m_cams.push_back(std::move(cam_ptr));

      RCLCPP_INFO(get_logger(), "Subscribed to %s -> saving to %s",
                  topic.c_str(), raw->save_dir.c_str());
    }
  }

  ~MultiCamListener() {
    // 通知所有落盘线程停止并等待其排空退出
    for (auto &cam : m_cams) {
      cam->running.store(false);
      cam->queue_cv.notify_all();
      if (cam->saver_thread.joinable()) {
        cam->saver_thread.join();
      }
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<MultiCamListener>(options));
  rclcpp::shutdown();
  return 0;
}
