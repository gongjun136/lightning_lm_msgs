#include "rclcpp/rclcpp.hpp"
#include "shm_msgs/array_helper.hpp"
#include "shm_msgs/msg/compressed_video.hpp"
#include "shm_msgs/msg/image6m.hpp"
#include "video_encode_api.h"

#include <dlfcn.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace shm_msgs
{
namespace compression
{

using CompressedVideo = shm_msgs::msg::CompressedVideo;
using Image6m = shm_msgs::msg::Image6m;

class VideoEncoderApi
{
public:
  VideoEncoderApi(const std::string & library_path, const std::string & yuv_library_path)
  {
    library_ = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (library_ == nullptr) {
      throw std::runtime_error(
              "Cannot load " + library_path + ": " + std::string(dlerror()));
    }
    init_ = load_symbol<InitFunction>("VIDEOENC_Init");
    release_ = load_symbol<ReleaseFunction>("VIDEOENC_Release");
    create_handle_ = load_symbol<CreateHandleFunction>("VIDEOENC_CreateHandle");
    destroy_handle_ = load_symbol<DestroyHandleFunction>("VIDEOENC_DestroyHandle");
    set_callback_ = load_symbol<SetCallbackFunction>("VIDEOENC_SetDataCallBack");
    input_data_ = load_symbol<InputDataFunction>("VIDEOENC_InputData2");
    yuv_library_ = dlopen(yuv_library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (yuv_library_ == nullptr) {
      throw std::runtime_error(
              "Cannot load " + yuv_library_path + ": " + std::string(dlerror()));
    }
    yuy2_to_i420_ = load_symbol_from<Yuy2ToI420Function>(
      yuv_library_, "YUY2ToI420");
    if (init_() != 0) {
      throw std::runtime_error("VIDEOENC_Init failed");
    }
    initialized_ = true;
  }

  VideoEncoderApi(const VideoEncoderApi &) = delete;
  VideoEncoderApi & operator=(const VideoEncoderApi &) = delete;

  ~VideoEncoderApi()
  {
    if (initialized_) {
      release_();
    }
    if (yuv_library_ != nullptr) {
      dlclose(yuv_library_);
    }
    if (library_ != nullptr) {
      dlclose(library_);
    }
  }

  void * create_handle(VIDEOENC_PARA * parameters) const
  {
    return create_handle_(parameters);
  }

  int destroy_handle(void * handle) const
  {
    return destroy_handle_(handle);
  }

  int set_callback(void * handle, VIDEOENC_CALLBACK callback, void * user_data) const
  {
    return set_callback_(handle, callback, user_data);
  }

  int input_data(
    void * handle, VIDEOENC_FRAME_INFO * frame_info,
    unsigned char * data, int data_length, int timeout_ms) const
  {
    return input_data_(handle, frame_info, data, data_length, timeout_ms);
  }

  int yuy2_to_i420(
    const unsigned char * src_yuy2, int src_stride_yuy2,
    unsigned char * dst_y, int dst_stride_y,
    unsigned char * dst_u, int dst_stride_u,
    unsigned char * dst_v, int dst_stride_v,
    int width, int height) const
  {
    return yuy2_to_i420_(
      src_yuy2, src_stride_yuy2, dst_y, dst_stride_y,
      dst_u, dst_stride_u, dst_v, dst_stride_v, width, height);
  }

private:
  using InitFunction = int (*)();
  using ReleaseFunction = int (*)();
  using CreateHandleFunction = void * (*)(VIDEOENC_PARA *);
  using DestroyHandleFunction = int (*)(void *);
  using SetCallbackFunction = int (*)(void *, VIDEOENC_CALLBACK, void *);
  using InputDataFunction = int (*)(
    void *, VIDEOENC_FRAME_INFO *, unsigned char *, int, int);
  using Yuy2ToI420Function = int (*)(
    const unsigned char *, int, unsigned char *, int,
    unsigned char *, int, unsigned char *, int, int, int);

  template<typename FunctionT>
  FunctionT load_symbol(const char * name)
  {
    return load_symbol_from<FunctionT>(library_, name);
  }

  template<typename FunctionT>
  FunctionT load_symbol_from(void * library, const char * name)
  {
    dlerror();
    void * symbol = dlsym(library, name);
    const char * error = dlerror();
    if (error != nullptr) {
      throw std::runtime_error(
              "Cannot resolve " + std::string(name) + ": " + std::string(error));
    }
    return reinterpret_cast<FunctionT>(symbol);
  }

  void * library_{nullptr};
  void * yuv_library_{nullptr};
  bool initialized_{false};
  InitFunction init_{nullptr};
  ReleaseFunction release_{nullptr};
  CreateHandleFunction create_handle_{nullptr};
  DestroyHandleFunction destroy_handle_{nullptr};
  SetCallbackFunction set_callback_{nullptr};
  InputDataFunction input_data_{nullptr};
  Yuy2ToI420Function yuy2_to_i420_{nullptr};
};

class H265Stream
{
public:
  H265Stream(
    rclcpp::Logger logger, std::string input_topic,
    rclcpp::Publisher<CompressedVideo>::SharedPtr publisher,
    VideoEncoderApi & api, unsigned int bitrate, unsigned char fps,
    unsigned int gop, std::size_t max_pending_frames)
  : logger_(logger), input_topic_(std::move(input_topic)), publisher_(std::move(publisher)),
    api_(api), bitrate_(bitrate), fps_(fps), gop_(gop),
    max_pending_frames_(max_pending_frames)
  {
    // fps is the desired output frame rate. Throttle uses a shared absolute
    // time grid (slot = round(stamp/interval)); each camera keeps only the
    // first frame entering each slot. All six cameras share the same grid
    // (anchored at the ROS epoch), so corresponding frames land in the same
    // slot and produce identical aligned output timestamps. round-to-nearest
    // puts the boundary at slot center +/-interval/2, so source jitter smaller
    // than interval/2 never crosses a boundary.
    interval_ns_ = static_cast<std::int64_t>(1000000000LL / fps_);
    RCLCPP_INFO(
      logger_, "%s frame-rate grid: target=%uHz (interval=%ldns)",
      input_topic_.c_str(), fps_, interval_ns_);
    worker_ = std::thread([this]() {worker_loop();});
  }

  H265Stream(const H265Stream &) = delete;
  H265Stream & operator=(const H265Stream &) = delete;

  ~H265Stream()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }
    queue_cv_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (handle_ != nullptr) {
      api_.destroy_handle(handle_);
    }
  }

  void encode(const Image6m::SharedPtr & input)
  {
    ++received_frames_;
    const std::string source_encoding = shm_msgs::get_str(input->encoding);
    if (source_encoding != "yuv422_yuy2") {
      ++rejected_frames_;
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 5000, "%s uses unsupported encoding '%s'",
        input_topic_.c_str(), source_encoding.c_str());
      return;
    }

    // Absolute time-grid throttle (shared across all cameras via the ROS
    // epoch): keep only the first frame whose stamp rounds into a new slot.
    // All six cameras share the same grid (anchored at the ROS epoch), so
    // corresponding frames land in the same slot and produce identical
    // aligned output timestamps for BEV pairing. round-to-nearest puts the
    // boundary at slot center +/-interval/2, so source jitter smaller than
    // interval/2 never crosses a boundary. The check runs under queue_mutex_
    // because the subscription callbacks execute concurrently (reentrant
    // callback group + multi-threaded executor): a racy check-then-act on
    // last_slot_ would let several frames through in the same slot and push
    // the output rate toward the input rate.
    const std::int64_t stamp_ns =
      static_cast<std::int64_t>(input->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(input->header.stamp.nanosec);
    const std::int64_t slot = (stamp_ns + interval_ns_ / 2) / interval_ns_;

    const std::uint64_t data_length =
      static_cast<std::uint64_t>(input->step) * input->height;
    if (data_length == 0 || data_length > input->data.size() ||
      data_length > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
    {
      ++rejected_frames_;
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 5000, "%s has invalid image payload size",
        input_topic_.c_str());
      return;
    }
    if ((input->width & 1U) != 0U || (input->height & 1U) != 0U ||
      input->step < input->width * 2U)
    {
      ++rejected_frames_;
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 5000, "%s has invalid YUY2 dimensions or stride",
        input_topic_.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (slot == last_slot_) {
        ++throttled_frames_;
        return;
      }
      last_slot_ = slot;
      if (has_queued_frame_) {
        ++queue_drops_;
      }
      // Hand the received message straight to the worker instead of copying
      // the ~6 MB payload in the subscription callback. The callback then
      // costs almost nothing, so the capture process is no longer slowed by
      // this node's memory traffic; the worker converts directly from the
      // message that the transport already delivered into this process.
      queued_frame_.image = input;
      queued_frame_.source_encoding = source_encoding;
      queued_frame_.header = shm_msgs::get_header(input->header);
      // Overwrite the stamp with the aligned slot boundary so all six cameras
      // at the same slot carry an identical timestamp downstream.
      const std::int64_t aligned_ns = slot * interval_ns_;
      queued_frame_.header.stamp.sec = static_cast<std::int32_t>(aligned_ns / 1000000000LL);
      queued_frame_.header.stamp.nanosec = static_cast<std::uint32_t>(aligned_ns % 1000000000LL);
      has_queued_frame_ = true;
    }
    queue_cv_.notify_one();
  }

  void log_stats() const
  {
    std::size_t encoder_pending_count;
    bool raw_pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      encoder_pending_count = pending_frames_.size();
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      raw_pending = has_queued_frame_;
    }
    RCLCPP_INFO(
      logger_, "%s: received=%lu chunks=%lu frames=%lu queue_drops=%lu rejected=%lu "
      "throttled=%lu "
      "input_failures=%lu empty_callbacks=%lu additional_callbacks=%lu "
      "unmatched_callbacks=%lu "
      "raw_pending=%d encoder_pending=%zu",
      input_topic_.c_str(), received_frames_.load(), chunks_.load(),
      encoded_frames_.load(), queue_drops_.load(), rejected_frames_.load(),
      throttled_frames_.load(),
      input_failures_.load(), empty_callbacks_.load(), additional_callbacks_.load(),
      unmatched_callbacks_.load(), raw_pending ? 1 : 0, encoder_pending_count);
  }

private:
  struct RawFrame
  {
    Image6m::SharedPtr image;
    std_msgs::msg::Header header;
    std::string source_encoding;
  };

  struct PendingFrame
  {
    std_msgs::msg::Header header;
    std::string source_encoding;
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint64_t sequence{0};
  };

  void worker_loop()
  {
    RawFrame frame;
    while (true) {
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {return stopping_ || has_queued_frame_;});
        if (stopping_ && !has_queued_frame_) {
          return;
        }
        std::swap(frame, queued_frame_);
        has_queued_frame_ = false;
      }
      encode_frame(frame);
    }
  }

  void encode_frame(const RawFrame & input)
  {
    if (input.image == nullptr) {
      ++rejected_frames_;
      return;
    }
    const std::uint32_t width = input.image->width;
    const std::uint32_t height = input.image->height;
    if (!ensure_handle(width, height)) {
      ++rejected_frames_;
      return;
    }

    const std::uint64_t i420_length =
      static_cast<std::uint64_t>(width) * height * 3U / 2U;
    if (i420_length > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
      ++rejected_frames_;
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 5000, "%s I420 frame is too large", input_topic_.c_str());
      return;
    }
    i420_buffer_.resize(static_cast<std::size_t>(i420_length));
    const std::size_t y_size = static_cast<std::size_t>(width) * height;
    const std::size_t chroma_size = y_size / 4U;
    if (api_.yuy2_to_i420(
        input.image->data.data(), static_cast<int>(input.image->step),
        i420_buffer_.data(), static_cast<int>(width),
        i420_buffer_.data() + y_size, static_cast<int>(width / 2U),
        i420_buffer_.data() + y_size + chroma_size,
        static_cast<int>(width / 2U),
        static_cast<int>(width), static_cast<int>(height)) != 0)
    {
      ++rejected_frames_;
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 5000, "%s YUY2-to-I420 conversion failed", input_topic_.c_str());
      return;
    }

    PendingFrame pending;
    pending.header = input.header;
    pending.source_encoding = input.source_encoding;
    pending.width = width;
    pending.height = height;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_frames_.size() >= max_pending_frames_) {
        ++rejected_frames_;
        RCLCPP_ERROR_THROTTLE(
          logger_, clock_, 5000, "%s encoder callback backlog reached %zu frames",
          input_topic_.c_str(), pending_frames_.size());
        return;
      }
      pending.sequence = next_sequence_++;
      pending_frames_.push_back(pending);
    }

    VIDEOENC_FRAME_INFO frame_info{};
    frame_info.sec = static_cast<unsigned int>(input.header.stamp.sec);
    frame_info.nan = input.header.stamp.nanosec;
    const int result = api_.input_data(
      handle_, &frame_info, i420_buffer_.data(), static_cast<int>(i420_length), 10);
    if (result != 0) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto item = std::find_if(
        pending_frames_.begin(), pending_frames_.end(),
        [&pending](const PendingFrame & candidate) {
          return candidate.sequence == pending.sequence;
        });
      if (item != pending_frames_.end()) {
        pending_frames_.erase(item);
      }
      ++input_failures_;
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 5000, "%s VIDEOENC_InputData2 returned %d",
        input_topic_.c_str(), result);
    }
  }

  bool ensure_handle(std::uint32_t width, std::uint32_t height)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ != nullptr) {
      if (width != width_ || height != height_) {
        RCLCPP_ERROR_THROTTLE(
          logger_, clock_, 5000, "%s resolution changed; restart the node",
          input_topic_.c_str());
        return false;
      }
      return true;
    }

    VIDEOENC_PARA parameters{};
    parameters.encfmt = 1;
    parameters.fps = fps_;
    parameters.pixfmt = 0;
    parameters.dma = 1;
    parameters.insert_sps_pps_at_idr = 1;
    parameters.alliframe = 0;
    parameters.bitrate = bitrate_;
    parameters.ifi = gop_;
    parameters.width = width;
    parameters.height = height;
    parameters.bufs = 1;
    handle_ = api_.create_handle(&parameters);
    if (handle_ == nullptr) {
      RCLCPP_ERROR(logger_, "%s failed to create H.265 encoder", input_topic_.c_str());
      return false;
    }
    if (api_.set_callback(handle_, &H265Stream::encoded_callback, this) != 0) {
      api_.destroy_handle(handle_);
      handle_ = nullptr;
      RCLCPP_ERROR(logger_, "%s failed to register encoder callback", input_topic_.c_str());
      return false;
    }
    width_ = width;
    height_ = height;
    RCLCPP_INFO(
      logger_, "%s encoder ready: %ux%u@%u, bitrate=%u, GOP=%u",
      input_topic_.c_str(), width_, height_, fps_, bitrate_, gop_);
    return true;
  }

  static void encoded_callback(
    unsigned char type, unsigned char * data, int data_length, void * user_data)
  {
    static_cast<H265Stream *>(user_data)->publish(type, data, data_length);
  }

  void publish(unsigned char type, unsigned char * data, int data_length)
  {
    if (data == nullptr || data_length <= 0) {
      ++empty_callbacks_;
      return;
    }
    ++chunks_;
    // The hardware encoder emits one callback per NAL unit, so a single input
    // frame produces several callbacks (see CompressedVideo.msg: chunks with
    // the same frame_sequence belong to one frame). Publishing every callback
    // made the output topic run at the NAL rate (~2.2x the frame rate),
    // i.e. ~22 Hz instead of 10 Hz. Accumulate all NALs of one frame and
    // publish a single CompressedVideo per frame so the topic rate equals
    // the frame rate. A frame is complete once the first chunk of the next
    // frame arrives (the encoder processes frames in order).
    CompressedVideo to_publish;
    bool do_publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_frames_.empty()) {
        if (!current_output_.data.empty()) {
          to_publish = std::move(current_output_);
          do_publish = true;
        }
        const PendingFrame pending = std::move(pending_frames_.front());
        pending_frames_.pop_front();
        last_callback_frame_ = pending;
        has_last_callback_frame_ = true;
        start_output(current_output_, pending, type, data, data_length);
      } else if (!current_output_.data.empty()) {
        append_output(current_output_, type, data, data_length);
        ++additional_callbacks_;
      } else if (has_last_callback_frame_) {
        start_output(current_output_, last_callback_frame_, type, data, data_length);
        ++additional_callbacks_;
      } else {
        ++unmatched_callbacks_;
        RCLCPP_WARN_THROTTLE(
          logger_, clock_, 5000,
          "%s received encoder data before any submitted frame: type=%u size=%d",
          input_topic_.c_str(), static_cast<unsigned int>(type), data_length);
        return;
      }
    }
    if (do_publish) {
      publisher_->publish(std::move(to_publish));
      ++encoded_frames_;
    }
  }

  static void start_output(
    CompressedVideo & out, const PendingFrame & pending,
    unsigned char type, unsigned char * data, int data_length)
  {
    out.header = pending.header;
    out.codec = "h265_annexb_chunk";
    out.source_encoding = pending.source_encoding;
    out.width = pending.width;
    out.height = pending.height;
    out.frame_sequence = pending.sequence;
    out.key_frame = (type == 1);
    out.data.assign(data, data + data_length);
  }

  static void append_output(
    CompressedVideo & out, unsigned char type, unsigned char * data, int data_length)
  {
    out.data.insert(out.data.end(), data, data + data_length);
    if (type == 1) {
      out.key_frame = true;
    }
  }

  rclcpp::Logger logger_;
  rclcpp::Clock clock_{RCL_SYSTEM_TIME};
  std::string input_topic_;
  rclcpp::Publisher<CompressedVideo>::SharedPtr publisher_;
  VideoEncoderApi & api_;
  unsigned int bitrate_;
  unsigned char fps_;
  unsigned int gop_;
  std::size_t max_pending_frames_;
  std::int64_t interval_ns_{0};
  std::int64_t last_slot_{-1};
  void * handle_{nullptr};
  std::uint32_t width_{0};
  std::uint32_t height_{0};
  std::vector<unsigned char> i420_buffer_;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  RawFrame queued_frame_;
  bool has_queued_frame_{false};
  bool stopping_{false};
  std::thread worker_;
  mutable std::mutex mutex_;
  std::deque<PendingFrame> pending_frames_;
  PendingFrame last_callback_frame_;
  bool has_last_callback_frame_{false};
  CompressedVideo current_output_;
  std::uint64_t next_sequence_{0};
  std::atomic<std::uint64_t> received_frames_{0};
  std::atomic<std::uint64_t> chunks_{0};
  std::atomic<std::uint64_t> encoded_frames_{0};
  std::atomic<std::uint64_t> queue_drops_{0};
  std::atomic<std::uint64_t> rejected_frames_{0};
  std::atomic<std::uint64_t> throttled_frames_{0};
  std::atomic<std::uint64_t> input_failures_{0};
  std::atomic<std::uint64_t> empty_callbacks_{0};
  std::atomic<std::uint64_t> additional_callbacks_{0};
  std::atomic<std::uint64_t> unmatched_callbacks_{0};
};

class H265EncoderNode : public rclcpp::Node
{
public:
  H265EncoderNode()
  : Node("shm_image6m_h265_encoder_6ch")
  {
    const auto input_topics = declare_parameter<std::vector<std::string>>(
      "input_topics",
      {"/camera/fisheye_back/shm_image_6m",
        "/camera/pinhole_back/shm_image_6m",
        "/camera/fisheye_right/shm_image_6m",
        "/camera/fisheye_left/shm_image_6m",
        "/camera/pinhole_front/shm_image_6m",
        "/camera/fisheye_front/shm_image_6m"});
    const auto output_topics = declare_parameter<std::vector<std::string>>(
      "output_topics",
      {"/camera/fisheye_back/h265",
        "/camera/pinhole_back/h265",
        "/camera/fisheye_right/h265",
        "/camera/fisheye_left/h265",
        "/camera/pinhole_front/h265",
        "/camera/fisheye_front/h265"});
    if (input_topics.size() != output_topics.size() || input_topics.empty()) {
      throw std::runtime_error(
              "input_topics and output_topics must have equal non-zero length");
    }

    const std::string library = declare_parameter("videoenc_library", "libvideoenc.so");
    const std::string yuv_library = declare_parameter("libyuv_library", "libyuv.so");
    const int bitrate_mbps = declare_parameter("bitrate_mbps", 15);
    const int fps = declare_parameter("fps", 10);
    const int gop = declare_parameter("gop", 10);
    const int max_pending = declare_parameter("max_pending_frames", 120);
    if (bitrate_mbps <= 0 || fps <= 0 || fps > 255 || gop <= 0 || max_pending <= 0) {
      throw std::runtime_error("Invalid encoder parameter");
    }

    api_ = std::make_unique<VideoEncoderApi>(library, yuv_library);
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    // Match the camera SDK's sensor-stream QoS. A best-effort subscriber is
    // compatible with both best-effort and reliable publishers and avoids
    // applying DDS backpressure to the capture process.
    const auto input_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(60)).reliable().durability_volatile();
    const unsigned int bitrate =
      static_cast<unsigned int>(bitrate_mbps) * 1024U * 1024U;

    for (std::size_t i = 0; i < input_topics.size(); ++i) {
      auto publisher = create_publisher<CompressedVideo>(output_topics[i], output_qos);
      auto stream = std::make_shared<H265Stream>(
        get_logger(), input_topics[i], publisher, *api_, bitrate,
        static_cast<unsigned char>(fps), static_cast<unsigned int>(gop),
        static_cast<std::size_t>(max_pending));
      auto subscription = create_subscription<Image6m>(
        input_topics[i], input_qos,
        [stream](const Image6m::SharedPtr input) {stream->encode(input);}, options);
      streams_.push_back(std::move(stream));
      subscriptions_.push_back(std::move(subscription));
      RCLCPP_INFO(
        get_logger(), "%s -> %s", input_topics[i].c_str(), output_topics[i].c_str());
    }

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this]() {
        for (const auto & stream : streams_) {
          stream->log_stats();
        }
      });
  }

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<VideoEncoderApi> api_;
  std::vector<std::shared_ptr<H265Stream>> streams_;
  std::vector<rclcpp::Subscription<Image6m>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace compression
}  // namespace shm_msgs

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<shm_msgs::compression::H265EncoderNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("shm_image6m_h265_encoder_6ch"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
