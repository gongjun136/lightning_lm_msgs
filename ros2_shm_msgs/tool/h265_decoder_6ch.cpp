#include "shm_msgs/msg/compressed_video.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using CompressedVideo = shm_msgs::msg::CompressedVideo;
using Image = sensor_msgs::msg::Image;
using SteadyClock = std::chrono::steady_clock;

std::string ffmpeg_error(int error)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
  av_strerror(error, buffer.data(), buffer.size());
  return buffer.data();
}

struct FrameMetadata
{
  std_msgs::msg::Header header;
  std::uint64_t sequence{0};
};

class H265Stream
{
public:
  H265Stream(
    rclcpp::Node & node,
    const std::string & input_topic,
    const std::string & output_topic,
    const rclcpp::CallbackGroup::SharedPtr & callback_group,
    int decoder_threads)
  : logger_(node.get_logger()),
    input_topic_(input_topic),
    output_topic_(output_topic)
  {
    const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (codec == nullptr) {
      throw std::runtime_error("FFmpeg HEVC decoder is unavailable");
    }
    codec_context_ = avcodec_alloc_context3(codec);
    frame_ = av_frame_alloc();
    if (codec_context_ == nullptr || frame_ == nullptr) {
      throw std::runtime_error("Failed to allocate FFmpeg decoder state");
    }
    codec_context_->thread_count = std::max(1, decoder_threads);
    // Slice threading avoids the tail-frame latency introduced by frame threading.
    // Each camera already has an independent decoder, so cross-camera parallelism
    // remains available even when a stream contains only one slice per frame.
    codec_context_->thread_type = FF_THREAD_SLICE;
    const int open_result = avcodec_open2(codec_context_, codec, nullptr);
    if (open_result < 0) {
      throw std::runtime_error(
              "Failed to open FFmpeg HEVC decoder: " + ffmpeg_error(open_result));
    }

    publisher_ = node.create_publisher<Image>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;
    subscription_ = node.create_subscription<CompressedVideo>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(512)).reliable().durability_volatile(),
      [this](const CompressedVideo::ConstSharedPtr message) { receive(message); }, options);

    RCLCPP_INFO(
      logger_, "H.265 decoding: %s -> %s (bgr8, FFmpeg threads=%d)",
      input_topic_.c_str(), output_topic_.c_str(), codec_context_->thread_count);
  }

  ~H265Stream()
  {
    if (sws_context_ != nullptr) {
      sws_freeContext(sws_context_);
    }
    if (frame_ != nullptr) {
      av_frame_free(&frame_);
    }
    if (codec_context_ != nullptr) {
      avcodec_free_context(&codec_context_);
    }
  }

  void flush_if_stale(SteadyClock::time_point now, std::chrono::milliseconds timeout)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_current_frame_ && now - last_chunk_time_ >= timeout) {
      submit_current_frame();
    }
  }

  void log_stats()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(
      logger_, "%s: chunks=%lu packets=%lu decoded=%lu sequence_gaps=%lu "
      "late_chunks=%lu decode_errors=%lu pending_metadata=%zu",
      input_topic_.c_str(), chunks_, packets_, decoded_frames_, sequence_gaps_,
      late_chunks_, decode_errors_, pending_metadata_.size());
  }

private:
  void receive(const CompressedVideo::ConstSharedPtr & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++chunks_;
    if (message->codec != "h265_annexb_chunk" || message->data.empty()) {
      ++decode_errors_;
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 5000, "%s received unsupported or empty codec payload '%s'",
        input_topic_.c_str(), message->codec.c_str());
      return;
    }

    if (!has_current_frame_) {
      if (has_submitted_sequence_ && message->frame_sequence <= submitted_sequence_) {
        ++late_chunks_;
        return;
      }
      begin_frame(*message);
    } else if (message->frame_sequence != current_metadata_.sequence) {
      if (message->frame_sequence < current_metadata_.sequence) {
        ++late_chunks_;
        return;
      }
      const std::uint64_t previous_sequence = current_metadata_.sequence;
      submit_current_frame();
      if (message->frame_sequence > previous_sequence + 1U) {
        sequence_gaps_ += message->frame_sequence - previous_sequence - 1U;
      }
      begin_frame(*message);
    }

    current_packet_.insert(
      current_packet_.end(), message->data.begin(), message->data.end());
    last_chunk_time_ = SteadyClock::now();
  }

  void begin_frame(const CompressedVideo & message)
  {
    current_metadata_.header = message.header;
    current_metadata_.sequence = message.frame_sequence;
    current_packet_.clear();
    has_current_frame_ = true;
    last_chunk_time_ = SteadyClock::now();
  }

  void submit_current_frame()
  {
    if (!has_current_frame_) {
      return;
    }
    has_current_frame_ = false;
    if (current_packet_.empty()) {
      ++decode_errors_;
      return;
    }

    AVPacket * packet = av_packet_alloc();
    if (packet == nullptr || av_new_packet(packet, static_cast<int>(current_packet_.size())) < 0) {
      if (packet != nullptr) {
        av_packet_free(&packet);
      }
      ++decode_errors_;
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 5000, "%s failed to allocate an FFmpeg packet",
        input_topic_.c_str());
      return;
    }
    std::memcpy(packet->data, current_packet_.data(), current_packet_.size());
    packet->pts = static_cast<std::int64_t>(current_metadata_.sequence);
    packet->dts = packet->pts;

    int result = avcodec_send_packet(codec_context_, packet);
    if (result == AVERROR(EAGAIN)) {
      drain_frames();
      result = avcodec_send_packet(codec_context_, packet);
    }
    av_packet_free(&packet);
    if (result < 0) {
      ++decode_errors_;
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 5000, "%s failed to submit H.265 packet: %s",
        input_topic_.c_str(), ffmpeg_error(result).c_str());
      return;
    }

    pending_metadata_.push_back(current_metadata_);
    submitted_sequence_ = current_metadata_.sequence;
    has_submitted_sequence_ = true;
    ++packets_;
    drain_frames();
  }

  void drain_frames()
  {
    while (true) {
      const int result = avcodec_receive_frame(codec_context_, frame_);
      if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
        return;
      }
      if (result < 0) {
        ++decode_errors_;
        RCLCPP_ERROR_THROTTLE(
          logger_, clock_, 5000, "%s failed to receive decoded frame: %s",
          input_topic_.c_str(), ffmpeg_error(result).c_str());
        return;
      }
      if (pending_metadata_.empty()) {
        ++decode_errors_;
        av_frame_unref(frame_);
        continue;
      }

      FrameMetadata metadata = std::move(pending_metadata_.front());
      pending_metadata_.pop_front();
      publish_frame(metadata);
      av_frame_unref(frame_);
    }
  }

  void publish_frame(const FrameMetadata & metadata)
  {
    sws_context_ = sws_getCachedContext(
      sws_context_, frame_->width, frame_->height,
      static_cast<AVPixelFormat>(frame_->format), frame_->width, frame_->height,
      AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (sws_context_ == nullptr) {
      ++decode_errors_;
      return;
    }

    Image output;
    output.header = metadata.header;
    output.height = static_cast<std::uint32_t>(frame_->height);
    output.width = static_cast<std::uint32_t>(frame_->width);
    output.encoding = "bgr8";
    output.is_bigendian = false;
    output.step = output.width * 3U;
    output.data.resize(static_cast<std::size_t>(output.step) * output.height);
    std::array<std::uint8_t *, 4> destination_data{
      output.data.data(), nullptr, nullptr, nullptr};
    std::array<int, 4> destination_linesize{
      static_cast<int>(output.step), 0, 0, 0};
    const int converted_rows = sws_scale(
      sws_context_, frame_->data, frame_->linesize, 0, frame_->height,
      destination_data.data(), destination_linesize.data());
    if (converted_rows != frame_->height) {
      ++decode_errors_;
      return;
    }

    publisher_->publish(std::move(output));
    ++decoded_frames_;
  }

  rclcpp::Logger logger_;
  rclcpp::Clock clock_{RCL_SYSTEM_TIME};
  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Publisher<Image>::SharedPtr publisher_;
  rclcpp::Subscription<CompressedVideo>::SharedPtr subscription_;
  AVCodecContext * codec_context_{nullptr};
  AVFrame * frame_{nullptr};
  SwsContext * sws_context_{nullptr};
  std::mutex mutex_;
  FrameMetadata current_metadata_;
  std::vector<std::uint8_t> current_packet_;
  std::deque<FrameMetadata> pending_metadata_;
  SteadyClock::time_point last_chunk_time_{SteadyClock::now()};
  std::uint64_t submitted_sequence_{0};
  bool has_current_frame_{false};
  bool has_submitted_sequence_{false};
  std::uint64_t chunks_{0};
  std::uint64_t packets_{0};
  std::uint64_t decoded_frames_{0};
  std::uint64_t sequence_gaps_{0};
  std::uint64_t late_chunks_{0};
  std::uint64_t decode_errors_{0};
};

class H265Decoder6Ch : public rclcpp::Node
{
public:
  H265Decoder6Ch()
  : Node("h265_decoder_6ch")
  {
    const int decoder_threads = declare_parameter<int>("decoder_threads_per_camera", 1);
    const int flush_timeout_ms = declare_parameter<int>("frame_flush_timeout_ms", 250);
    if (decoder_threads <= 0 || flush_timeout_ms < 50) {
      throw std::runtime_error(
              "decoder_threads_per_camera must be positive and frame_flush_timeout_ms >= 50");
    }
    flush_timeout_ = std::chrono::milliseconds(flush_timeout_ms);
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    const std::array<std::pair<const char *, const char *>, 6> topics{{
      {"/camera/fisheye_back/h265", "/camera/fisheye_back/image_raw"},
      {"/camera/fisheye_front/h265", "/camera/fisheye_front/image_raw"},
      {"/camera/fisheye_left/h265", "/camera/fisheye_left/image_raw"},
      {"/camera/fisheye_right/h265", "/camera/fisheye_right/image_raw"},
      {"/camera/pinhole_back/h265", "/camera/pinhole_back/image_raw"},
      {"/camera/pinhole_front/h265", "/camera/pinhole_front/image_raw"},
    }};
    streams_.reserve(topics.size());
    for (const auto & topics_for_stream : topics) {
      streams_.push_back(std::make_unique<H265Stream>(
          *this, topics_for_stream.first, topics_for_stream.second,
          callback_group_, decoder_threads));
    }

    last_stats_time_ = SteadyClock::now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {
        const auto now = SteadyClock::now();
        for (auto & stream : streams_) {
          stream->flush_if_stale(now, flush_timeout_);
        }
        if (now - last_stats_time_ >= std::chrono::seconds(5)) {
          for (auto & stream : streams_) {
            stream->log_stats();
          }
          last_stats_time_ = now;
        }
      });
  }

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::vector<std::unique_ptr<H265Stream>> streams_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds flush_timeout_{250};
  SteadyClock::time_point last_stats_time_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<H265Decoder6Ch>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 6U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("h265_decoder_6ch"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
