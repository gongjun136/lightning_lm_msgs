#include <opencv2/opencv.hpp>

#include "shm_msgs/msg/compressed_video.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
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
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using CompressedVideo = shm_msgs::msg::CompressedVideo;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

constexpr const char * kVideoType = "shm_msgs/msg/CompressedVideo";
constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";
constexpr const char * kCodec = "h265_annexb_chunk";

struct CameraSpec
{
  std::string topic;
  std::string name;
};

// 与 perception_swc 的 obstacle_camera_order 对齐 (h265 编码源):
//   front_fisheye / left_fisheye / right_fisheye / rear_fisheye
const std::vector<CameraSpec> kCameraSpecs = {
  {"/camera/fisheye_front/h265", "front_fisheye"},
  {"/camera/fisheye_left/h265", "left_fisheye"},
  {"/camera/fisheye_right/h265", "right_fisheye"},
  {"/camera/fisheye_back/h265", "rear_fisheye"},
};

struct Options
{
  std::string bag;
  std::string out;
  std::string storage_id;
  std::string lidar_topic{"/LidarDataInv"};
  std::vector<std::string> camera_topics;
  bool sync{true};
  double sync_tolerance_ms{50.0};
  std::uint64_t max_frames{0};
  std::string image_ext{"png"};
  int decoder_threads{1};
};

std::string ffmpeg_error(int error)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
  av_strerror(error, buffer.data(), buffer.size());
  return buffer.data();
}

std::string trim(const std::string & input)
{
  const std::size_t begin = input.find_first_not_of(" \t\r\n\"'");
  if (begin == std::string::npos) {
    return "";
  }
  const std::size_t end = input.find_last_not_of(" \t\r\n\"'");
  return input.substr(begin, end - begin + 1);
}

void split(const std::string & input, char delimiter, std::vector<std::string> & output)
{
  std::size_t start = 0;
  while (true) {
    const std::size_t position = input.find(delimiter, start);
    const std::string token = input.substr(
      start, position == std::string::npos ? std::string::npos : position - start);
    if (!token.empty()) {
      output.push_back(token);
    }
    if (position == std::string::npos) {
      break;
    }
    start = position + 1;
  }
}

std::string resolve_storage_id(const std::string & bag, const std::string & explicit_id)
{
  if (!explicit_id.empty()) {
    return explicit_id;
  }
  const std::filesystem::path metadata = std::filesystem::path(bag) / "metadata.yaml";
  if (std::filesystem::is_regular_file(metadata)) {
    std::ifstream input(metadata);
    std::string line;
    while (std::getline(input, line)) {
      const std::size_t position = line.find("storage_identifier");
      if (position == std::string::npos) {
        continue;
      }
      const std::size_t colon = line.find(':', position);
      if (colon != std::string::npos) {
        const std::string identifier = trim(line.substr(colon + 1));
        if (!identifier.empty()) {
          return identifier;
        }
      }
    }
  }
  if (std::filesystem::is_directory(bag)) {
    return "sqlite3";
  }
  if (std::filesystem::path(bag).extension() == ".mcap") {
    return "mcap";
  }
  return "sqlite3";
}

std::string topic_leaf(const std::string & topic)
{
  const std::size_t slash = topic.find_last_of('/');
  if (slash == std::string::npos) {
    return topic;
  }
  return topic.substr(slash + 1);
}

std::string camera_name_for(const std::string & topic)
{
  for (const auto & spec : kCameraSpecs) {
    if (topic == spec.topic) {
      return spec.name;
    }
  }
  return topic_leaf(topic);
}

std::uint64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.nanosec);
}

std::string make_image_name(std::uint64_t ts_ns, const std::string & cam, const std::string & ext)
{
  return std::to_string(ts_ns) + "_" + cam + "." + ext;
}

std::string make_pcd_name(std::uint64_t ts_ns)
{
  return std::to_string(ts_ns) + "_lidar.pcd";
}

bool save_image(const cv::Mat & bgr, const std::string & path)
{
  if (bgr.empty()) {
    return false;
  }
  std::vector<int> params;
  const std::string ext = std::filesystem::path(path).extension().string();
  if (ext == ".jpg" || ext == ".jpeg") {
    params = {cv::IMWRITE_JPEG_QUALITY, 95};
  } else {
    params = {cv::IMWRITE_PNG_COMPRESSION, 1};
  }
  return cv::imwrite(path, bgr, params);
}

struct BagEntry
{
  std::string uri;
  std::string storage_id;
  std::string name;
};

std::vector<BagEntry> discover_bags(const std::string & path, const std::string & storage_id)
{
  std::vector<BagEntry> entries;
  const std::filesystem::path root(path);

  auto push = [&](const std::filesystem::path & bag_path, const std::string & name) {
    entries.push_back(
      BagEntry{bag_path.string(), resolve_storage_id(bag_path.string(), storage_id), name});
  };

  if (std::filesystem::is_regular_file(root)) {
    push(root, root.stem().string());
    return entries;
  }
  if (!std::filesystem::is_directory(root)) {
    throw std::runtime_error("input path does not exist: " + path);
  }
  if (std::filesystem::is_regular_file(root / "metadata.yaml")) {
    push(root, root.filename().string());
    return entries;
  }

  for (const auto & entry : std::filesystem::directory_iterator(root)) {
    if (entry.is_directory() &&
      std::filesystem::is_regular_file(entry.path() / "metadata.yaml"))
    {
      push(entry.path(), entry.path().filename().string());
    } else if (entry.is_regular_file() && entry.path().extension() == ".mcap") {
      push(entry.path(), entry.path().stem().string());
    }
  }

  std::sort(entries.begin(), entries.end(), [](const BagEntry & left, const BagEntry & right) {
      return left.name < right.name;
    });
  return entries;
}

struct FieldSpec
{
  std::size_t offset = 0;
  std::uint8_t datatype = 0;
};

std::optional<FieldSpec> find_field(const PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      FieldSpec spec;
      spec.offset = field.offset;
      spec.datatype = field.datatype;
      return spec;
    }
  }
  return std::nullopt;
}

float read_as_float(const std::uint8_t * base, const FieldSpec & spec)
{
  switch (spec.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: {
      float v;
      std::memcpy(&v, base + spec.offset, sizeof(float));
      return v;
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      double v;
      std::memcpy(&v, base + spec.offset, sizeof(double));
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::UINT8: {
      std::uint8_t v;
      std::memcpy(&v, base + spec.offset, sizeof(std::uint8_t));
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::UINT16: {
      std::uint16_t v;
      std::memcpy(&v, base + spec.offset, sizeof(std::uint16_t));
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::UINT32: {
      std::uint32_t v;
      std::memcpy(&v, base + spec.offset, sizeof(std::uint32_t));
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::INT8: {
      std::int8_t v;
      std::memcpy(&v, base + spec.offset, sizeof(std::int8_t));
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::INT16: {
      std::int16_t v;
      std::memcpy(&v, base + spec.offset, sizeof(std::int16_t));
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::INT32: {
      std::int32_t v;
      std::memcpy(&v, base + spec.offset, sizeof(std::int32_t));
      return static_cast<float>(v);
    }
    default:
      return 0.0f;
  }
}

double read_as_double(const std::uint8_t * base, const FieldSpec & spec)
{
  if (spec.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    double v;
    std::memcpy(&v, base + spec.offset, sizeof(double));
    return v;
  }
  return static_cast<double>(read_as_float(base, spec));
}

struct PointXYZIT
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float intensity = 0.0f;
  double timestamp = 0.0;
};

struct LidarFrame
{
  std::uint64_t ts_ns = 0;
  std::vector<PointXYZIT> points;
};

LidarFrame parse_pointcloud(const PointCloud2 & msg, std::uint64_t ts_ns)
{
  LidarFrame frame;
  frame.ts_ns = ts_ns;

  const auto x = find_field(msg, "x");
  const auto y = find_field(msg, "y");
  const auto z = find_field(msg, "z");
  auto intensity = find_field(msg, "intensity");
  if (!intensity) {
    intensity = find_field(msg, "i");
  }
  auto timestamp = find_field(msg, "timestamp");
  if (!timestamp) {
    timestamp = find_field(msg, "t");
  }

  if (!x || !y || !z) {
    throw std::runtime_error("PointCloud2 missing x/y/z fields");
  }

  const std::size_t point_count =
    static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
  frame.points.reserve(point_count);

  const std::uint8_t * data = msg.data.data();
  for (std::size_t i = 0; i < point_count; ++i) {
    const std::uint8_t * base = data + i * msg.point_step;
    PointXYZIT p;
    p.x = read_as_float(base, *x);
    p.y = read_as_float(base, *y);
    p.z = read_as_float(base, *z);
    p.intensity = intensity ? read_as_float(base, *intensity) : 0.0f;
    p.timestamp = timestamp ? read_as_double(base, *timestamp) : 0.0;
    frame.points.push_back(p);
  }

  return frame;
}

bool write_pcd(const LidarFrame & frame, const std::string & path)
{
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    return false;
  }

  out << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z intensity timestamp\n"
      << "SIZE 4 4 4 4 8\n"
      << "TYPE F F F F F\n"
      << "COUNT 1 1 1 1 1\n"
      << "WIDTH " << frame.points.size() << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << frame.points.size() << "\n"
      << "DATA binary\n";

  for (const auto & p : frame.points) {
    out.write(reinterpret_cast<const char *>(&p.x), sizeof(float));
    out.write(reinterpret_cast<const char *>(&p.y), sizeof(float));
    out.write(reinterpret_cast<const char *>(&p.z), sizeof(float));
    out.write(reinterpret_cast<const char *>(&p.intensity), sizeof(float));
    out.write(reinterpret_cast<const char *>(&p.timestamp), sizeof(double));
  }

  out.close();
  return true;
}

struct FrameMetadata
{
  std_msgs::msg::Header header;
  std::uint64_t sequence{0};
  bool key_frame{false};
};

// 复刻 h265_decoder_readfile 的 StreamDecoder, 但不写新 bag, 而是把解码后的
// BGR 帧 (时间戳, cv::Mat) 留在内存里供后续时间戳匹配使用。
class StreamDecoder
{
public:
  StreamDecoder(std::string topic, std::uint64_t max_frames, int decoder_threads)
  : topic_(std::move(topic)), max_frames_(max_frames)
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
    codec_context_->thread_type = FF_THREAD_SLICE;
    const int open_result = avcodec_open2(codec_context_, codec, nullptr);
    if (open_result < 0) {
      throw std::runtime_error(
              "Failed to open FFmpeg HEVC decoder: " + ffmpeg_error(open_result));
    }
  }

  ~StreamDecoder()
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

  void ingest(const CompressedVideo & message)
  {
    ++chunks_;
    if (message.codec != kCodec || message.data.empty()) {
      ++decode_errors_;
      std::cerr << topic_ << ": unsupported or empty codec payload '"
                << message.codec << "'\n";
      return;
    }

    if (!has_current_frame_) {
      if (has_submitted_sequence_ && message.frame_sequence <= submitted_sequence_) {
        ++late_chunks_;
        return;
      }
      begin_frame(message);
    } else if (message.frame_sequence != current_metadata_.sequence) {
      if (message.frame_sequence < current_metadata_.sequence) {
        ++late_chunks_;
        return;
      }
      const std::uint64_t previous_sequence = current_metadata_.sequence;
      submit_current_frame();
      if (message.frame_sequence > previous_sequence + 1U) {
        sequence_gaps_ += message.frame_sequence - previous_sequence - 1U;
      }
      begin_frame(message);
    }

    current_packet_.insert(
      current_packet_.end(), message.data.begin(), message.data.end());
  }

  void flush()
  {
    submit_current_frame();
    if (avcodec_send_packet(codec_context_, nullptr) >= 0) {
      drain_frames();
    }
  }

  void log_stats() const
  {
    std::cout << topic_ << ": chunks=" << chunks_
              << " packets=" << packets_ << " decoded=" << decoded_frames_
              << " sequence_gaps=" << sequence_gaps_ << " late_chunks=" << late_chunks_
              << " decode_errors=" << decode_errors_ << "\n";
  }

  std::vector<std::pair<std::uint64_t, cv::Mat>> take_frames()
  {
    return std::move(frames_);
  }

private:
  void begin_frame(const CompressedVideo & message)
  {
    current_metadata_.header = message.header;
    current_metadata_.sequence = message.frame_sequence;
    current_metadata_.key_frame = message.key_frame;
    current_packet_.clear();
    has_current_frame_ = true;
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
      std::cerr << topic_ << ": failed to allocate an FFmpeg packet\n";
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
      std::cerr << topic_ << ": failed to submit H.265 packet: "
                << ffmpeg_error(result) << "\n";
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
        std::cerr << topic_ << ": failed to receive decoded frame: "
                  << ffmpeg_error(result) << "\n";
        return;
      }
      if (pending_metadata_.empty()) {
        ++decode_errors_;
        av_frame_unref(frame_);
        continue;
      }

      FrameMetadata metadata = std::move(pending_metadata_.front());
      pending_metadata_.pop_front();
      write_frame(metadata);
      av_frame_unref(frame_);
    }
  }

  void write_frame(const FrameMetadata & metadata)
  {
    sws_context_ = sws_getCachedContext(
      sws_context_, frame_->width, frame_->height,
      static_cast<AVPixelFormat>(frame_->format), frame_->width, frame_->height,
      AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (sws_context_ == nullptr) {
      ++decode_errors_;
      return;
    }

    cv::Mat bgr(frame_->height, frame_->width, CV_8UC3);
    if (bgr.empty()) {
      ++decode_errors_;
      return;
    }
    std::array<std::uint8_t *, 4> destination_data{bgr.data, nullptr, nullptr, nullptr};
    std::array<int, 4> destination_linesize{static_cast<int>(bgr.step), 0, 0, 0};
    const int converted_rows = sws_scale(
      sws_context_, frame_->data, frame_->linesize, 0, frame_->height,
      destination_data.data(), destination_linesize.data());
    if (converted_rows != frame_->height) {
      ++decode_errors_;
      return;
    }

    ++decoded_frames_;
    if (max_frames_ != 0 && decoded_frames_ > max_frames_) {
      return;
    }

    const std::uint64_t ts_ns = stamp_to_ns(metadata.header.stamp);
    frames_.emplace_back(ts_ns, std::move(bgr));
  }

  std::string topic_;
  std::uint64_t max_frames_{0};
  AVCodecContext * codec_context_{nullptr};
  AVFrame * frame_{nullptr};
  SwsContext * sws_context_{nullptr};
  FrameMetadata current_metadata_;
  std::vector<std::uint8_t> current_packet_;
  std::deque<FrameMetadata> pending_metadata_;
  std::uint64_t submitted_sequence_{0};
  bool has_current_frame_{false};
  bool has_submitted_sequence_{false};
  std::uint64_t chunks_{0};
  std::uint64_t packets_{0};
  std::uint64_t decoded_frames_{0};
  std::uint64_t sequence_gaps_{0};
  std::uint64_t late_chunks_{0};
  std::uint64_t decode_errors_{0};
  std::vector<std::pair<std::uint64_t, cv::Mat>> frames_;
};

struct ExtractedStreams
{
  std::map<std::string, std::vector<std::pair<std::uint64_t, cv::Mat>>> images;
  std::vector<LidarFrame> lidar_frames;
};

struct SyncMatch
{
  std::uint64_t lidar_ts = 0;
  std::array<std::uint64_t, 4> cam_ts{};
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program << " --bag <path> --out <dir> [options]\n"
    << "\n"
    << "Reads a rosbag2 bag (db3 or mcap) of shm_msgs/msg/CompressedVideo\n"
    << "(H.265) camera streams plus a sensor_msgs/msg/PointCloud2 LiDAR stream,\n"
    << "decodes every H.265 stream to BGR, matches camera frames to each LiDAR\n"
    << "frame by timestamp, and writes aligned clips:\n"
    << "  <out>/<bag_name>_clip/frame_<lidar_ts>/\n"
    << "    <cam_ts>_<cam>.png   (4 fisheye cameras)\n"
    << "    <lidar_ts>_lidar.pcd (x y z intensity timestamp)\n"
    << "\n"
    << "--bag may be a single bag (.mcap file, or db3/mcap dir) or a directory\n"
    << "containing multiple bags; every bag under it is processed in batch.\n"
    << "\n"
    << "Options:\n"
    << "  --bag <path>              Input bag, or a directory of bags (required)\n"
    << "  --out <dir>               Output dir; results go to <dir>/<bag_name>_clip/\n"
    << "  --storage-id <id>         Input storage: 'mcap' or 'sqlite3'; empty = auto\n"
    << "  --lidar-topic <t>         LiDAR topic (default /LidarDataInv)\n"
    << "  --camera-topics <a,b,..>  Override the 4 fisheye h265 topics\n"
    << "  --image-ext <ext>         png (default) or jpg\n"
    << "  --decoder-threads <n>     FFmpeg threads per camera (default 1)\n"
    << "  --max-frames <n>          Max frames per stream, 0 = unlimited\n"
    << "  --flat                    No sync: dump all decoded images + pcd flat\n"
    << "  --sync                    Force sync mode (default)\n"
    << "  --sync-tolerance-ms <ms>  Sync window for matching (default 50)\n"
    << "  -h, --help                Show this help\n";
}

Options parse_args(int argc, char ** argv)
{
  Options options;
  auto require_value = [&](int & index) -> std::string {
    if (index + 1 >= argc) {
      throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return argv[++index];
  };

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--bag") {
      options.bag = require_value(index);
    } else if (argument == "--out") {
      options.out = require_value(index);
    } else if (argument == "--storage-id") {
      options.storage_id = require_value(index);
    } else if (argument == "--lidar-topic") {
      options.lidar_topic = require_value(index);
    } else if (argument == "--camera-topics") {
      split(require_value(index), ',', options.camera_topics);
    } else if (argument == "--image-ext") {
      options.image_ext = require_value(index);
    } else if (argument == "--decoder-threads") {
      options.decoder_threads = std::stoi(require_value(index));
    } else if (argument == "--max-frames") {
      options.max_frames = std::stoull(require_value(index));
    } else if (argument == "--sync") {
      options.sync = true;
    } else if (argument == "--flat") {
      options.sync = false;
    } else if (argument == "--sync-tolerance-ms") {
      options.sync_tolerance_ms = std::stod(require_value(index));
    } else if (argument == "-h" || argument == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }

  if (options.bag.empty()) {
    throw std::runtime_error("--bag is required");
  }
  if (options.out.empty()) {
    throw std::runtime_error("--out is required");
  }
  if (options.decoder_threads <= 0) {
    throw std::runtime_error("--decoder-threads must be positive");
  }
  if (options.image_ext.empty()) {
    options.image_ext = "png";
  }
  if (options.image_ext.front() == '.') {
    options.image_ext.erase(0, 1);
  }
  return options;
}

void write_summary(const std::filesystem::path & output_root,
                   const Options & options,
                   const std::string & bag_uri,
                   const std::vector<std::string> & image_topics,
                   const std::string & lidar_topic,
                   const ExtractedStreams & streams,
                   const std::vector<SyncMatch> & sync_matches)
{
  const std::filesystem::path summary_path = output_root / "extract_summary.txt";
  std::ofstream out(summary_path);
  if (!out.is_open()) {
    std::cerr << "failed to write summary " << summary_path << "\n";
    return;
  }

  out << "mergelidar_h265_clip summary\n";
  out << "bag: " << bag_uri << "\n";
  out << "output: " << output_root.string() << "\n";
  out << "mode: " << (options.sync ? "sync" : "flat") << "\n\n";

  out << "=== frame counts ===\n";
  out << "lidar (" << lidar_topic << "): " << streams.lidar_frames.size() << " frames\n";
  for (const auto & topic : image_topics) {
    const std::string cam = camera_name_for(topic);
    const std::size_t n =
      streams.images.count(topic) ? streams.images.at(topic).size() : 0;
    out << cam << " (" << topic << "): " << n << " frames\n";
  }

  if (!sync_matches.empty()) {
    out << "\n=== matched frames (lidar ts -> camera ts) ===\n";
    for (std::size_t i = 0; i < sync_matches.size(); ++i) {
      const auto & m = sync_matches[i];
      out << "[" << i << "] lidar=" << m.lidar_ts;
      for (std::size_t c = 0; c < kCameraSpecs.size(); ++c) {
        out << "  " << kCameraSpecs[c].name << "=" << m.cam_ts[c];
      }
      out << "\n";
    }
    out << "\nmatched frames: " << sync_matches.size() << " / "
        << streams.lidar_frames.size() << " lidar frames\n";
  }

  out.close();
  std::cout << "summary written to " << summary_path << "\n";
}

template <typename T>
std::optional<std::pair<std::uint64_t, const T *>> nearest(
  const std::vector<std::pair<std::uint64_t, T>> & items,
  std::uint64_t target,
  double tolerance_ms)
{
  const T * best = nullptr;
  std::uint64_t best_ts = 0;
  std::uint64_t best_diff = std::numeric_limits<std::uint64_t>::max();
  for (const auto & item : items) {
    const std::uint64_t diff =
      item.first > target ? item.first - target : target - item.first;
    if (diff < best_diff) {
      best_diff = diff;
      best_ts = item.first;
      best = &item.second;
    }
  }
  if (best == nullptr ||
    best_diff > static_cast<std::uint64_t>(tolerance_ms * 1e6))
  {
    return std::nullopt;
  }
  return std::make_pair(best_ts, best);
}

int process_bag(
  const Options & options,
  const BagEntry & bag,
  const std::filesystem::path & output_root)
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage;
  storage.uri = bag.uri;
  storage.storage_id = bag.storage_id;
  reader.open(storage);

  std::vector<std::string> video_topics;
  std::string lidar_topic;
  for (const auto & topic : reader.get_all_topics_and_types()) {
    if (topic.type == kVideoType) {
      if (options.camera_topics.empty() ||
        std::find(options.camera_topics.begin(), options.camera_topics.end(), topic.name) !=
          options.camera_topics.end())
      {
        video_topics.push_back(topic.name);
      }
    } else if (topic.type == kPointCloudType && topic.name == options.lidar_topic) {
      lidar_topic = topic.name;
    }
  }

  if (video_topics.empty() && lidar_topic.empty()) {
    std::cerr << bag.name << ": no matching CompressedVideo / PointCloud2 topics\n";
    return 1;
  }

  std::filesystem::create_directories(output_root);

  rosbag2_storage::StorageFilter filter;
  filter.topics = video_topics;
  if (!lidar_topic.empty()) {
    filter.topics.push_back(lidar_topic);
  }
  reader.set_filter(filter);

  std::map<std::string, std::unique_ptr<StreamDecoder>> decoders;
  for (const auto & topic : video_topics) {
    decoders.emplace(
      topic,
      std::make_unique<StreamDecoder>(topic, options.max_frames, options.decoder_threads));
  }

  std::vector<LidarFrame> lidar_frames;
  std::map<std::string, std::uint64_t> per_stream_count;

  rclcpp::Serialization<CompressedVideo> video_serialization;
  rclcpp::Serialization<PointCloud2> cloud_serialization;

  while (reader.has_next()) {
    const auto message = reader.read_next();

    if (message->topic_name == lidar_topic) {
      if (options.max_frames != 0 && per_stream_count[lidar_topic] >= options.max_frames) {
        continue;
      }
      PointCloud2 cloud;
      rclcpp::SerializedMessage serialized(*message->serialized_data);
      cloud_serialization.deserialize_message(&serialized, &cloud);
      const std::uint64_t ts_ns = stamp_to_ns(cloud.header.stamp);
      lidar_frames.push_back(parse_pointcloud(cloud, ts_ns));
      ++per_stream_count[lidar_topic];
    } else {
      const auto decoder = decoders.find(message->topic_name);
      if (decoder == decoders.end()) {
        continue;
      }
      CompressedVideo video;
      rclcpp::SerializedMessage serialized(*message->serialized_data);
      video_serialization.deserialize_message(&serialized, &video);
      decoder->second->ingest(video);
    }
  }

  for (const auto & entry : decoders) {
    entry.second->flush();
    entry.second->log_stats();
  }

  ExtractedStreams streams;
  for (auto & entry : decoders) {
    streams.images[entry.first] = entry.second->take_frames();
  }
  streams.lidar_frames = std::move(lidar_frames);

  std::vector<SyncMatch> sync_matches;

  if (!options.sync) {
    for (const auto & topic : video_topics) {
      const std::string cam = camera_name_for(topic);
      for (const auto & [ts_ns, bgr] : streams.images[topic]) {
        const std::string path =
          (output_root / make_image_name(ts_ns, cam, options.image_ext)).string();
        if (!save_image(bgr, path)) {
          std::cerr << "failed to write " << path << "\n";
        }
      }
      std::cout << topic << " -> " << streams.images[topic].size() << " image frames\n";
    }
    for (const auto & frame : streams.lidar_frames) {
      const std::string path = (output_root / make_pcd_name(frame.ts_ns)).string();
      if (!write_pcd(frame, path)) {
        std::cerr << "failed to write " << path << "\n";
      }
    }
    std::cout << lidar_topic << " -> " << streams.lidar_frames.size() << " point clouds\n";
    write_summary(output_root, options, bag.uri, video_topics, lidar_topic, streams, sync_matches);
    return 0;
  }

  if (lidar_topic.empty()) {
    std::cerr << bag.name << ": --sync requires a LiDAR topic, skipping\n";
    return 1;
  }

  std::size_t grouped = 0;
  for (const auto & lidar : streams.lidar_frames) {
    std::vector<std::pair<std::uint64_t, const cv::Mat *>> matched;
    bool complete = true;
    for (const auto & spec : kCameraSpecs) {
      const auto it = streams.images.find(spec.topic);
      if (it == streams.images.end()) {
        complete = false;
        break;
      }
      const auto found = nearest(it->second, lidar.ts_ns, options.sync_tolerance_ms);
      if (!found) {
        complete = false;
        break;
      }
      matched.push_back(*found);
    }
    if (!complete) {
      continue;
    }

    // 用 lidar 锚点时间戳建目录；png 用各自相机时间戳命名，保留原始采集时间戳
    const std::filesystem::path frame_dir =
      output_root / ("frame_" + std::to_string(lidar.ts_ns));
    std::filesystem::create_directories(frame_dir);

    SyncMatch m;
    m.lidar_ts = lidar.ts_ns;
    for (std::size_t i = 0; i < kCameraSpecs.size(); ++i) {
      const std::uint64_t cam_ts = matched[i].first;
      m.cam_ts[i] = cam_ts;
      const std::string path =
        (frame_dir / make_image_name(cam_ts, kCameraSpecs[i].name, options.image_ext)).string();
      save_image(*matched[i].second, path);
    }
    sync_matches.push_back(m);

    const std::string lidar_path = (frame_dir / make_pcd_name(lidar.ts_ns)).string();
    write_pcd(lidar, lidar_path);
    ++grouped;
  }
  std::cout << bag.name << ": sync mode: wrote " << grouped << " aligned frames into "
            << output_root << "/frame_<lidar_ts>/ (files named by their own ts)\n";
  write_summary(output_root, options, bag.uri, video_topics, lidar_topic, streams, sync_matches);
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int return_code = 0;
  try {
    const Options options = parse_args(argc, argv);

    const std::vector<BagEntry> bags = discover_bags(options.bag, options.storage_id);
    if (bags.empty()) {
      throw std::runtime_error("no bags found under " + options.bag);
    }

    std::size_t succeeded = 0;
    std::size_t failed = 0;
    for (const auto & bag : bags) {
      const std::string bag_name = bag.name + "_clip";
      const std::filesystem::path output_root =
        std::filesystem::path(options.out) / bag_name;
      try {
        if (process_bag(options, bag, output_root) == 0) {
          ++succeeded;
        } else {
          ++failed;
        }
      } catch (const std::exception & error) {
        std::cerr << bag.name << ": " << error.what() << "\n";
        ++failed;
      }
    }
    std::cout << "processed " << succeeded << " bag(s), " << failed << " failed\n";
    if (failed != 0) {
      return_code = 1;
    }
  } catch (const std::exception & error) {
    std::cerr << "mergelidar_h265_clip: " << error.what() << "\n";
    return_code = 1;
  }
  rclcpp::shutdown();
  return return_code;
}
