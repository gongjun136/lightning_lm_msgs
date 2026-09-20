#include <opencv2/opencv.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "shm_msgs/msg/image6m.hpp"
#include "shm_msgs/array_helper.hpp"

namespace
{

using Image6m = shm_msgs::msg::Image6m;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

constexpr const char * kImageType = "shm_msgs/msg/Image6m";
constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";

struct CameraSpec
{
  std::string topic;
  std::string name;
};

// 与 perception_swc 的 obstacle_camera_order 对齐:
//   front_fisheye / left_fisheye / right_fisheye / rear_fisheye
const std::vector<CameraSpec> kCameraSpecs = {
  {"/camera/fisheye_front/shm_image_6m", "front_fisheye"},
  {"/camera/fisheye_left/shm_image_6m", "left_fisheye"},
  {"/camera/fisheye_right/shm_image_6m", "right_fisheye"},
  {"/camera/fisheye_back/shm_image_6m", "rear_fisheye"},
};

struct Options
{
  std::string bag;
  std::string out;
  std::string storage_id;
  std::string lidar_topic{"/LidarDataInv"};
  std::vector<std::string> camera_topics;
  bool sync{false};
  double sync_tolerance_ms{50.0};
  std::uint64_t max_frames{0};
  std::string image_ext{"png"};
};

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

std::string bag_basename(const std::string & bag)
{
  std::filesystem::path p(bag);
  std::filesystem::path name_path = p.filename();
  if (name_path.empty()) {
    name_path = p.parent_path().filename();
  }
  std::string name = name_path.string();
  const std::string ext = name_path.extension().string();
  if (ext == ".mcap" || ext == ".db3") {
    name = name_path.stem().string();
  }
  if (name.empty()) {
    name = "bag_output";
  }
  return name;
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

// 复刻 perception_node::cameraCallback 的 encoding -> BGR 逻辑:
//   yuv422 / yuv422_yuy2 -> COLOR_YUV2BGR_YUY2; bgr8 原样; 其余按 BGR 回退。
cv::Mat decode_to_bgr(const Image6m & msg)
{
  const std::string encoding = shm_msgs::get_str(msg.encoding);
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);

  if (encoding == "bgr8") {
    const std::size_t step =
      (msg.step >= static_cast<std::uint32_t>(width * 3)) ? msg.step : width * 3;
    cv::Mat raw(height, width, CV_8UC3, const_cast<std::uint8_t *>(msg.data.data()), step);
    return raw.clone();
  }

  if (encoding == "yuv422" || encoding == "yuv422_yuy2") {
    const std::size_t step =
      (msg.step >= static_cast<std::uint32_t>(width * 2)) ? msg.step : width * 2;
    cv::Mat raw(height, width, CV_8UC2, const_cast<std::uint8_t *>(msg.data.data()), step);
    cv::Mat bgr;
    cv::cvtColor(raw, bgr, cv::COLOR_YUV2BGR_YUY2);
    return bgr;
  }

  std::cerr << "warning: unsupported encoding '" << encoding << "', fallback to bgr8\n";
  const std::size_t step =
    (msg.step >= static_cast<std::uint32_t>(width * 3)) ? msg.step : width * 3;
  cv::Mat raw(height, width, CV_8UC3, const_cast<std::uint8_t *>(msg.data.data()), step);
  return raw.clone();
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

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program << " --bag <path> --out <dir> [options]\n"
    << "\n"
    << "Reads a rosbag2 bag (db3 or mcap) directly and extracts perception_swc\n"
    << "obstacle inputs:\n"
    << "  - shm_msgs/msg/Image6m -> BGR image files (timestamp-named)\n"
    << "  - sensor_msgs/msg/PointCloud2 -> binary PCD (x y z intensity timestamp)\n"
    << "\n"
    << "Options:\n"
    << "  --bag <path>              Input bag (.mcap file, or db3/mcap dir)\n"
    << "  --out <dir>               Output directory; results go to <dir>/<bag_name>/\n"
    << "  --storage-id <id>         'mcap' or 'sqlite3'; empty = auto-detect\n"
    << "  --lidar-topic <t>         LiDAR topic (default /LidarDataInv)\n"
    << "  --camera-topics <a,b,..>  Override the 4 fisheye Image6m topics\n"
    << "  --image-ext <ext>         png (default) or jpg\n"
    << "  --max-frames <n>          Max frames per stream, 0 = unlimited\n"
    << "  --sync                    Aligned set -> frame_<lidar_ts>/ (png named by cam ts)\n"
    << "  --sync-tolerance-ms <ms>  Sync window for --sync (default 50)\n"
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
    } else if (argument == "--max-frames") {
      options.max_frames = std::stoull(require_value(index));
    } else if (argument == "--sync") {
      options.sync = true;
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
  if (options.image_ext.empty()) {
    options.image_ext = "png";
  }
  if (options.image_ext.front() == '.') {
    options.image_ext.erase(0, 1);
  }
  return options;
}

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

void write_summary(const std::filesystem::path & output_root,
                   const Options & options,
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

  out << "bag_to_obstacle_input summary\n";
  out << "bag: " << options.bag << "\n";
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

int run(const Options & options)
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage;
  storage.uri = options.bag;
  storage.storage_id = resolve_storage_id(options.bag, options.storage_id);
  reader.open(storage);

  std::vector<std::string> image_topics;
  std::string lidar_topic;
  for (const auto & topic : reader.get_all_topics_and_types()) {
    if (topic.type == kImageType) {
      if (options.camera_topics.empty() ||
        std::find(options.camera_topics.begin(), options.camera_topics.end(), topic.name) !=
          options.camera_topics.end())
      {
        image_topics.push_back(topic.name);
      }
    } else if (topic.type == kPointCloudType && topic.name == options.lidar_topic) {
      lidar_topic = topic.name;
    }
  }

  if (image_topics.empty() && lidar_topic.empty()) {
    std::cerr << "no matching Image6m / PointCloud2 topics found in bag\n";
    return 1;
  }

  const std::string bag_name = bag_basename(options.bag) + "_clip";
  const std::filesystem::path output_root =
    std::filesystem::path(options.out) / bag_name;
  std::filesystem::create_directories(output_root);

  rosbag2_storage::StorageFilter filter;
  filter.topics = image_topics;
  if (!lidar_topic.empty()) {
    filter.topics.push_back(lidar_topic);
  }
  reader.set_filter(filter);

  ExtractedStreams streams;
  std::map<std::string, std::uint64_t> per_stream_count;

  rclcpp::Serialization<Image6m> image_serialization;
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
      streams.lidar_frames.push_back(parse_pointcloud(cloud, ts_ns));
      ++per_stream_count[lidar_topic];
    } else {
      const auto it = std::find(image_topics.begin(), image_topics.end(), message->topic_name);
      if (it == image_topics.end()) {
        continue;
      }
      if (options.max_frames != 0 &&
        per_stream_count[message->topic_name] >= options.max_frames)
      {
        continue;
      }
      Image6m image;
      rclcpp::SerializedMessage serialized(*message->serialized_data);
      image_serialization.deserialize_message(&serialized, &image);
      const std::uint64_t ts_ns = stamp_to_ns(image.header.stamp);
      cv::Mat bgr = decode_to_bgr(image);
      streams.images[message->topic_name].emplace_back(ts_ns, std::move(bgr));
      ++per_stream_count[message->topic_name];
    }
  }

  std::vector<SyncMatch> sync_matches;

  if (!options.sync) {
    for (const auto & topic : image_topics) {
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
    write_summary(output_root, options, image_topics, lidar_topic, streams, sync_matches);
    return 0;
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
  std::cout << "sync mode: wrote " << grouped << " aligned frames into " << output_root
            << "/frame_<lidar_ts>/ (files named by their own ts)\n";
  write_summary(output_root, options, image_topics, lidar_topic, streams, sync_matches);
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int return_code = 0;
  try {
    const Options options = parse_args(argc, argv);
    return_code = run(options);
  } catch (const std::exception & error) {
    std::cerr << "bag_to_obstacle_input: " << error.what() << "\n";
    return_code = 1;
  }
  rclcpp::shutdown();
  return return_code;
}
