#include "shm_msgs/msg/compressed_video.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

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
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using CompressedVideo = shm_msgs::msg::CompressedVideo;
using Image = sensor_msgs::msg::Image;

constexpr const char * kVideoType = "shm_msgs/msg/CompressedVideo";
constexpr const char * kCodec = "h265_annexb_chunk";

std::string ffmpeg_error(int error)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
  av_strerror(error, buffer.data(), buffer.size());
  return buffer.data();
}

std::string trim(const std::string & input)
{
  std::size_t begin = input.find_first_not_of(" \t\r\n\"'");
  if (begin == std::string::npos) {
    return "";
  }
  std::size_t end = input.find_last_not_of(" \t\r\n\"'");
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

std::string image_topic_for(const std::string & input_topic)
{
  const std::size_t slash = input_topic.find_last_of('/');
  if (slash == std::string::npos) {
    return "/image_raw";
  }
  return input_topic.substr(0, slash) + "/image_raw";
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

struct FrameMetadata
{
  std_msgs::msg::Header header;
  std::uint64_t sequence{0};
  bool key_frame{false};
};

class StreamDecoder
{
public:
  StreamDecoder(
    std::string topic,
    std::string output_topic,
    rosbag2_cpp::Writer & writer,
    std::uint64_t max_frames,
    int decoder_threads)
  : topic_(std::move(topic)),
    output_topic_(std::move(output_topic)),
    writer_(writer),
    max_frames_(max_frames)
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
    std::cout << topic_ << " -> " << output_topic_ << ": chunks=" << chunks_
              << " packets=" << packets_ << " decoded=" << decoded_frames_
              << " sequence_gaps=" << sequence_gaps_ << " late_chunks=" << late_chunks_
              << " decode_errors=" << decode_errors_ << "\n";
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

    ++decoded_frames_;
    if (max_frames_ != 0 && decoded_frames_ > max_frames_) {
      return;
    }

    const rclcpp::Time stamp(
      metadata.header.stamp.sec, metadata.header.stamp.nanosec, RCL_ROS_TIME);
    writer_.write(output, output_topic_, stamp);
  }

  std::string topic_;
  std::string output_topic_;
  rosbag2_cpp::Writer & writer_;
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
};

struct Options
{
  std::string bag;
  std::string out;
  std::string storage_id;
  std::string out_storage_id{"mcap"};
  std::vector<std::string> topics;
  int decoder_threads{1};
  std::uint64_t max_frames{0};
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program << " --bag <path> --out <dir> [options]\n"
    << "\n"
    << "Reads rosbag2 bags (db3 or mcap) of shm_msgs/msg/CompressedVideo and\n"
    << "decodes each H.265 stream into sensor_msgs/msg/Image (bgr8), writing\n"
    << "the result to new rosbag2 bags. No ros2 bag play required.\n"
    << "\n"
    << "--bag may be a single bag (a .mcap file, or a db3/mcap directory) or a\n"
    << "directory that contains multiple bags; in the latter case every bag under\n"
    << "it is decoded in batch.\n"
    << "\n"
    << "Each output bag keeps the input bag's name and is written under --out.\n"
    << "Output topics are derived from the input: /camera/<cam>/h265 becomes\n"
    << "/camera/<cam>/image_raw.\n"
    << "\n"
    << "Options:\n"
    << "  --bag <path>            Input bag, or a directory of bags (required)\n"
    << "  --out <dir>             Output directory (required)\n"
    << "  --storage-id <id>       Input storage: 'mcap' or 'sqlite3'; empty = auto\n"
    << "  --out-storage-id <id>   Output storage: 'mcap' (default) or 'sqlite3'\n"
    << "  --topics <t1,t2,...>    Only decode these input topics (default: all)\n"
    << "  --decoder-threads <n>   FFmpeg threads per camera (default 1)\n"
    << "  --max-frames <n>        Max frames written per stream, 0 = unlimited\n"
    << "  -h, --help              Show this help\n";
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
    } else if (argument == "--out-storage-id") {
      options.out_storage_id = require_value(index);
    } else if (argument == "--topics") {
      split(require_value(index), ',', options.topics);
    } else if (argument == "--decoder-threads") {
      options.decoder_threads = std::stoi(require_value(index));
    } else if (argument == "--max-frames") {
      options.max_frames = std::stoull(require_value(index));
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
  return options;
}

int decode_bag(
  const Options & options,
  const BagEntry & bag,
  const std::filesystem::path & output_uri)
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage;
  storage.uri = bag.uri;
  storage.storage_id = bag.storage_id;
  reader.open(storage);

  std::vector<std::string> video_topics;
  for (const auto & topic : reader.get_all_topics_and_types()) {
    if (topic.type != kVideoType) {
      continue;
    }
    if (!options.topics.empty() &&
      std::find(options.topics.begin(), options.topics.end(), topic.name) == options.topics.end())
    {
      continue;
    }
    video_topics.push_back(topic.name);
  }
  if (video_topics.empty()) {
    std::cerr << bag.name << ": no " << kVideoType << " topics, skipped\n";
    return 1;
  }

  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions out_storage;
  out_storage.uri = output_uri.string();
  out_storage.storage_id = options.out_storage_id;
  writer.open(out_storage);

  std::map<std::string, std::unique_ptr<StreamDecoder>> decoders;
  for (const auto & topic : video_topics) {
    decoders.emplace(
      topic,
      std::make_unique<StreamDecoder>(
        topic, image_topic_for(topic), writer, options.max_frames, options.decoder_threads));
  }

  rosbag2_storage::StorageFilter filter;
  for (const auto & entry : decoders) {
    filter.topics.push_back(entry.first);
  }
  reader.set_filter(filter);

  rclcpp::Serialization<CompressedVideo> serialization;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    const auto decoder = decoders.find(message->topic_name);
    if (decoder == decoders.end()) {
      continue;
    }
    CompressedVideo video;
    rclcpp::SerializedMessage serialized(*message->serialized_data);
    serialization.deserialize_message(&serialized, &video);
    decoder->second->ingest(video);
  }

  for (const auto & entry : decoders) {
    entry.second->flush();
    entry.second->log_stats();
  }
  writer.close();
  std::cout << "wrote " << output_uri << "\n";
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

    const std::filesystem::path output_root(options.out);
    std::filesystem::create_directories(output_root);

    std::size_t succeeded = 0;
    std::size_t failed = 0;
    for (const auto & bag : bags) {
      const std::filesystem::path output_uri = output_root / bag.name;
      try {
        if (decode_bag(options, bag, output_uri) == 0) {
          ++succeeded;
        } else {
          ++failed;
        }
      } catch (const std::exception & error) {
        std::cerr << bag.name << ": " << error.what() << "\n";
        ++failed;
      }
    }
    std::cout << "decoded " << succeeded << " bag(s), " << failed << " failed\n";
    if (failed != 0) {
      return_code = 1;
    }
  } catch (const std::exception & error) {
    std::cerr << "h265_decoder_readfile: " << error.what() << "\n";
    return_code = 1;
  }
  rclcpp::shutdown();
  return return_code;
}
