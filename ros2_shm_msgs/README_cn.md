# ros2_shm_msgs

English version: [README](./README.md)

## 概述

`shm_msgs` 提供用于共享内存（零拷贝）图像与 H.265 视频传输的定长 ROS 2 消息定义，
以及一组在线工具与离线 rosbag2 处理工具。定长消息是 loaned message API 的前提，
使进程间真正的零拷贝传输成为可能。

## 消息定义

所有消息由 `rosidl` 生成，定义位于 `msg/`。

| 消息                | 说明                                              |
|---------------------|---------------------------------------------------|
| `shm_msgs/String`   | 定长 char[256] 字符串                             |
| `shm_msgs/Header`   | 时间戳 + frame_id（使用 `shm_msgs/String`）       |
| `Image8k/512k/1m/2m/4m/6m/8m` | 定长未压缩图像（data `[N]`）             |
| `CompressedVideo`   | 一个 Annex-B H.265 数据分片 + 帧元信息            |
| `DynamicObject`     | 动态目标（使用 `geometry_msgs`）                  |
| `StaticObject`      | 静态目标（使用 `geometry_msgs`）                  |
| `Obstacle`          | 障碍物结果列表（动态 + 静态）                      |
| `Pallet`/`PalletData` | 载具检测结果 + 单项                              |
| `Slot`/`SlotData`   | 库位检测结果 + 单项                                |
| `DrivableArea`      | 可行驶区域（freespace、curb、occupancy）          |
| `LocalizationInfo`  | 定位消息（pose、attitude、speed、std）            |
| `ImageFrame`        | 相机帧（sensor id、data_ptr/image_data）          |
| `LidarFrame`        | 激光雷达帧（点云数组）                              |
| `CompressedImageFrame` | 编码图像（format + data）                       |
| `EulerAngle`/`SensorID`/`PointCloudData`/`Polygon`/`PolyLine`/`FreeSpace`/`Curb`/`Occupancy` | 支撑子消息 |

`include/shm_msgs/` 下的头文件提供围绕这些消息的辅助工具：
`opencv_conversions.hpp`（shm Image <-> cv::Mat）、`array_helper.hpp`
（shm String <-> std::string）、`image_encodings.hpp`、`fill_image.hpp` 与
`rgb_colors.hpp`。`lib/` 编译的 `shm_msgs_image` 库实现了 OpenCV 转换与 RGB 辅助函数。

## 在线工具

ROS 2 运行时可执行程序，完成 shm 消息的桥接 / 编码 / 解码。

| 工具                              | 说明                                                           |
|-----------------------------------|----------------------------------------------------------------|
| `shm_image{8k,512k,1m,2m,4m,6m,8m}_bridge` | 桥接 `shm_msgs::msg::Image*` -> `sensor_msgs::msg::Image`        |
| `shm_image6m_bridge_6ch`          | 6 路桥接版本                                                   |
| `shm_image6m_bridge_6ch_compressed` | 6 路 `Image6m` -> `sensor_msgs::CompressedImage`（OpenCV，不依赖 cv_bridge） |
| `shm_image6m_subscriber_6ch`      | 6 路订阅器参考示例                                             |
| `shm_image6m_consumer_6ch`        | 6 路消费者参考示例（遥操推流模块使用）                          |
| `shm_image6m_h265_encoder_6ch`    | 6 路硬件 H.265 编码器（`Image6m` -> `CompressedVideo`），运行时 dlopen `libvideoenc.so` |
| `shm_image6m_h265_encoder_6ch_allfps` | 全帧率变体：按原始采集时间戳编码每一帧                     |
| `h265_decoder_6ch`                | 6 路在线 H.265 解码器（`CompressedVideo` -> `sensor_msgs/Image` bgr8，FFmpeg） |

桥接 / 订阅器 / 解码器工具对应的 launch 文件位于 `launch/`。

## 离线工具

以下工具直接读取 rosbag2 包（db3/mcap），通过 `rosbag2_cpp` 访问，不依赖 `ros2 bag play`。

### h265_decoder_readfile

读取包含 `shm_msgs/msg/CompressedVideo`（H.265）的包，解码为 `sensor_msgs/msg/Image`（bgr8）写入新包。

```sh
# mcap 输入 -> db3 输出
ros2 run shm_msgs h265_decoder_readfile \
  --bag /path/to/input.mcap --out /path/to/output_bag --out-storage-id sqlite3

# db3 输入 -> mcap 输出
ros2 run shm_msgs h265_decoder_readfile \
  --bag /path/to/input_bag_dir --out /path/to/out
```

### bag_to_obstacle_input

读取 `shm_msgs/msg/Image6m`（4 路鱼眼）+ `sensor_msgs/msg/PointCloud2`（LiDAR），按时间戳落盘为图片与 PCD。结果写入 `<out>/<bag_name>_clip/`。

```sh
# 全量按时间戳落盘（flat 模式）
ros2 run shm_msgs bag_to_obstacle_input --bag <bag> --out <dir>

# 只留对齐帧：每个 frame_<lidar_ts>/ 含 4 路 png + 1 个 pcd
ros2 run shm_msgs bag_to_obstacle_input --bag <bag> --out <dir> --sync --sync-tolerance-ms 50
```

### mergelidar_h265_clip

融合工具：读取含 H.265 `CompressedVideo`（4 路鱼眼）+ `PointCloud2`（LiDAR）的包，先解码 H.265 为 BGR，再按 lidar 时间戳匹配 4 路相机帧并切片落盘。适用于 `/camera/fisheye_*/h265` + `/LidarDataInv` 这类组合包。结果写入 `<out>/<bag_name>_clip/`。

```sh
# 默认 sync 模式：每个 lidar 帧匹配容差内最近的 4 路相机帧
ros2 run shm_msgs mergelidar_h265_clip \
  --bag /data1/sunyuan/test/rosbag2_2026_09_02-08_39_21_deskew \
  --out /data1/sunyuan/test/clip_output

# 调整匹配容差与解码线程
ros2 run shm_msgs mergelidar_h265_clip \
  --bag <bag_or_dir> --out <dir> \
  --sync-tolerance-ms 30 --decoder-threads 4

# flat 模式：不解码后匹配，直接平铺所有图片和点云
ros2 run shm_msgs mergelidar_h265_clip --bag <bag> --out <dir> --flat

# 批量处理目录下所有包
ros2 run shm_msgs mergelidar_h265_clip --bag /path/to/bags_dir --out /path/to/out
```

常用选项：

| 选项 | 默认 | 说明 |
|------|------|------|
| `--bag` | 必填 | 输入包或含多个包的目录 |
| `--out` | 必填 | 输出目录，结果落在 `<out>/<bag>_clip/` |
| `--lidar-topic` | `/LidarDataInv` | LiDAR 点云话题 |
| `--camera-topics` | 4 路 h265 | 逗号分隔覆盖相机话题 |
| `--sync-tolerance-ms` | 50 | 时间戳匹配容差（毫秒） |
| `--decoder-threads` | 1 | 每路 FFmpeg 解码线程数 |
| `--image-ext` | png | 输出图片格式（png/jpg） |
| `--max-frames` | 0 | 每流最大帧数，0=不限 |
| `--flat` | - | 不匹配，平铺输出所有帧 |
| `--storage-id` | 自动 | 输入存储格式（mcap/sqlite3） |

### extract_h265_from_bag.py

从 rosbag2 包中提取 Annex-B H.265 码流及每路的时间戳 CSV。

```sh
ros2 run shm_msgs extract_h265_from_bag.py <bag_dir> <output_dir>
```