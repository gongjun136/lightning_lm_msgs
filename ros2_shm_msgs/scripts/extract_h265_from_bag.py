#!/usr/bin/env python3

import argparse
import csv
import re
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


VIDEO_TYPE = "shm_msgs/msg/CompressedVideo"


def safe_name(topic: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", topic.strip("/"))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract Annex-B H.265 streams and timestamp CSV files from a rosbag2 bag."
    )
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("output", help="output directory")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = {
        topic: get_message(message_type)
        for topic, message_type in topic_types.items()
        if message_type == VIDEO_TYPE
    }
    if not selected:
        raise RuntimeError(f"No {VIDEO_TYPE} topics found in {args.bag}")

    streams = {}
    csv_files = {}
    writers = {}
    try:
        for topic in selected:
            name = safe_name(topic)
            streams[topic] = (output_dir / f"{name}.h265").open("wb")
            csv_files[topic] = (output_dir / f"{name}.csv").open(
                "w", newline="", encoding="utf-8"
            )
            writers[topic] = csv.writer(csv_files[topic])
            writers[topic].writerow(
                ["sequence", "stamp_sec", "stamp_nanosec", "key_frame", "bytes"]
            )

        while reader.has_next():
            topic, serialized, _ = reader.read_next()
            message_class = selected.get(topic)
            if message_class is None:
                continue
            message = deserialize_message(serialized, message_class)
            streams[topic].write(bytes(message.data))
            writers[topic].writerow(
                [
                    message.frame_sequence,
                    message.header.stamp.sec,
                    message.header.stamp.nanosec,
                    int(message.key_frame),
                    len(message.data),
                ]
            )
    finally:
        for stream in streams.values():
            stream.close()
        for csv_file in csv_files.values():
            csv_file.close()

    print(f"Extracted {len(selected)} H.265 stream(s) to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
