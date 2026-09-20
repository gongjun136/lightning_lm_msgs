# 规划结果回传 (plan → dec): /GlobalPlanning 与 /LocalPlanning 的消息类型
# ★2026-08-20: 由 std_msgs/String(JSON) 升级为带 ROS Header 的消息 —
#   原 JSON header 的 request_id + timestamp 移入本消息字段。
std_msgs/Header header   # header.stamp = 结果发布时间 (原 JSON header.timestamp)
string request_id        # 原 JSON header.request_id
string data              # 原 JSON 剩余部分 ({"body":{"data":...}}) 的 JSON 文本
