# vlm_pipeline (message-only subset)

**这不是 VLM 团队的完整包。** 真包在从 Orin
`192.168.10.101:~/vlm_loader/vlm_pipeline`,含 pipeline 节点本体,依赖
opencv / ffmpeg / curl / shm_msgs,只能在车上(ARM)编。

本仓只抽出 `state_machine` 编译和跑单测所需的 9 个 `.msg`,从真包**逐字节拷贝**,
因此 rosidl 生成的类型哈希与真包一致,不会造成 DDS 类型不匹配:

    VehiclePose  TaskItem  TaskQueueRequest  TaskQueueResponse
    PhaseTransition  VehicleState  HmiDisplay  VlmError  WholeCycleSummary

未收录(依赖 shm_msgs/sensor_msgs,state_machine 不引用):
`Alarm` `VlmInferenceRequest` `VlmInferenceResult`。

## 用途与禁忌

- 用途:在没有 VLM 全套依赖的机器(如云服务器 wanghui、开发本机)上编译
  `state_machine` 的 `vlm` 分支、跑 gtest。
- **禁忌:不要拿这个包去车上部署。** 车上用 VLM 团队的真包。

## VLM 团队改了 msg 怎么办

从真包重新拷贝对应 `.msg` 覆盖,提交即可:

    scp nvidia@192.168.10.101:'~/vlm_loader/vlm_pipeline/msg/*.msg' msg/

然后 `git diff` 就能看出接口变更;若新增字段/消息,记得同步更新
`CMakeLists.txt` 里的 `rosidl_generate_interfaces` 列表。

来源快照:2026-08-24 从 `~/vlm_loader/vlm_pipeline` 取。
