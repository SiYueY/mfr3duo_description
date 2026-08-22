# MJCF 资源与来源

`mjcf/mfr3duo.xml` 是完整的默认 MuJoCo 机器人模型，`mjcf/scene.xml`
是包含地面、灯光和预览相机的查看场景。可使用 MuJoCo Python API 加载：

```python
import mujoco
mujoco.MjModel.from_xml_path("mjcf/scene.xml")
```

## 目录

`mjcf/` 顶层的组件 XML 与 `urdf/` 的模型分组同名：`franka_tmr`、
`franka_spine`、`franka_head`、`franka_fr3`、`franka_hand`、`imu`、
`nanoscan3`、`realsense_d435`、`realsense_d455`、`wrist_camera_mount` 和
`zed_mini`。对应 OBJ/STL 资源位于 `mjcf/meshes/<model>/{visual,collision}/`；
IMU 没有 Mesh。

`mfr3duo.xml` 固定了 Mobile FR3 Duo 的 MuJoCo 装配、动力学、执行器、
传感器和接触参数。它是 MuJoCo 资源，不作为 ROS `robot_description` 输入。

## 最终来源

| 模型 | 最终来源 | 固定版本与许可证 |
| --- | --- | --- |
| TMR、Spine、Duo mount、Head、FR3、Franka Hand、IMU | Franka Robotics [`franka_description`](https://github.com/frankarobotics/franka_description) | 2.8.1，commit `02afaae282d4a8e10d7d2f781b23b3515c303ce5`，Apache-2.0 |
| RealSense D435、D455 | Intel [`realsense-ros`](https://github.com/IntelRealSense/realsense-ros) | 4.58.3，commit `60c850958d651130fc2cc3d10efb37ff5be93da5`，Apache-2.0 |
| NanoScan3 | SICK [`sick_safetyscanners2`](https://github.com/SICKAG/sick_safetyscanners2) | 1.0.5，commit `c8d787f45d0679f11a75085553f39e7555c359b5`，Apache-2.0 |
| ZED Mini | Stereolabs [`zed-ros2-description`](https://github.com/stereolabs/zed-ros2-description) | 0.1.5，commit `449eef9566a49461cc37d6ac13230fe2886ff2be`，Apache-2.0 |
| D435 Wrist-Cam Mount | [Franka 3D Assets](https://franka.de/3d-assets) | RealSenseD435 Camera Mount 官方 STL/STEP，下载条款 |

## 转换复用

本包直接复用 `mobile_fr3_duo/models` 中既有的 MuJoCo OBJ/STL 转换结果，
以避免重新执行相同的格式转换。这只是工程转换输入：它不构成模型的最终来源、
版本权威或运行时依赖。每个 MJCF XML 和 Mesh 引用均已重定位到本包的
`mjcf/` 目录；运行时不需要 `mobile_fr3_duo`。

MJCF 使用转换后的 OBJ/STL，以适配 MuJoCo；这不改变 URDF 使用官方原始
DAE/STL 的资源策略。
