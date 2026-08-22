# URDF 资源与来源

`urdf/mfr3duo.urdf.xacro` 是可参数化公开入口，机器人名固定为 `mfr3duo`，唯一公开装配参数为 `connected_to`。`urdf/mfr3duo.urdf` 是已提交的完整默认模型，固定使用 `connected_to=base`，可由 `robot_state_publisher` 或 RViz 直接加载而无需 Xacro。所有 Mesh URI 都指向本包的 `urdf/meshes/`。

## 布局

`urdf/` 顶层只包含入口、总装配及平铺的子模型 Xacro；`meshes/` 是唯一子目录。每个 `meshes/<model>/` 分组都有同名 `<model>.xacro`：Franka 内部辅助宏和冻结参数已合并进所属模型，因此展开时不读取 YAML 文件。

`mfr3duo.urdf` 由默认 Xacro 展开生成，不是独立维护的第二套模型；任何默认装配或 Xacro 变更都必须重新生成并提交该文件。

## 一键可视化

在构建并 source 工作空间后运行：

```bash
ros2 run mfr3duo_description visualize_urdf
```

该脚本加载安装前缀中的静态 `mfr3duo.urdf`，启动
`robot_state_publisher`、`joint_state_publisher_gui` 和 `rviz2`。默认 RViz
配置使用 `base_link` 作为 Fixed Frame，并从 `/robot_description` 显示模型；
关闭 RViz 后脚本会停止两个发布器并删除临时参数文件。

| 子模型 | Xacro | 冻结来源 |
| --- | --- | --- |
| 移动底盘 | `franka_tmr.xacro` | `franka_description` 2.8.1，commit `02afaae282d4a8e10d7d2f781b23b3515c303ce5`，`robots/tmrv0_2/`，Apache-2.0 |
| Spine | `franka_spine.xacro` | 同上 |
| Duo mount 与 Head | `franka_head.xacro` | 同上；`accessories/fr3_duo_mount_v0_3/`、`accessories/franka_head_v0_2/`，共同使用 `meshes/franka_head/` |
| 双 FR3v2.1 | `franka_fr3.xacro` | 同上；`robots/fr3v2_1/`，含冻结 joint limits、kinematics、inertials、dynamics、accelerometers |
| Franka Hand | `franka_hand.xacro` | 同上；`end_effectors/franka_hand/` |
| IMU | `imu.xacro` | 项目定义；无 Mesh |
| D435、D455 | `realsense_d435.xacro`、`realsense_d455.xacro` | Intel `realsense-ros` 4.58.3，commit `60c850958d651130fc2cc3d10efb37ff5be93da5`，`realsense2_description/meshes/d435.dae`、`d455.stl`，Apache-2.0 |
| NanoScan3 | `nanoscan3.xacro` | SICK `sick_safetyscanners2` 1.0.5，commit `c8d787f45d0679f11a75085553f39e7555c359b5`，`description/meshes/NANS3.dae`，Apache-2.0 |
| ZED Mini | `zed_mini.xacro` | Stereolabs `zed-ros2-description` 0.1.5，commit `449eef9566a49461cc37d6ac13230fe2886ff2be`，`meshes/zedm.stl`，Apache-2.0 |
| D435 Wrist-Cam Mount | `wrist_camera_mount.xacro` | Franka [3D Assets](https://franka.de/3d-assets)，`RealSenseD435 Camera Mount.stl`，官方下载条款 |

Franka 主体的 visual/collision Mesh 直接来自上述 Franka 基线；D435、D455、NanoScan3、ZED Mini 和腕部 mount 是相应厂商的原始 DAE/STL。`meshes/` 不包含 `.obj` 或来自其他项目的资源。

## Franka 参数冻结

以下上游 YAML 已内联，原文不保留在本包中：

| 范围 | 上游文件数 | 用途 |
| --- | ---: | --- |
| FR3v2.1 | 5 | joint limits、kinematics、inertials、dynamics、accelerometers |
| Spine、Duo mount、Head | 6 | kinematics 与 inertials |
| Franka Hand | 1 | inertials |

参数内容位于 `urdf/franka_fr3.xacro`、`franka_spine.xacro`、`franka_head.xacro` 与 `franka_hand.xacro`；其来源固定为上表所列 `franka_description` 2.8.1 commit。这使模型保持自包含，同时保留官方版本锚点。

左右 D435 与 wrist mount 的相对变换仍是暂定 figure-fit 值，须由 CAD 或实测标定替换；传感器和 mount 不定义近似 collision。
