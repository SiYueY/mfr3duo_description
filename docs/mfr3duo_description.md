# mfr3duo_description 设计文档

## 1. 项目定位与设计目标

### 1.1 项目定位

`mfr3duo_description` 定义为：

> Mobile FR3 Duo 的统一机器人描述仓库。

项目同时正式维护：

```text
URDF / Xacro
SRDF
MJCF
```

其中：

```text
URDF / Xacro / SRDF
```

主要服务：

* ROS 2；
* `robot_state_publisher`；
* RViz；
* MoveIt；
* TF；
* Nav2 所依赖的机器人 frame；
* 后续 ros2_control 模型组合。

而：

```text
MJCF
```

主要服务：

* MuJoCo；
* 机器人动力学；
* actuator；
* contact；
* friction；
* sensor；
* camera；
* 抓取；
* 移动底盘物理仿真。

项目本身只负责：

> 机器人模型描述。

不承担上层控制、规划、导航和仿真运行时逻辑。

### 1.2 V1 目标

V1 阶段需要完成：

1. 提供完整的 Mobile FR3 Duo URDF/Xacro；
2. 提供完整的 Mobile FR3 Duo MJCF；
3. 提供基础 SRDF；
4. 固定 Franka 官方机械模型基线；
5. 明确项目支持的设备集合；
6. 保证 URDF 与 MJCF 描述同一台机器人；
7. URDF 与 MJCF 分别维护自己的资源；
8. 所有关键资源来源可追溯；
9. 自动检测 URDF/MJCF 模型漂移；
10. URDF 可以直接用于标准 ROS 2 工具；
11. MJCF 可以脱离 ROS 2 被原生 MuJoCo 加载；
12. 为 MoveIt、Nav2、ros2_control 和 MuJoCo Runtime 提供稳定的底层模型基础。

### 1.3 非目标

V1 不负责：

* MoveIt planning pipeline；
* MoveIt `kinematics.yaml`；
* OMPL/Pilz/CHOMP 配置；
* MoveIt controller 配置；
* Nav2 planner/controller；
* localization；
* SLAM；
* costmap 配置；
* ros2_control controller manager；
* hardware interface；
* 真机通信；
* MuJoCo Runtime；
* Viewer；
* Scheduler；
* ROS 2 与 MuJoCo 通信；
* 世界场景管理；
* 业务层任务逻辑。

这些功能应由上层项目承担。

### 1.4 上层依赖关系

整体依赖关系：

```text
                  franka_description 2.8.1
                           │
                           ▼
                  mfr3duo_description
                           │
                ┌──────────┴──────────┐
                ▼                     ▼
            URDF / SRDF              MJCF
                │                     │
                ▼                     ▼
             ROS 2                  MuJoCo
                │                     │
         ┌──────┼──────┐              │
         ▼      ▼      ▼              ▼
      MoveIt   RViz   Nav2      MuJoCo Runtime
```

`mfr3duo_description` 是底层公共模型依赖。

上层项目可以依赖它，但它不应反向依赖：

* MoveIt；
* Nav2；
* ros2_control；
* MuJoCo Runtime。

---

## 2. 官方基线与设备范围

### 2.1 Franka 官方基线

V1 固定使用：

```text
franka_description = 2.8.1
```

仓库：

```text
https://github.com/frankarobotics/franka_description
```

官方机器人类型固定为：

```text
mobile_fr3_duo_v0_2
```

项目正式构建、Golden URDF 生成、模型验证以及官方资源校验均必须以：

```text
franka_description 2.8.1
```

为基线。

禁止使用：

```text
main
master
latest
latest tag
```

等 floating dependency。

即使 upstream 后续出现更新版本，V1 仍保持：

```text
2.8.1
```

不自动升级。

版本升级必须作为独立设计变更执行。

### 2.2 Upstream 与项目命名

Franka upstream identifier：

```text
mobile_fr3_duo_v0_2
```

项目机器人公开名称：

```text
mfr3duo
```

应明确区分：

```text
upstream robot type:
mobile_fr3_duo_v0_2

project robot name:
mfr3duo
```

官方名称用于：

* upstream model lookup；
* source manifest；
* compatibility validation。

项目名称用于：

* URDF；
* MJCF；
* SRDF；
* ROS package；
* 上层软件接口。

### 2.3 Canonical Device Set

设备命名直接继承现有：

```text
SiYueY/mobile_fr3_duo/models
```

V1 固定设备集合为：

```text
franka_fr3
franka_hand
franka_head
franka_spine
franka_tmr
imu
nanoscan3
realsense_d435
realsense_d455
wrist_camera_mount
zed_mini
```

该列表定义为：

> Canonical Device Set。

这些名称作为：

* 资源目录名称；
* Manifest device identifier；
* Asset source tracking identifier；
* Validation identifier。

后续不应再次引入同义目录，例如：

```text
fr3/
arm/
base/
tmr/
camera/
```

来表示相同设备。

### 2.4 设备分类

V1 将设备划分为三类。

Franka 主体机械设备：

```text
franka_fr3
franka_hand
franka_head
franka_spine
franka_tmr
```

传感器：

```text
imu
nanoscan3
realsense_d435
realsense_d455
zed_mini
```

机械安装组件：

```text
wrist_camera_mount
```

`wrist_camera_mount` 必须作为独立设备保留，而不是并入：

```text
franka_fr3
```

或某个具体 camera model。

因为它本身是独立机械部件，具有：

* geometry；
* transform；
* collision；
* source；
* version。

### 2.5 左右相同设备的资源复用

左右 FR3 不建立：

```text
left_franka_fr3/
right_franka_fr3/
```

只保留：

```text
franka_fr3/
```

左右机械臂通过：

* prefix；
* mounting transform；
* joint namespace；

进行区分。

同样，左右 Franka Hand 只共享：

```text
franka_hand/
```

资源目录。

---

## 3. 总体模型架构

### 3.1 双格式描述模型

`mfr3duo_description` 同时维护：

```text
URDF / Xacro
MJCF
```

但二者在文件实现和资源管理层面完全分离。

不建立：

```text
shared/
shared_assets/
common_meshes/
```

作为两个 backend 的公共 runtime 资源目录。

核心原则：

```text
URDF implementation != MJCF implementation
```

但：

```text
URDF mechanical semantics == MJCF mechanical semantics
```

### 3.2 URDF 的职责

URDF/Xacro 主要表达：

* kinematic tree；
* links；
* joints；
* visual geometry；
* collision geometry；
* inertial；
* joint limits；
* mounting frames；
* sensor frames；
* ROS frame hierarchy。

主要消费者：

* robot_state_publisher；
* RViz；
* MoveIt；
* TF；
* Nav2；
* ros2_control 的组合描述。

### 3.3 MJCF 的职责

MJCF 除了表达机器人结构，还需要表达：

* body；
* geom；
* actuator；
* contact；
* friction；
* solver 参数；
* sensor；
* camera；
* site；
* keyframe；
* equality；
* tendon；
* physical collision representation。

主要消费者：

* 原生 MuJoCo；
* MuJoCo Runtime；
* 后续 ROS/MuJoCo 集成层。

### 3.4 必须保持一致的机械语义

URDF 与 MJCF 必须保持一致：

* robot topology；
* link/body hierarchy；
* joint hierarchy；
* joint name；
* joint type；
* joint parent/child；
* joint axis；
* joint origin；
* joint limit；
* mass；
* center of mass；
* inertia；
* arm mounting transform；
* spine transform；
* head transform；
* sensor mounting transform；
* wheel radius；
* wheel position；
* wheel axis；
* spine travel；
* 关键机械尺寸。

一致性要求针对：

> 机器人机械语义。

而不是直接比较 XML 结构。

### 3.5 允许存在的 Backend-specific 差异

允许不同：

| URDF / ROS                | MJCF                      |
| ------------------------- | ------------------------- |
| `<link>`                  | `<body>`                  |
| `<collision>`             | `<geom>`                  |
| ROS material              | MJCF material             |
| `package://`              | filesystem relative path  |
| transmission              | actuator                  |
| ROS frame                 | body/site/camera          |
| MoveIt collision geometry | physical contact geometry |
| 无 solver 参数               | solref/solimp             |
| 无 keyframe                | keyframe                  |
| 无 MuJoCo friction         | friction                  |
| 无原生 MJCF sensor           | sensor                    |
| 无 tendon                  | tendon                    |
| 无 equality                | equality                  |

项目不应为了形式统一而削弱某个 backend 的建模能力。

---

## 4. 项目目录结构

V1 推荐目录：

```text
mfr3duo_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── LICENSE
│
├── urdf/
│   ├── mfr3duo.urdf.xacro
│   ├── mfr3duo.xacro
│   │
│   ├── sensors/
│   │   ├── imu.xacro
│   │   ├── nanoscan3.xacro
│   │   ├── realsense_d435.xacro
│   │   ├── realsense_d455.xacro
│   │   └── zed_mini.xacro
│   │
│   ├── mounts/
│   │   └── wrist_camera_mount.xacro
│   │
│   ├── common/
│   │   └── materials.xacro
│   │
│   └── meshes/
│       └── <device>/
│           ├── visual/
│           └── collision/
│
├── srdf/
│   └── mfr3duo.srdf.xacro
│
├── mjcf/
│   ├── mfr3duo.xml
│   ├── scene.xml
│   │
│   ├── model/
│   │   ├── defaults.xml
│   │   ├── actuators.xml
│   │   ├── sensors.xml
│   │   └── contacts.xml
│   │
│   └── assets/
│       └── <device>/
│           ├── visual/
│           ├── collision/
│           └── textures/
│
├── source/
│   ├── model_manifest.yaml
│   ├── asset_manifest.yaml
│   ├── parameters.yaml
│   └── generated/
│       └── official_mfr3duo.urdf
│
├── tools/
│   ├── prepare_sources.py
│   ├── generate_urdf.py
│   ├── generate_mjcf.py
│   └── validate_models.py
│
├── launch/
│   └── display.launch.py
│
├── rviz/
│   └── display.rviz
│
└── test/
    ├── test_urdf.py
    ├── test_srdf.py
    ├── test_mjcf.py
    ├── test_consistency.py
    └── test_official_compatibility.py
```

### 4.1 顶层目录职责

`urdf/`

> ROS-facing robot model。

`srdf/`

> Robot semantic description。

`mjcf/`

> MuJoCo-facing robot model。

`source/`

> Build-time source tracking 和官方模型基准。

`tools/`

> Source preparation、model generation、validation。

`launch/`

> Description 展示入口。

`rviz/`

> 基础模型展示配置。

`test/`

> URDF、SRDF、MJCF 和 cross-format validation。

### 4.2 资源目录采用“设备优先”

URDF：

```text
urdf/meshes/<device>/
```

MJCF：

```text
mjcf/assets/<device>/
```

设备内部再按资源类型划分。

URDF：

```text
<device>/
├── visual/
└── collision/
```

MJCF：

```text
<device>/
├── visual/
├── collision/
└── textures/
```

不采用：

```text
visual/<device>/
collision/<device>/
```

这样的“资源类型优先”结构。

设备优先更适合：

* 设备版本升级；
* Asset 来源追踪；
* Mesh 转换；
* License 检查；
* Collision 优化；
* Hash 校验。

### 4.3 空目录处理

目录规范不意味着必须创建所有子目录。

如果某设备没有 texture：

```text
<device>/
├── visual/
└── collision/
```

即可。

如果只有 visual：

```text
<device>/
└── visual/
```

即可。

禁止为了形式对称创建无意义空目录。

---

## 5. URDF / SRDF 设计

### 5.1 URDF 公开入口

正式 ROS Robot Description 入口：

```text
urdf/mfr3duo.urdf.xacro
```

上层项目应统一通过该文件生成：

```text
robot_description
```

该文件作为：

> Public URDF Entry Point。

职责：

* 定义 `<robot>`；
* 定义公开参数；
* include `mfr3duo.xacro`；
* 实例化完整机器人。

它应保持非常薄，不直接维护大量：

```xml
<link/>
<joint/>
<visual/>
<collision/>
```

### 5.2 机器人组合层

主要组合文件：

```text
urdf/mfr3duo.xacro
```

职责：

```text
franka_description 2.8.1
        +
project extensions
        +
sensors
        +
mounts
```

Franka 主体机械结构优先复用：

```text
mobile_fr3_duo_v0_2
```

不重新手写：

```text
franka_tmr
franka_spine
franka_head
franka_fr3
franka_hand
```

的完整官方机械描述。

只有在以下情况才允许项目级覆盖：

* upstream 明确缺失；
* upstream 存在已确认问题；
* 项目真实机械结构与官方不同；
* 项目新增设备。

### 5.3 Sensor 组织

传感器 Xacro：

```text
urdf/sensors/
├── imu.xacro
├── nanoscan3.xacro
├── realsense_d435.xacro
├── realsense_d455.xacro
└── zed_mini.xacro
```

每个 sensor 提供独立 macro。

例如概念接口：

```xml
<xacro:macro
  name="mfr3duo_realsense_d455"
  params="
    parent
    prefix
    xyz:='0 0 0'
    rpy:='0 0 0'
  ">
```

传感器模型只描述：

* sensor body；
* sensor frame；
* optical frame；
* visual；
* collision；
* 必要固定 joint。

### 5.4 Mount 与 Sensor 分离

机械安装件：

```text
urdf/mounts/wrist_camera_mount.xacro
```

与具体相机独立。

关系：

```text
FR3
 │
 ▼
wrist_camera_mount
 │
 ├── realsense_d435
 ├── realsense_d455
 └── zed_mini
```

不应将某个具体 camera 直接固化成机械安装结构的一部分。

这样未来设备替换不会改变主体机械定义。

### 5.5 Camera Frame

Camera 建议明确区分：

```text
camera_mount
    │
    ▼
camera_link
    │
    ├── color_frame
    │     └── color_optical_frame
    │
    └── depth_frame
          └── depth_optical_frame
```

ROS optical frame 遵循常见约定：

```text
+x → image right
+y → image down
+z → optical forward
```

### 5.6 URDF Mesh 管理

项目自有或派生 URDF Mesh 位于：

```text
urdf/meshes/<device>/
```

例如：

```text
urdf/meshes/realsense_d455/
├── visual/
└── collision/
```

如果 `franka_description 2.8.1` 中的官方 Mesh 可以稳定直接引用，则优先：

```text
package://franka_description/...
```

不应为了目录完整而复制全部 Franka 官方 Mesh。

只有以下资源才进入本仓库：

* 项目自有 Mesh；
* 修改后的 Mesh；
* 格式转换后的 Mesh；
* 项目专用 Collision；
* 为独立发布而需要固化的派生资源。

### 5.7 SRDF

正式 SRDF：

```text
srdf/mfr3duo.srdf.xacro
```

SRDF 属于：

> Robot Semantic Description。

不属于：

> MoveIt Planning Configuration。

V1 至少支持：

```text
left_arm
right_arm

left_hand
right_hand

left_arm_hand
right_arm_hand

dual_arm
```

根据需要可增加：

```text
upper_body
```

第一阶段暂不建议定义：

```text
whole_body
```

避免过早引入：

* mobile base planning；
* Nav2/MoveIt ownership；
* whole-body IK；
* trajectory synchronization；
* controller arbitration。

### 5.8 SRDF 与官方模型关系

应优先复用 `franka_description 2.8.1` 已有的 Mobile FR3 Duo SRDF 语义信息。

项目仅增加必要扩展。

不建议重新维护一整套与官方重复的：

```text
disable_collisions
arm group
hand group
```

数据。

以下内容不进入 `mfr3duo_description`：

```text
kinematics.yaml
joint_limits.yaml
ompl_planning.yaml
pilz_cartesian_limits.yaml
moveit_controllers.yaml
move_group launch
```

它们属于未来的：

```text
mfr3duo_moveit_config
```

---

## 6. MJCF 设计

### 6.1 正式模型入口

正式机器人 MJCF：

```text
mjcf/mfr3duo.xml
```

它只定义机器人本身。

包括：

* body；
* joint；
* geom；
* actuator；
* sensor；
* site；
* camera；
* contact；
* defaults；
* keyframe；
* equality/tendon（若需要）。

不负责大型 world。

### 6.2 最小场景入口

最小测试场景：

```text
mjcf/scene.xml
```

负责：

```text
mfr3duo
+
floor
+
light
+
minimal environment
```

必须满足：

```bash
simulate mjcf/scene.xml
```

能够直接运行。

复杂房间、导航 world、benchmark world 不属于本 description 仓库的基础 robot model。

### 6.3 MJCF 模型组织

MuJoCo 专属模型内容：

```text
mjcf/model/
├── defaults.xml
├── actuators.xml
├── sensors.xml
└── contacts.xml
```

`defaults.xml`

负责：

* visual class；
* collision class；
* joint defaults；
* geom defaults；
* actuator defaults。

`actuators.xml`

负责：

* left FR3；
* right FR3；
* Franka Hand；
* Spine；
* base wheels。

这些参数属于 MJCF 专属内容，例如：

```text
gear
ctrlrange
forcerange
kp
kv
```

不应为了跨格式统一而强行进入共享参数层。

`sensors.xml`

负责 MuJoCo 原生 sensor，例如：

* jointpos；
* jointvel；
* accelerometer；
* gyro；
* force；
* torque；
* framepos；
* framequat；
* touch；
* range；
* camera-related sensor。

其中：

```text
sensor mounting transform
```

需要与 URDF 一致。

但：

```text
MuJoCo sensor implementation
```

属于 MJCF 专属。

`contacts.xml`

负责：

* contact exclude；
* explicit pair；
* self-contact；
* wheel-ground contact；
* grasping contact；
* MuJoCo collision policy。

不要求与 SRDF `disable_collisions` 完全相同，因为二者目标不同：

```text
SRDF → planning collision
MJCF → physical contact
```

### 6.4 MJCF Assets

所有正式 MuJoCo 资源必须位于：

```text
mjcf/assets/<device>/
```

例如：

```text
mjcf/assets/franka_fr3/
├── visual/
├── collision/
└── textures/
```

MJCF 必须保持 self-contained。

禁止依赖：

```text
package://
$(find ...)
xacro
ROS parameter
```

即使整个项目是 ROS 2 package，MJCF 本身仍必须是 ROS-Free。

### 6.5 URDF Mesh 与 MJCF Asset 的关系

两者物理目录完全独立：

```text
URDF:
urdf/meshes/

MJCF:
mjcf/assets/
```

不建立公共 runtime mesh 目录。

真正需要保持的是：

> Same source。

而不是：

> Same physical file。

例如：

```text
franka_description 2.8.1
        │
        ├── URDF → DAE
        └── MJCF → converted OBJ
```

这是正常且允许的。

### 6.6 Visual Geometry

Visual Geometry 原则：

```text
same upstream source
```

如果某个格式不能被两个 backend 同时稳定使用，可以生成不同表示。

例如：

```text
official link1.dae
        │
        ├── URDF → link1.dae
        └── MJCF → link1.obj
```

### 6.7 Collision Geometry

Collision Geometry 明确允许 backend-specific。

URDF collision 主要用于：

* MoveIt；
* planning scene；
* self collision；
* collision checking。

MJCF collision 主要用于：

* physics；
* friction；
* grasping；
* contact；
* solver。

因此允许：

```text
URDF collision mesh != MJCF collision geometry
```

例如轮子：

URDF 可以使用简化 mesh 或 cylinder。

MJCF 应优先使用经过验证的物理接触 geom，而不是复杂 visual mesh。

真正需要一致的是：

```text
wheel radius
wheel position
wheel axis
```

而不是 Mesh 文件本身。

---

## 7. Source、生成与资源追踪

### 7.1 Source Layer 定位

```text
source/
```

属于：

> Build-time Source Layer。

不是 runtime API。

负责：

* upstream version；
* official model；
* source manifest；
* asset source；
* asset conversion；
* hash；
* Golden URDF；
* shared project parameters。

### 7.2 `model_manifest.yaml`

V1 必须明确写死：

```yaml
robot:
  name: mfr3duo

upstream:
  franka_description:
    repository: https://github.com/frankarobotics/franka_description
    version: 2.8.1

model:
  robot_type: mobile_fr3_duo_v0_2

devices:
  - franka_fr3
  - franka_hand
  - franka_head
  - franka_spine
  - franka_tmr
  - imu
  - nanoscan3
  - realsense_d435
  - realsense_d455
  - wrist_camera_mount
  - zed_mini
```

该文件是：

* upstream baseline；
* Canonical Device Set；

的主要机器可读来源。

### 7.3 `asset_manifest.yaml`

Asset 按设备组织。

例如：

```yaml
devices:

  franka_fr3:
    source:
      repository: https://github.com/frankarobotics/franka_description
      version: 2.8.1

    urdf:
      mode: external

    mjcf:
      path: mjcf/assets/franka_fr3

  realsense_d455:
    source:
      repository: <repository>
      version: <fixed-version>

    urdf:
      path: urdf/meshes/realsense_d455

    mjcf:
      path: mjcf/assets/realsense_d455
```

针对具体资源还应记录：

* original path；
* generated path；
* conversion；
* hash；
* license/source information。

### 7.4 `parameters.yaml`

只保存真正需要 URDF 和 MJCF 共同使用的项目级机械参数。

例如：

```yaml
devices:
  realsense_d455:
    mount:
      parent: head_camera_mount
      xyz: [...]
      rpy: [...]

  nanoscan3:
    mount:
      parent: <mount-frame>
      xyz: [...]
      rpy: [...]
```

适合保存：

* 项目新增 Sensor Mount；
* 项目新增机械尺寸；
* 无官方来源但两个 backend 都需要的参数。

不得保存：

```text
MuJoCo solref
MuJoCo solimp
MuJoCo friction
actuator kp
actuator kv
MoveIt scaling
controller PID
```

共享参数层只统一真正共享的数据。

### 7.5 Golden Reference

固定生成：

```text
source/generated/official_mfr3duo.urdf
```

它必须严格来自：

```text
franka_description 2.8.1
+
mobile_fr3_duo_v0_2
```

用途：

* 官方机械基准；
* upstream compatibility；
* Canonical IR 输入；
* MJCF generation；
* URDF/MJCF consistency validation。

它不是正式 ROS runtime URDF。

正式 ROS 模型仍是：

```text
urdf/mfr3duo.urdf.xacro
```

### 7.6 Canonical IR

允许保留现有 Canonical IR。

关系：

```text
official/project URDF
         │
         ▼
     Canonical IR
         │
      ┌──┴──────────┐
      ▼             ▼
MJCF Builder   consistency
```

Canonical IR 是内部实现细节。

不作为第三种公开机器人模型格式。

V1 不建议：

```text
Canonical IR
 ├── generate URDF
 └── generate MJCF
```

避免逐步演化成自定义 Robot Description Language。

### 7.7 Tools

统一使用：

```text
tools/
```

而不同时存在 `scripts/` 与 `tools/`。

`prepare_sources.py`

负责：

1. 验证 `franka_description == 2.8.1`；
2. 验证 `mobile_fr3_duo_v0_2`；
3. 验证 upstream 必需文件；
4. 生成 Golden URDF；
5. 准备 MJCF Asset；
6. 转换必要 Mesh；
7. 计算 hash；
8. 验证 Manifest。

`generate_urdf.py`

负责：

```text
mfr3duo.urdf.xacro
      ↓
expanded URDF
```

主要用于：

* CI；
* `check_urdf`；
* debug；
* consistency validation。

`generate_mjcf.py`

负责：

```text
Golden/project model
       ↓
Canonical IR
       ↓
MJCF Builder
       ↓
mjcf/
```

现有 `mobile_fr3_duo` 中成熟的 Builder 应优先迁移，不重新实现。

`validate_models.py`

提供统一开发者入口，例如：

```bash
python3 tools/validate_models.py
```

内部依次执行：

```text
source validation
URDF validation
SRDF validation
MJCF validation
cross-format consistency
official compatibility
```

---

## 8. 模型一致性与测试

### 8.1 测试体系

至少提供：

```text
test_urdf.py
test_srdf.py
test_mjcf.py
test_consistency.py
test_official_compatibility.py
```

测试不是辅助能力，而是本项目的核心组成部分。

### 8.2 URDF 测试

验证：

* Xacro expansion；
* XML；
* URDF parser；
* link uniqueness；
* joint uniqueness；
* parent/child；
* axis；
* limits；
* inertial；
* mesh path；
* sensor frame；
* camera optical frame；
* required links；
* required joints。

### 8.3 SRDF 测试

验证：

* XML；
* robot name；
* group；
* chain；
* subgroup；
* end effector；
* group state；
* disable collision；
* URDF link reference；
* URDF joint reference。

### 8.4 MJCF 测试

至少验证：

```text
mj_loadXML()
mj_forward()
```

并检查：

* asset 完整性；
* body/joint 名称；
* actuator；
* sensor；
* keyframe；
* initial qpos；
* 无 NaN；
* 无严重 penetration；
* 短时间 simulation smoke test；
* 初始状态稳定。

### 8.5 URDF/MJCF 一致性测试

`test_consistency.py` 是项目最重要的测试之一。

至少比较：

```text
URDF                     MJCF

joint name          ↔    joint name
joint hierarchy     ↔    body hierarchy
joint type          ↔    joint type
joint axis          ↔    axis
joint origin        ↔    pose
joint limits        ↔    range
link mass           ↔    body mass
COM                 ↔    inertial position
inertia             ↔    inertia
sensor mount        ↔    body/site pose
```

一致性检查应按设备输出。

例如：

```text
[franka_tmr]
[PASS] ...

[franka_spine]
[PASS] ...

[franka_fr3]
[PASS] ...

[realsense_d455]
[PASS] ...
```

### 8.6 容差与差异白名单

浮点值可以设置合理容差：

* translation tolerance；
* rotation tolerance；
* mass tolerance；
* inertia tolerance。

禁止通过不断增大 tolerance 掩盖真实模型漂移。

允许存在的结构差异必须进入显式 whitelist。

例如：

```text
URDF visual-only frame
MJCF contact-only geom
MJCF site
MJCF camera
backend-specific collision object
```

禁止：

```text
if mismatch:
    ignore
```

这种隐式行为。

### 8.7 一致性错误输出

推荐：

```text
[PASS] left_fr3_joint1 axis
[PASS] left_fr3_joint1 range
[PASS] right_fr3_joint4 origin

[FAIL] head_camera_mount

URDF:
  xyz = [0.120, 0.000, 0.215]

MJCF:
  pos = [0.125, 0.000, 0.215]

delta:
  [0.005, 0.000, 0.000]
```

测试输出必须直接告诉开发者：

> 哪个设备、哪个关节、哪个参数发生漂移。

### 8.8 官方兼容性测试

必须验证：

```text
franka_description == 2.8.1
robot_type == mobile_fr3_duo_v0_2
```

以及：

* expected files；
* expected links；
* expected joints；
* key mounting frames；
* asset hashes。

如果检测到其它版本，例如：

```text
2.9.x
main
unknown
```

默认应失败。

不能静默使用系统安装的其它版本。

---

## 9. 与现有 mobile_fr3_duo 的迁移

### 9.1 迁移目标

现有：

```text
SiYueY/mobile_fr3_duo
```

已经包含大量成熟的：

* MJCF；
* Builder；
* asset processing；
* source preparation；
* Golden URDF；
* validation；
* model resources。

这些成果应迁移到：

```text
mfr3duo_description
```

而不是重新实现。

### 9.2 现有设备集合

现有 `mobile_fr3_duo/models`：

```text
franka_fr3
franka_hand
franka_head
franka_spine
franka_tmr
imu
nanoscan3
realsense_d435
realsense_d455
wrist_camera_mount
zed_mini
mobile_fr3_duo.xml
scene.xml
```

这些名称直接成为 `mfr3duo_description` 的 Canonical Device Naming。

### 9.3 MJCF 顶层文件迁移

现有：

```text
models/mobile_fr3_duo.xml
```

迁移为：

```text
mjcf/mfr3duo.xml
```

现有：

```text
models/scene.xml
```

迁移为：

```text
mjcf/scene.xml
```

项目公开 robot identifier 统一调整为：

```text
mfr3duo
```

但 upstream 名称仍保留：

```text
mobile_fr3_duo_v0_2
```

### 9.4 设备资源迁移

例如：

```text
mobile_fr3_duo/models/franka_fr3/
        ↓
mfr3duo_description/mjcf/assets/franka_fr3/
```

同理迁移：

```text
franka_hand
franka_head
franka_spine
franka_tmr
imu
nanoscan3
realsense_d435
realsense_d455
wrist_camera_mount
zed_mini
```

但迁移时不应机械地把整个旧目录当作 asset 目录。

需要区分：

* XML model fragment；
* visual asset；
* collision asset；
* texture；
* metadata。

其中 XML 结构应进入：

```text
mjcf/
```

或：

```text
mjcf/model/
```

真正几何资源才进入：

```text
mjcf/assets/<device>/
```

### 9.5 Builder 与验证迁移

以下成熟能力应优先复用：

* source validation；
* Golden URDF generation；
* mesh conversion；
* asset manifest；
* Canonical IR；
* MJCF Builder；
* MJCF validation。

迁移原则：

> 尽量重构路径和命名，不重写已经验证过的核心逻辑。

### 9.6 不长期双仓维护

不推荐长期保持：

```text
mobile_fr3_duo
    └── MJCF

mfr3duo_description
    └── URDF
```

这种结构。

否则很容易出现：

```text
URDF updated
MJCF forgotten
```

或：

```text
sensor moved in MJCF
URDF not updated
```

最终两套模型会发生漂移。

因此模型层应逐步统一进入：

```text
mfr3duo_description
```

---

## 10. ROS 2 Package 与上层边界

### 10.1 Package 类型

项目采用：

```text
ament_cmake
```

作为标准 ROS 2 description package。

但需要明确：

> MJCF 本身仍保持 ROS-Free。

### 10.2 CMake 设计

保持简单，例如：

```cmake
cmake_minimum_required(VERSION 3.8)

project(mfr3duo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    srdf
    mjcf
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

Description package 不需要额外 C++ Library。

### 10.3 Runtime 安装内容

安装：

```text
urdf/
srdf/
mjcf/
launch/
rviz/
```

默认不安装：

```text
source/
tools/
test/
```

这些主要服务：

* 开发；
* source preparation；
* CI；
* model generation。

### 10.4 基础依赖

基础依赖：

```text
ament_cmake
xacro
franka_description
```

Display 依赖：

```text
robot_state_publisher
joint_state_publisher_gui
rviz2
```

Test 依赖根据实际测试实现增加。

不直接依赖：

```text
moveit_ros_move_group
nav2_bringup
controller_manager
franka_hardware
MuJoCo ROS runtime
```

### 10.5 Display Launch

提供：

```text
launch/display.launch.py
```

使开发者能够：

```bash
ros2 launch mfr3duo_description display.launch.py
```

验证 URDF。

启动链：

```text
Xacro
  ↓
robot_description
  ↓
robot_state_publisher
  ↓
joint_state_publisher_gui
  ↓
RViz
```

该 launch 不启动：

* MoveIt；
* Nav2；
* ros2_control；
* MuJoCo；
* 实机驱动。

### 10.6 RViz

`display.rviz` 仅保留：

* RobotModel；
* TF；
* Grid。

主要用于检查：

* geometry；
* joint movement；
* TF；
* sensor frame；
* mount frame。

### 10.7 与 MoveIt 的关系

未来：

```text
mfr3duo_moveit_config
```

依赖：

```text
mfr3duo_description
```

使用：

```text
URDF
SRDF
```

MoveIt package 负责：

```text
kinematics
planning pipeline
trajectory controller mapping
planning scene config
```

### 10.8 与 Nav2 的关系

Nav2 使用：

```text
base_link
sensor frame
lidar frame
camera frame
TF hierarchy
```

但以下内容不进入 description：

```text
planner
controller
costmap
BT
localization
SLAM
```

### 10.9 与 ros2_control 的关系

V1 不在基础 description 中内置完整：

```xml
<ros2_control>
```

上层控制包可以通过 Xacro composition：

```text
mfr3duo base URDF
        +
ros2_control extension
```

分别支持：

```text
real hardware
MuJoCo
fake system
```

### 10.10 与 MuJoCo Runtime 的关系

未来如果存在：

```text
mfr3duo_mujoco
```

它应负责：

* simulation launch；
* world；
* ROS bridge；
* ros2_control integration；
* runtime parameters。

机器人 MJCF：

```text
mjcf/mfr3duo.xml
```

仍然属于：

```text
mfr3duo_description
```

---

## 11. 命名与版本策略

### 11.1 项目命名

统一：

```text
package:
mfr3duo_description

robot:
mfr3duo

URDF:
mfr3duo.urdf.xacro

MJCF:
mfr3duo.xml

SRDF:
mfr3duo.srdf.xacro
```

避免继续混用：

```text
mobile_fr3_duo
franka_mobile_fr3_duo
mfr3_duo
```

上游官方 identifier 除外。

### 11.2 Joint Naming

Joint naming 对以下上层组件影响很大：

* MoveIt；
* ros2_control；
* controller；
* telemetry；
* logs；
* bag。

原则：

> 能保持官方名称时，不进行无意义重命名。

如果必须引入左右 prefix，则 URDF 和 MJCF 必须完全一致。

在 Joint Naming 冻结之前，不应大量构建：

* MoveIt config；
* ros2_control config；
* controller config。

### 11.3 Frame Naming

推荐稳定使用：

```text
*_mount
*_link
*_frame
*_optical_frame
```

其中：

```text
mount
```

表示机械安装接口。

例如：

```text
wrist_camera_mount
        │
        ▼
realsense_d455_link
        │
        ▼
realsense_d455_color_optical_frame
```

### 11.4 MJCF Naming

MJCF 中尽量复用 URDF 关键名称。

尤其：

* joint；
* body；
* sensor mount；
* camera；
* site。

MuJoCo 中：

```text
body
joint
geom
site
camera
sensor
actuator
```

名称需要全局唯一。

不能假设不同 `<include>` 文件提供 namespace。

### 11.5 版本冻结

正式 release 必须记录：

```text
mfr3duo_description version
franka_description version
robot type
asset source
asset hash
generated asset hash
```

V1 明确：

```text
franka_description = 2.8.1
```

后续如果考虑升级，必须执行：

```text
2.8.1
  ↓
upstream diff
  ↓
model diff
  ↓
asset diff
  ↓
URDF validation
  ↓
MJCF validation
  ↓
cross-format validation
  ↓
manual review
  ↓
new baseline
```

不能直接修改版本号后继续构建。

---

## 12. 开发计划

### Phase 1：建立项目骨架

完成：

```text
package.xml
CMakeLists.txt

urdf/
srdf/
mjcf/
source/
tools/
launch/
rviz/
test/
```

并建立基础 CI。

### Phase 2：冻结官方基线

在：

```text
source/model_manifest.yaml
```

明确：

```text
franka_description = 2.8.1
robot_type = mobile_fr3_duo_v0_2
```

建立自动版本校验。

### Phase 3：迁移现有 MJCF

迁移现有：

* `mobile_fr3_duo.xml`；
* `scene.xml`；
* device models；
* assets；
* Builder；
* Golden URDF；
* source tracking；
* validation。

要求迁移完成后：

```bash
simulate mjcf/scene.xml
```

仍能独立运行。

### Phase 4：重组 MJCF Assets

按照 Canonical Device Set：

```text
franka_fr3
franka_hand
franka_head
franka_spine
franka_tmr
imu
nanoscan3
realsense_d435
realsense_d455
wrist_camera_mount
zed_mini
```

重组：

```text
mjcf/assets/<device>/
```

设备内部再区分：

```text
visual/
collision/
textures/
```

同步完善：

```text
asset_manifest.yaml
```

### Phase 5：建立正式 URDF

基于：

```text
franka_description 2.8.1
mobile_fr3_duo_v0_2
```

建立：

```text
urdf/mfr3duo.urdf.xacro
urdf/mfr3duo.xacro
```

验收：

```text
xacro expansion PASS
check_urdf PASS
```

### Phase 6：补齐设备 URDF

逐个确认：

```text
franka_fr3
franka_hand
franka_head
franka_spine
franka_tmr
imu
nanoscan3
realsense_d435
realsense_d455
wrist_camera_mount
zed_mini
```

在 URDF 中属于哪种状态：

* upstream 已提供；
* 直接引用；
* project extension；
* project mesh；
* frame-only；
* full visual/collision。

### Phase 7：建立 SRDF

完成：

```text
left_arm
right_arm
left_hand
right_hand
left_arm_hand
right_arm_hand
dual_arm
```

以及：

* end effector；
* collision exclusions；
* 必要 group states。

### Phase 8：冻结 Naming

冻结：

* URDF link；
* URDF joint；
* MJCF body；
* MJCF joint；
* mount frame；
* sensor frame；
* actuator。

形成明确 mapping。

### Phase 9：建立 Cross-Format Validation

实现：

```text
test_consistency.py
```

比较：

* topology；
* joint；
* axis；
* limits；
* transform；
* mass；
* inertia；
* sensor mount。

### Phase 10：建立 Official Compatibility Validation

实现：

```text
test_official_compatibility.py
```

确保：

```text
franka_description == 2.8.1
robot_type == mobile_fr3_duo_v0_2
```

以及 upstream 关键资源未发生意外变化。

### Phase 11：双端模型验收

ROS：

```bash
ros2 launch mfr3duo_description display.launch.py
```

MuJoCo：

```bash
simulate mjcf/scene.xml
```

两端均应可以独立加载并完成基本验证。

### Phase 12：MoveIt 前置验证

本阶段不实现完整 MoveIt config。

只验证：

* URDF 可被 MoveIt 使用；
* SRDF 合法；
* left/right arm chain 正确；
* hand/end effector 正确；
* collision geometry 合理；
* naming 已冻结。

---

## 13. V1 验收标准

### 13.1 官方基线

必须满足：

```text
[PASS] franka_description == 2.8.1
[PASS] robot_type == mobile_fr3_duo_v0_2
```

### 13.2 Canonical Device Set

必须确认：

```text
[PASS] franka_fr3
[PASS] franka_hand
[PASS] franka_head
[PASS] franka_spine
[PASS] franka_tmr
[PASS] imu
[PASS] nanoscan3
[PASS] realsense_d435
[PASS] realsense_d455
[PASS] wrist_camera_mount
[PASS] zed_mini
```

每个设备必须有明确的：

* source；
* model status；
* URDF status；
* MJCF status；
* asset status。

### 13.3 URDF

必须满足：

```text
[PASS] Xacro expansion
[PASS] XML
[PASS] URDF parser
[PASS] check_urdf
[PASS] robot_state_publisher
[PASS] RViz
[PASS] required links
[PASS] required joints
[PASS] sensor frames
```

### 13.4 SRDF

必须满足：

```text
[PASS] XML
[PASS] groups
[PASS] chains
[PASS] end effectors
[PASS] collision exclusions
[PASS] URDF references
```

### 13.5 MJCF

必须满足：

```text
[PASS] mj_loadXML
[PASS] mj_forward
[PASS] scene load
[PASS] no missing assets
[PASS] no duplicate names
[PASS] actuator definitions
[PASS] sensor definitions
[PASS] no NaN
[PASS] stable initial state
```

### 13.6 Cross-Format

必须满足：

```text
[PASS] topology
[PASS] joint names
[PASS] joint hierarchy
[PASS] joint axis
[PASS] joint limits
[PASS] key transforms
[PASS] mass
[PASS] COM
[PASS] inertia
[PASS] sensor mounting
```

### 13.7 Source Traceability

必须满足：

```text
[PASS] upstream repository recorded
[PASS] franka_description 2.8.1 fixed
[PASS] robot_type recorded
[PASS] Canonical Device Set recorded
[PASS] asset source recorded
[PASS] conversions recorded
[PASS] generated files traceable
[PASS] required hashes validated
```

---

## 14. 最终冻结原则

V1 最终冻结以下设计规则：

1. `franka_description` 固定为 `2.8.1`；
2. upstream robot type 固定为 `mobile_fr3_duo_v0_2`；
3. 项目公开机器人名统一为 `mfr3duo`；
4. URDF 与 MJCF 是两个独立正式模型实现；
5. URDF 与 MJCF 文件物理分离；
6. URDF Mesh 位于 `urdf/meshes/`；
7. MJCF Asset 位于 `mjcf/assets/`；
8. 不建立公共 runtime asset 目录；
9. URDF 与 MJCF 资源均采用“设备优先、资源类型次级”的目录结构；
10. Canonical Device Naming 严格继承现有 `mobile_fr3_duo/models`；
11. Canonical Device Set 为：

```text
franka_fr3
franka_hand
franka_head
franka_spine
franka_tmr
imu
nanoscan3
realsense_d435
realsense_d455
wrist_camera_mount
zed_mini
```

12. 左右相同设备共享同一套资源；
13. 不为了目录形式完整创建无意义空目录；
14. Franka 官方机械结构优先复用，不重新维护一份完整 fork；
15. URDF 可以直接引用稳定的 `franka_description 2.8.1` 官方资源；
16. MJCF 必须保持 self-contained 和 ROS-Free；
17. Visual Geometry 尽量保持同一 upstream source，但不要求同一物理文件；
18. Collision Geometry 可以针对 MoveIt 与 MuJoCo 分别优化；
19. Shared Source Layer 只保存真正跨格式共享的数据；
20. MuJoCo-specific 参数不得为了所谓“统一”进入 URDF/shared parameter；
21. MoveIt-specific 参数不得进入基础 description；
22. Golden URDF 必须由 `franka_description 2.8.1 + mobile_fr3_duo_v0_2` 确定性生成；
23. Canonical IR 只作为内部生成/验证数据结构；
24. URDF/MJCF 机械语义必须通过自动测试保持一致；
25. upstream 版本升级必须作为独立模型升级任务处理；
26. `mfr3duo_description` 不承担 MoveIt、Nav2、ros2_control 或 MuJoCo Runtime。

最终目标是确保：

```text
ROS 2 / MoveIt 看到的 mfr3duo
```

与：

```text
MuJoCo 看到的 mfr3duo
```

在机器人拓扑、关节定义、惯性参数、关键安装关系以及传感器安装关系上始终代表**同一台 Mobile FR3 Duo**，同时允许 URDF 与 MJCF 根据各自 backend 的能力和用途采用最适合的模型表达方式。
