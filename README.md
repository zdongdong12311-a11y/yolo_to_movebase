# yolo_to_movebase

把 YOLO 的 2D 检测框，结合 MAVROS 位姿，解算成**以起飞点为原点**的世界坐标（ENU），加偏移后输出给 move_base，或发到自定义话题由主程序接管。

- 平台：PX4 + MAVROS + ROS1 Noetic（C++14）
- 上游：YOLOv8 / 任意发布 `vision_msgs/Detection2DArray` 的检测节点
- 定位：单目标点的三维定位，是三维重建、目标跟随、精准抛投的基础环节

```text
YOLO 像素框 ──┐
              ├─► yolo_to_movebase ─► /move_base_simple/goal（或自定义话题）
MAVROS 位姿 ──┘                     └► TF：world → drone → cam / target
```

## 1. 实现了什么（功能 → 方法）

| 功能 | 实现方法 |
|------|----------|
| 多相机组合（下视 / 前视 / 双相机） | 纯配置驱动：`cameras` 列表决定启用哪些相机，每相机一个参数段；每相机注册独立订阅者，用 `std::bind` 绑定相机名，互不干扰 |
| 像素 → 世界坐标 | 针孔模型反投影 → 安装外参（光心位置 + 安装角，含「光学系→FLU」固定旋转）→ 里程计四元数位姿 → ENU 世界系，四步链式变换 |
| 下视测距 `ground` | 把像素还原成世界系射线，与平面 `z = ground_z` 求交；深度由里程计高度决定，**不依赖目标尺寸** |
| 前视测距 `distance` | 相似三角形 `Z = fx · H_real / h_px`，用类别真实高度反算深度，再反投影 |
| 倾斜保护 | 用四元数算机体 z 轴与世界 z 轴夹角；超过 `max_tilt_deg` 丢弃该帧，避免像素误差被放大成地面位置误差 |
| 置信度 / 距离过滤 | 多目标取最高分；再按 `min_score`、`min_range` / `max_range` 过滤 |
| 发布节流 | 距上次发布不足 `publish_delay`（默认 8 s）时，用 one-shot 定时器**延迟发布最新目标**，不是丢弃 |
| 运行时切测距模式 | 订阅 `/yolo_to_movebase/set_depth_mode`，格式 `相机名:ground\|distance\|off`；只改内存，无需重启 |
| 目标点输出 | `PoseStamped`：位置 = 目标 + `(x_offset, 0, z_offset)`；朝向 = 当前 yaw（保持机头方向） |
| TF 可视化 | 10 Hz 广播 `world→drone_current→cam_*`、`world→target_original_*`、`world→target_final_*`、历史目标 `target_i_*` |
| `vision_msgs` 双版本兼容 | `results[0].id` 同时重载 `int64` 与 `string` 两种签名（apt 版是 int64，源码新版可能是 string） |
| 话题全部可覆盖 | `down_topic` / `front_topic` / `odom_topic` / `goal_topic` 都能从 launch 覆盖，不传就用 yaml / 源码默认值 |

## 2. 坐标与算法

### 2.1 三个坐标系

| 坐标系 | 轴向 | 用在哪 |
|--------|------|--------|
| 相机光学系 | x 右、y 下、z 前（光轴） | 内参 `fx/fy/cx/cy` 的基准 |
| 机体系 FLU | x 前、y 左、z 上 | 安装外参 `mount_xyz` / `mount_rpy_deg` 的基准 |
| 世界系 ENU | x 东、y 北、z 上 | 目标点输出；原点是**起飞点**（odom 原点） |

> 原点永远是起飞点，与检测时飞机在哪无关。飞控/EKF 重置会让原点重设，之前的目标全部作废。

### 2.2 四步变换

```text
像素 (u, v)
  │ ① 针孔反投影
  ▼
相机系   d_cam = [ (u−cx)/fx , (v−cy)/fy , 1 ]
  │ ② 安装外参   R_cb = R(mount_rpy) · R(光学→FLU)
  ▼
机体系   p_body = mount_xyz + R_cb · p_cam
  │ ③ 位姿变换   R_wb、T_wb 来自里程计（ENU）
  ▼
世界系   P_world = T_wb + R_wb · p_body
```

- `R_wb` 默认用**完整四元数**（`use_full_attitude: true`），倾斜时自动补偿；设 `false` 则只取 yaw，仅适合水平悬停对照。
- 光学系→FLU 的固定旋转已封装在 `camera_config.hpp` 的 `opticalToFlu()` 里，填安装角时不用管它。

### 2.3 两种测距模式

**ground（下视推荐）**——射线与地平面求交，深度来自高度：

```text
C_world = T_wb + R_wb · mount_xyz          相机光心世界坐标
d_world = R_wb · R_cb · d_cam              射线方向
t       = (ground_z − C_world.z) / d_world.z
P_world = C_world + t · d_world
```

**distance（前视推荐）**——相似三角形，深度来自目标真实尺寸：

```text
Z       = fx · H_real / h_px                H_real: 类别真实高度, h_px: 框高(像素)
p_cam   = [ (u−cx)/fx · Z , (v−cy)/fy · Z , Z ]
P_world = T_wb + R_wb · ( mount_xyz + R_cb · p_cam )
```

| | ground | distance |
|---|---|---|
| 适用 | 下视 | 前视 / 斜视 |
| 深度来源 | 里程计高度 + 地面求交 | 目标真实尺寸 `real_sizes` |
| 回传 z | **恒等于 `ground_z`** | 几何解算值，随帧变化 |
| 主要误差源 | `ground_z` 与实际地面高差 | 尺寸标定误差（按比例放大） |
| 局限 | 射线须与地面相交，前视会平行失败 | 类别没配 `real_sizes` 就测不了 |

## 3. 快速开始

```bash
# 依赖
sudo apt install ros-noetic-vision-msgs libeigen3-dev

# 编译
cd ~/yolo_ws/src
# 放入 yolo_to_movebase 包
cd ~/yolo_ws
catkin_make
source devel/setup.bash

# 运行（三选一）
roslaunch yolo_to_movebase yolo_to_movebase.launch camera:=down
roslaunch yolo_to_movebase yolo_to_movebase.launch camera:=front
roslaunch yolo_to_movebase yolo_to_movebase.launch camera:=dual
```

临时换话题（不改 yaml、不改源码）：

```bash
roslaunch yolo_to_movebase yolo_to_movebase.launch camera:=down \
    down_topic:=/yolo/down_result odom_topic:=/odometry/global \
    goal_topic:=/yolo_target_world
```

运行日志：

```text
[down] 检测到 person  score=0.87  px=(640, 360)
[down] 目标世界坐标: (3.21, -1.05, 0.00)  距离: 5.02 m
已发布目标 'person'(down相机) -> (3.21, -1.05, 0.50)  偏移(x:+0.00, z:+0.50)
```

## 4. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | 每相机一个（yaml `topic`） | `vision_msgs/Detection2DArray` | 上游 YOLO 检测 |
| 订阅 | `/mavros/local_position/odom`（可覆盖） | `nav_msgs/Odometry` | 位姿来源 |
| 订阅 | `/yolo_to_movebase/set_depth_mode` | `std_msgs/String` | 运行时切测距模式 |
| 发布 | `goal_topic`（默认 `/move_base_simple/goal`） | `geometry_msgs/PoseStamped` | 目标点 |
| 发布 | `/tf` | `tf2_msgs/TFMessage` | 可视化 |

切模式：

```bash
rostopic pub -1 /yolo_to_movebase/set_depth_mode std_msgs/String "data: 'down:ground'"
rostopic pub -1 /yolo_to_movebase/set_depth_mode std_msgs/String "data: 'front:distance'"
rostopic pub -1 /yolo_to_movebase/set_depth_mode std_msgs/String "data: 'down:off'"
```

## 5. 配置

三个 yaml 结构一致，只差 `cameras` 列表和相机段：`cameras_down.yaml` / `cameras_front.yaml` / `cameras_dual.yaml`。

```yaml
world_frame: "world"
use_full_attitude: true
publish_delay: 8.0          # 两次发布的最小间隔(秒)
tf_rate: 10.0

class_names:
  "0": "person"
  "1": "car"

cameras: ["down"]

down:
  topic: "/yolo_msgs_down"
  fx: 1000.0                # TODO 换成标定值
  fy: 1000.0
  cx: 640.0
  cy: 360.0
  mount_rpy_deg: [0.0, 90.0, 0.0]   # 光轴朝下
  mount_xyz: [0.0, 0.0, -0.05]
  depth_mode: "ground"
  ground_z: 0.0
  min_score: 0.5
  min_range: 0.5
  max_range: 15.0
  max_tilt_deg: 15.0
  x_offset: 0.0
  z_offset: 0.5             # 悬停在目标上方 0.5 m
```

关键参数：

| 参数 | 代码默认 | 说明 |
|------|----------|------|
| `world_frame` | `world` | 输出目标点与 TF 的父坐标系，**要与里程计 frame_id 一致** |
| `use_full_attitude` | `true` | `true` 用完整四元数补偿倾斜；`false` 只用 yaw |
| `publish_delay` | `8.0` | 发布最小间隔，太小目标点会跳，太大响应慢 |
| `class_names` | — | 类别 id → 名称，缺项显示 `class_<id>` |
| `topic` | `/yolo_msgs` | 该相机的上游检测话题 |
| `fx` / `fy` / `cx` / `cy` | 1000 / 1000 / 640 / 360 | **占位值，必须标定替换** |
| `mount_xyz` | `[0,0,0]` | 相机光心在机体系的位置（米） |
| `mount_rpy_deg` | `[0,0,0]` | 安装朝向（度）：下视 `[0,90,0]`，前视 `[0,0,0]` |
| `depth_mode` | `ground` | `ground` / `distance` / `off` |
| `ground_z` | `0.0` | 地平面世界高度，ground 模式必填 |
| `real_sizes` | — | 类别真实高度（米），distance 模式必填 |
| `min_score` | `0.5` | 置信度门槛 |
| `min_range` / `max_range` | `0.5` / `80.0` | 有效距离区间（米）；yaml 示例收紧到 15 |
| `max_tilt_deg` | `-1.0`（不限制） | 倾斜保护阈值，`<=0` 关闭；yaml 示例 15 |
| `x_offset` / `z_offset` | `0.0` / `0.0` | 发布前偏移，**世界系 ENU 方向，不随机头旋转** |

## 6. 目录结构

```text
yolo_to_movebase/
├── CMakeLists.txt
├── package.xml
├── include/yolo_to_movebase/camera_config.hpp   # 配置结构体 + 坐标变换数学
├── src/
│   ├── yolo_to_movebase_node.cpp                # 主节点
│   └── camera_config.cpp                        # 从 rosparam 读相机配置
├── config/                                      # down / front / dual 三套 yaml
└── launch/                                      # 主入口 + down/front 便捷入口
```

## 7. 注意事项与已知限制

1. **内参必须标定**：`1000/640/360` 只是 720p 占位值，距离会成比例偏。
2. **安装角填错会镜像或旋转**：下视 `[0,90,0]`、前视 `[0,0,0]`；上飞机前先做地面手持验证（卷尺对照日志坐标，误差 < 0.3 m）。
3. **`world_frame` 要和里程计 frame_id 一致**（MAVROS 常见 `map` 或 `odom`），否则 TF 树 / move_base 对不上，RViz 里会错位。
4. **ground 模式回传的 z 恒为 `ground_z`**，代表目标「脚底」的地面点；最终 goal 的 z = `ground_z + z_offset`。
5. **`max_range` 必须 ≥ 巡航高度**：下视斜距约等于高度，超出会被距离过滤，永远检不到。
6. **偏移只有 x / z，没有 y**：风偏是北向的，需要补 `y_offset`（改 `camera_config.hpp` 字段、`camera_config.cpp` 读参、`publishGoal` 加一项）。
7. **双相机同时检测时**，8 s 窗口内只保留一个待发布目标，后来的检测会覆盖之前的（内部只有一份 `target_world_`）；对时序敏感就分开跑两个节点。
8. 前视相机用 `ground` 模式会因射线与地面平行而失败，前视请用 `distance`。

## 8. 故障排查

| 现象 | 原因 | 处理 |
|------|------|------|
| 启动即 FATAL「缺少参数 cameras」 | yaml 没加载 | 检查 `camera` 参数拼写、`config` 路径 |
| 一直没有「检测到 xxx」 | 上游话题名/类型不对 | `rostopic info <topic>` 确认是 `Detection2DArray`，或用 `down_topic:=` 覆盖 |
| 一直「尚未收到里程计」 | odom 话题不对 | `rostopic info` 确认 `nav_msgs/Odometry`，或用 `odom_topic:=` 覆盖 |
| 频繁「距离超出 [a,b]」 | 区间太窄 / 内参离谱 | 先放宽区间排查，再收紧 |
| 频繁「机体倾斜 xx°」 | 阈值太小 / 巡航俯仰大 | 放宽 `max_tilt_deg` 或减速悬停后再识别 |
| 有「目标世界坐标」但没发布 | 8 s 节流中，或日志漏看 | `rostopic echo <goal_topic>` 直接验证 |
| goal 的 z 不对 | 看的是加偏移前的日志 | 「已发布目标」那行才是含 `z_offset` 的最终值 |
| 自定义话题收不到 | 话题名/类型不一致，或订阅方启动晚 | `rostopic info` 核对两端；必要时给 publisher 加 latch |

## License

MIT
