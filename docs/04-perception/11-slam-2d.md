# 第 11 章 2D SLAM 建图实战

> 前面十章,你已经让 Go2 能动、能听话、能在 RViz 里"照镜子"。但它还没有"空间记忆"—— 走出去十米就不知道自己走了多远,更别提回家。从本章开始,我们给它装上这块记忆。

---

## 本章你将学到

- 理解激光 SLAM 的基本思路(前端扫描匹配 + 后端图优化 + 回环检测)
- 明白为什么 Go2 的**3D 点云**要被"压扁"成 **2D 激光扫描**
- 用 SLAM Toolbox 在线建出一张可用的 2D 栅格地图
- 诊断 3 类典型踩坑:**点云时间戳延迟**、**QoS 策略不兼容**、**TF 坐标系缺失**
- 把建好的地图保存下来,为第 13 章的自主导航打底

---

## 背景与原理

### 什么是 SLAM

SLAM 全称 **Simultaneous Localization And Mapping**,同时定位与建图。

想象你第一次进一座迷宫 —— 没有地图,但你得一边走一边做两件事:

1. **记住自己在哪**(相对于起点往北走了多少、左转了几次)
2. **画一张地图**(把看到的墙壁、门、拐角记下来)

这两件事是**互相依赖**的:你得先知道"我在哪",才知道刚刚看到的墙画在地图的什么位置;反过来,地图画出来后又能帮你确认"我真的走到这儿了吗"。

机器人做 SLAM 就是这个过程自动化。算法层面通常拆成三块:

| 模块 | 干什么 | 类比 |
|---|---|---|
| **前端(Scan Matching)** | 把当前这一帧激光数据,和上一帧(或最近的关键帧)对齐,算出"我移动了多少" | 数步子 |
| **后端(Graph Optimization)** | 把所有关键帧连成一张图,反复微调让整体一致 | 回家后对着笔记把地图画端正 |
| **回环检测(Loop Closure)** | 识别出"哎?这地方我好像来过",把累积的误差一次性修正 | 走了一圈发现回到起点,把地图首尾对齐 |

### 2D 还是 3D?

SLAM 按地图维度分 2D(平面栅格)和 3D(点云/体素)两大类。本书**第一次建图选 2D**,因为:

- 教学友好 —— 一张俯视图,眼睛一看就懂
- 硬件要求低 —— 不需要强劲 GPU
- Nav2 原生支持 —— 下一章导航直接用
- 室内平地够用 —— Go2 的主战场就是这种场景

至于 3D SLAM(KISS-ICP / FAST-LIO / Point-LIO 之类),我们放在第 12 章再讲。

### SLAM Toolbox 是什么

[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) 是 ROS2 生态里最主流的 2D 激光 SLAM 包。它的卖点:

- 开源、文档全、社区活跃
- 支持**在线建图**(边走边画) 和 **离线建图**(回放 bag)
- 支持**异步模式**(async) —— 不保证每帧都处理,但低延迟,适合实时建图
- 原生支持**地图序列化** —— 下次可以继续在这张地图上补建,不用重新开始

本章我们用**异步在线模式**。

### Go2 雷达的"适配问题"

Go2 EDU 自带的是 **L1 4D 雷达**,发布的是 **3D 点云**(`/utlidar/cloud_deskewed`)。但 SLAM Toolbox 只吃 **2D LaserScan**(`/scan`)。

解决办法:**把 3D 点云压扁成 2D 扫描** —— 只保留某个高度区间内的点,投影到一个水平面上。这个活由 ROS2 官方的 `pointcloud_to_laserscan` 包完成。

!!! note "为什么这样做不是"掉精度""
    建图只关心"哪里能走、哪里是墙"。墙从地面到天花板都是墙,保留 0.1m 高度的那一圈就够判断。天花板和地板,我们**主动过滤掉**—— 因为地板会被当成"障碍物"把路堵死。

---

## 架构总览

### 数据流

```mermaid
flowchart TD
    A[Go2 UTlidar 硬件] -->|原始 3D 点云| B["/utlidar/cloud_deskewed<br/>(odom 坐标系)"]
    B -->|时间戳延迟 ~510s ⚠️| C[pointcloud_timestamp_fix<br/>用当前时间重写时间戳]
    C -->|时间戳正确| D["/utlidar/cloud_fixed"]
    D --> E[pointcloud_to_laserscan<br/>3D → 2D 投影]
    E -->|"/scan (2D 激光扫描)"| F[slam_toolbox<br/>异步在线建图]
    F -->|"/map + map→odom TF"| G[RViz 可视化]

    H[go2_driver_py] -->|"/odom + odom→base TF"| F
```

### TF 树

```
map                  ← SLAM Toolbox 发布
  └── odom           ← go2_driver_py 发布(里程计)
        └── base     ← 机器人躯干中心,离地约 0.31m
              ├── FL_hip → ... → FL_foot
              ├── FR_hip → ... → FR_foot
              ├── RL_hip → ... → RL_foot
              ├── RR_hip → ... → RR_foot
              ├── imu
              └── radar → utlidar_lidar
```

!!! warning "坐标系叫 `base` 不是 `base_link`"
    Go2 原生 TF 用的是 `base`,Nav2 默认期望 `base_link`。全书统一用 **`base`**,凡是第三方配置里写着 `base_link` 的地方都要改成 `base`。

---

## 环境准备

### 前置成果

本章**直接复用第 6 章完成的 `go2_driver_py` 包** —— 它已经能发布 `/odom` 和 `odom→base` 的 TF。如果你跳着看到这儿,先回第 6 章把驱动跑起来。

### 新增依赖

打开终端,装三件套:

```bash
# slam_toolbox: SLAM 核心包
# pointcloud_to_laserscan: 3D 点云压扁成 2D 扫描
# nav2_map_server: 用来保存地图成 pgm + yaml 格式
# tf2_tools: 可视化 TF 树(调试用)
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-nav2-map-server \
    ros-humble-tf2-tools
```

检查装没装上:

```bash
# 列一下相关包,有输出就是装上了
ros2 pkg list | grep -E "slam_toolbox|pointcloud_to_laserscan"
```

---

## 实现步骤

我们会在工作空间里新建两个 ROS2 包:

- `go2_sensors` —— 传感器数据处理(时间戳修复、点云转扫描)
- `go2_slam` —— SLAM 配置、启动文件、地图存储

### 步骤一:创建 go2_sensors 包

在工作空间的 `src/` 目录下创建感知包:

```bash
# 进入工作空间源码目录
cd ~/unitree_go2_ws/src

# 创建 Python 类型的 ROS2 包,声明需要的依赖
ros2 pkg create go2_sensors \
    --build-type ament_python \
    --dependencies rclpy sensor_msgs
```

### 步骤二:写"时间戳修复"节点

**为什么需要**:Go2 的 UTlidar 驱动发布点云时,带的时间戳比系统时间**早约 510 秒**(原因未明,可能是固件内部时钟漂移)。这会让下游节点查不到对应时刻的 TF,整条管线直接崩掉。

**怎么修**:写一个"中转节点",收到点云后把时间戳替换成**当前系统时间**,数据本身不动,再重新发出去。

在 `go2_sensors/go2_sensors/` 目录下创建 `pointcloud_timestamp_fix.py`:

```python
#!/usr/bin/env python3
"""
时间戳修复节点:订阅原始点云,重写时间戳后重新发布。
专治 Go2 UTlidar 点云时间戳早于系统时间的毛病。
"""

import rclpy                                       # ROS2 Python 客户端库,节点的入口
from rclpy.duration import Duration                # 用来构造"回拨 0.05 秒"的时间差
from rclpy.node import Node                        # 所有 ROS2 节点的基类
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS 策略,控制通信可靠性
from sensor_msgs.msg import PointCloud2            # 标准点云消息类型


class PointCloudTimestampFix(Node):
    def __init__(self):
        super().__init__("pointcloud_timestamp_fix")

        # 回拨 0.05 秒:保证下游"查 TF"时不会查到未来,否则第 12 章 AMCL
        # 会持续报 "Lookup would require extrapolation into the future"
        self.backdate = Duration(seconds=0.05)

        # 订阅 QoS:BEST_EFFORT(不保证每一帧都收到,但延迟低)
        # 选 BEST_EFFORT 是因为 Go2 驱动发布时兼容这个策略
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 发布 QoS:RELIABLE(保证 SLAM 一帧不漏,必要时重传)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 订阅原始点云
        self.sub = self.create_subscription(
            PointCloud2,
            "/utlidar/cloud_deskewed",
            self.on_cloud,
            sub_qos,
        )

        # 发布时间戳修复后的点云,下游节点来订这个话题
        self.pub = self.create_publisher(
            PointCloud2,
            "/utlidar/cloud_fixed",
            pub_qos,
        )

        self.get_logger().info("时间戳修复节点已启动(回拨 0.05s)")

    def on_cloud(self, msg: PointCloud2) -> None:
        # 用"当前时间 − 0.05s"覆盖原时间戳,其他字段不动
        msg.header.stamp = (self.get_clock().now() - self.backdate).to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PointCloudTimestampFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

!!! info "为什么要"回拨 0.05 秒"而不是直接用当前时间"
    Go2 的 TF 发布有几毫秒到几十毫秒的抖动。如果点云 stamp = 当前时间,下游 `pointcloud_to_laserscan` → AMCL 查 `odom→base` TF 时,**激光时刻可能比最新 TF 时刻还新几毫秒**,tf2 会报 `Lookup would require extrapolation into the future`,激光整包被丢掉。
    
    回拨 0.05s 等于永远用"刚才这一瞬间"的激光去对"刚才这一瞬间"的 TF,给 TF 查询留足缓冲,第 11 章建图和第 12 章 Nav2 都能直接受益。

**这段代码做了三件事**:

1. 声明两种 QoS:订阅端**宽松**(跟得上 Go2 原生驱动就行),发布端**严格**(不能让 SLAM 丢帧)
2. 订阅 `/utlidar/cloud_deskewed`,在回调里只改 `header.stamp`,其余 `fields` / `data` 完全不动
3. 发布到 `/utlidar/cloud_fixed`,下游节点订这个就拿到时间戳正确的点云

然后把节点注册进 `setup.py` 的 `entry_points`:

```python
entry_points={
    "console_scripts": [
        "pointcloud_timestamp_fix = go2_sensors.pointcloud_timestamp_fix:main",
    ],
},
```

### 步骤三:配置 pointcloud_to_laserscan

`pointcloud_to_laserscan` 是现成的包,我们只需要给它一份参数文件。在 `go2_sensors/config/pointcloud_to_laserscan_params.yaml` 写:

```yaml
# 3D 点云 → 2D 扫描 的转换参数
/pointcloud_to_laserscan_node:
  ros__parameters:
    # ---- 坐标系 ----
    target_frame: base               # 输出的 LaserScan 定义在 base 坐标系下
    transform_tolerance: 0.5         # TF 查询容忍,时间戳修复后仍可能有小抖动,这个能兜住

    # ---- 高度过滤(相对 base 坐标系)----
    # base 原点离地面 ~0.31m,即地面在 base 中 Z ≈ -0.31
    min_height: ==-0.28==            # 略高于地面,防止地面反射当障碍
    max_height: ==1.0==              # 足够覆盖大部分室内障碍(桌椅/人)

    # ---- 角度范围(全圆)----
    angle_min: -3.14159
    angle_max: 3.14159
    angle_increment: 0.0087          # 约 0.5°/射线,一圈 723 射线

    # ---- 距离过滤 ----
    range_min: ==0.3==               # 低于此距离的点视为自身反射或近场噪声
    range_max: ==20.0==              # L1 雷达有效测距远端

    # ---- 性能 ----
    scan_time: 0.1
    concurrency_level: 1
    use_inf: true                    # 无返回时填 inf 而非最大距离(SLAM 更稳)
```

!!! tip "高度过滤参数是调出来的,不是算出来的"
    我们用对照实验测过三组参数:
    
    | min_height | max_height | range_min | 有效射线 | 平均距离 | 评价 |
    |:-:|:-:|:-:|:-:|:-:|---|
    | -0.5 | 1.0 | 0.1 | 255 (35%) | 0.79 m | 地面噪声太多,墙壁被淹没 |
    | -0.1 | 0.5 | 0.1 | 61 (8%) | 2.25 m | 过滤过头,看不到桌椅 |
    | **-0.28** | **1.0** | **0.3** | **51 (7%)** | **4.75 m** | **本书选用** |
    
    别看有效射线只有 7%,这些射线的"信噪比"最高,建出的地图最干净。

### 步骤四:创建 go2_slam 包

```bash
cd ~/unitree_go2_ws/src

# SLAM 包只放配置和 launch,不需要 Python 节点,用最轻的 ament_cmake 即可
ros2 pkg create go2_slam \
    --build-type ament_cmake
```

建立子目录:

```bash
# config 放参数文件,launch 放启动文件,maps 存建好的地图
mkdir -p go2_slam/config go2_slam/launch go2_slam/maps
```

### 步骤五:配置 SLAM Toolbox

在 `go2_slam/config/slam_toolbox_params.yaml` 写:

```yaml
# SLAM Toolbox 异步在线建图参数
slam_toolbox:
  ros__parameters:
    # ---- 运行模式 ----
    use_sim_time: false              # 实机运行,使用系统时间
    mode: mapping                    # mapping(建图) / localization(纯定位)

    # ---- 坐标系 ----
    odom_frame: odom
    map_frame: map
    base_frame: base                 # ⚠ Go2 用 base 不是 base_link
    scan_topic: /scan

    # ---- QoS 匹配 ----
    # /scan 上游用 BEST_EFFORT,这里必须对齐,否则订不到
    # ⚠ slam_toolbox 只认扁平的点分 key 格式,嵌套写法会被静默忽略
    qos_overrides:
      "/scan.subscription.reliability": "best_effort"
      "/scan.subscription.durability": "volatile"
      "/scan.subscription.history": "keep_last"
      "/scan.subscription.depth": 10

    # ---- TF / 时间 ----
    transform_publish_period: 0.02   # 50 Hz 发布 map→odom TF,默认 5 Hz 对 Nav2 偏慢
    map_update_interval: 5.0         # 每 5 秒向 /map 话题推一次最新地图
    transform_timeout: 0.2
    tf_buffer_duration: 30.0

    # ---- 地图参数 ----
    resolution: ==0.05==             # 5 cm / 像素,室内场景够用;调小地图更精细但内存吃得多

    # ---- 建图触发条件 ----
    # 机器人移动/转动超过阈值才触发一次新的关键帧,避免同一位置刷帧
    minimum_travel_distance: ==0.2== # 移动 0.2 m 才更新
    minimum_travel_heading: ==0.2==  # 转动 0.2 rad(~11.5°)才更新
    scan_buffer_size: 30
    scan_buffer_maximum_scan_distance: 15.0

    # ---- 扫描匹配 ----
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 2.5
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # ---- 回环检测 ----
    do_loop_closing: true            # 开启回环,绕一圈能对齐的话地图质量大大提升
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.6
    loop_match_minimum_response_fine: 0.7
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # ---- 匹配惩罚项(影响精度的关键)----
    distance_variance_penalty: 0.5
    angle_variance_penalty: 2.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

!!! info "关于这套参数是怎么来的"
    上面这份 yaml 不是拍脑袋抄默认值 —— 是我们在 Go2 实机上反复调过的推荐值。主要关注这几组:
    
    - **`transform_publish_period: 0.02`** —— 默认 0.05(20 Hz),Nav2 的 local costmap 更新时经常拿不到最新 TF,调到 50 Hz 之后肉眼可见流畅
    - **`scan_buffer_size: 30` + `scan_buffer_maximum_scan_distance: 15.0`** —— 决定"当前帧能跟多远的历史帧匹配",买断短距离鬼影
    - **`distance_variance_penalty` / `angle_variance_penalty`** —— 越大越"相信里程计",Go2 腿式里程计打滑时容易骗人,这两个值不能调太高
    - **`use_response_expansion: true`** —— 首次匹配失败时扩大搜索空间再试一次,对空旷场景更友好
    
    一句话:想让建图更精细,先调 ==`resolution`== 和 ==`minimum_travel_distance`==;其它项保持默认通常就是最优。

### 步骤六:写一键启动 launch

一次启动涉及的节点挺多,手工敲 7 个终端太痛苦。我们写一个 launch 文件把它们串起来。`go2_slam/launch/mapping.launch.py`:

```python
"""
一键启动建图:驱动 + 传感器处理 + SLAM + RViz(SLAM 专用视图)
"""

import os
from pathlib import Path                              # 处理 launch 文件内的相对路径
from launch import LaunchDescription                  # ROS2 launch 的顶层描述对象
from launch_ros.actions import Node                   # 启动一个 ROS2 节点
from launch.actions import IncludeLaunchDescription   # 嵌套另一个 launch 文件
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 本包的 share 目录,用来定位 config 文件
    slam_share = Path(get_package_share_directory("go2_slam"))
    sensors_share = Path(get_package_share_directory("go2_sensors"))

    slam_params = str(slam_share / "config" / "slam_toolbox_params.yaml")
    scan_params = str(sensors_share / "config" / "pointcloud_to_laserscan_params.yaml")
    rviz_cfg = str(slam_share / "config" / "slam.rviz")

    # 1) Go2 驱动 —— 复用第 6 章的包,提供 odom/TF
    #    ⚠ 明确关掉 driver 自带的 rviz,否则会和下面第 5 步的 slam rviz 打架(起两个窗口)
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("go2_driver_py"))
                / "launch" / "driver.launch.py")
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    # 2) 时间戳修复 —— 第 11 章新增
    timestamp_fix = Node(
        package="go2_sensors",
        executable="pointcloud_timestamp_fix",
        name="pointcloud_timestamp_fix",
        output="screen",
    )

    # 3) 3D 点云 → 2D 扫描
    pc_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        parameters=[scan_params],
        remappings=[
            ("cloud_in", "/utlidar/cloud_fixed"),     # 订时间戳修复后的点云
            ("scan", "/scan"),                         # 输出标准 /scan 话题
        ],
        output="screen",
    )

    # 4) SLAM Toolbox 本体
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params],
        output="screen",
    )

    # 5) RViz —— 加载 SLAM 专用配置(Fixed Frame = map)
    #    如果 slam.rviz 还没保存,不带 -d,免得启动失败
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg] if os.path.exists(rviz_cfg) else [],
        output="screen",
    )

    return LaunchDescription([driver, timestamp_fix, pc_to_scan, slam, rviz])
```

!!! warning "为什么要显式关掉 driver 的 rviz"
    第 6 章的 `driver.launch.py` 默认会启动一个 RViz(加载 `display.rviz`,Fixed Frame=`odom`),用来看机器人模型和 TF。到了第 11 章,我们要看的是**建图视角**(Fixed Frame=`map`,带 Map 图层),两个 rviz 同时起会把屏幕占满还误导读者 "建图没在工作"。所以这里强制 `use_rviz: "false"`,统一由下面第 5 步起一个专用的 slam rviz。

### 步骤七:准备 RViz 配置

最后在 `go2_slam/config/slam.rviz` 保存一个专用 RViz 布局:

- **Fixed Frame**:`map`(建图时要站在地图视角看机器人移动)
- **显示项**:Map + LaserScan(颜色选绿、Reliability 选 `Best Effort`) + RobotModel + TF
- **视角**:`TopDownOrtho`(俯视正交,2D 地图看得最清)

!!! tip "RViz 配置文件怎么来"
    实际操作:在 RViz 里手动布好界面 → **File → Save Config As** → 存到 `go2_slam/config/slam.rviz`。之后每次 launch 自动加载。

---

## 编译与运行

### 编译

```bash
# 回到工作空间根目录
cd ~/unitree_go2_ws

# 只编译本章新增的两个包,节省时间
colcon build --packages-select go2_sensors go2_slam

# 加载编译产物到当前终端
source install/setup.bash
```

### 启动建图

```bash
# 一条命令,所有相关节点一起上
ros2 launch go2_slam mapping.launch.py
```

启动后你会看到:

1. 终端刷出 5 个节点的启动日志
2. RViz 弹出,左上角 Fixed Frame 是 `map`
3. 几秒后,机器人模型出现,周围开始画出 2D 地图雏形

### 操控机器人建图

Go2 自带的**手柄本身就能走**,建图过程中直接用手柄遛狗即可,这一步不是必需的。

如果你想用键盘代替手柄(比如手柄不在手边),**新开一个终端**启动第 3 章的键盘节点:

```bash
# 可选:仅当不想用手柄时启动
source ~/unitree_go2_ws/install/setup.bash
ros2 run go2_teleop_ctrl_keyboard go2_teleop_ctrl_keyboard
```

!!! tip "键盘只是额外输入源"
    键盘节点和手柄底层都通过狗的运动控制接口下发速度,开或不开都不影响 slam_toolbox 建图。用哪个顺手就用哪个。

建图小技巧:

- 🐢 **慢走** —— 每秒 0.2 m 最稳;快走会让扫描匹配失败,地图出现扭曲
- 🔄 **尽量绕圈回起点** —— 触发回环检测,地图一次性对齐
- 🧱 **特征丰富的路径优先** —— 空旷走廊建图质量差,因为扫描匹配找不到参照
- 👀 **实时看 RViz** —— 发现地图歪了/重影了,马上停,排查问题比硬建强

### 保存地图

!!! danger "千万别先 Ctrl-C"
    `slam_toolbox` 节点一关,`/map` 话题就跟着消失,再调保存服务只会得到空地图。正确顺序:
    
    1. **第一个终端保持建图状态不动**(不要 Ctrl-C)
    2. 新开一个终端运行保存命令
    3. 看到 `Map saved successfully` 再回第一个终端 Ctrl-C 结束建图

**新开一个终端**:

```bash
# ⚠ 下面这条路径替换成你自己的工作空间,本书后续以 ==~/unitree_go2_ws/maps== 为例
mkdir -p ==~/unitree_go2_ws/maps==

# 别忘了 source,新终端默认没加载工作空间
source ==~/unitree_go2_ws==/install/setup.bash

# 保存为标准 pgm + yaml 格式,给下一章 Nav2 用
ros2 run nav2_map_server map_saver_cli \
    -f ==~/unitree_go2_ws/maps==/my_map
```

你会得到两个文件:

- `my_map.pgm` —— 地图图像(黑=障碍、白=空地、灰=未知)
- `my_map.yaml` —— 元数据(分辨率、原点、阈值)

如果希望**下次继续在这张地图上补建**,用 SLAM Toolbox 的序列化功能:

```bash
# 序列化保存,包含位姿图信息,可恢复
# ⚠ filename 替换成你自己的工作空间绝对路径
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '==/home/$USER/unitree_go2_ws/maps==/my_map'}"
```

---

## 结果验证

![TODO:RViz 建图中的截图,左侧显示 Displays 面板,主视图显示绿色激光扫描和白色栅格地图,机器人模型在地图中央](../assets/images/11-slam-rviz-mapping.png){ width="640" }

### 命令行检查

打开另一个终端,依次跑:

```bash
# 1) /scan 话题应该有数据,类型是 LaserScan
ros2 topic echo /scan --no-arr --once

# 2) /map 话题应该在推送 OccupancyGrid
ros2 topic hz /map

# 3) TF 树应该是连通的: map → odom → base → 各关节
ros2 run tf2_tools view_frames
# 会在当前目录生成 frames.pdf,用 PDF 阅读器打开检查
```

**正常标志**:

- `/scan` 有稳定的 `ranges` 数组输出(~723 个数值)
- `/map` 话题频率在 0.2 Hz 左右(5 秒一帧,与 `map_update_interval` 对应)
- `frames.pdf` 里能看到从 `map` 一路连到 `base`,中间无断裂

### 地图质量自检

建图结束后打开 `my_map.pgm`:

- ✅ **好地图**:墙壁连续、拐角清晰、房间轮廓闭合
- ❌ **差地图的常见症状**:
  - 一堵墙画成两条平行线 → 回环检测没触发,或者走太快
  - 整张图歪斜 → 里程计漂移严重,需要 IMU 融合(进阶话题,本书暂不展开)
  - 地板被画成障碍 → `min_height` 设太低

---

## 常见问题

### Q1: 启动后 `/scan` 话题没数据

**排查顺序**:

```bash
# 1. 检查原始点云是否在发
ros2 topic hz /utlidar/cloud_deskewed
# 没输出 → Go2 雷达驱动没起;检查 unitree_ros2 setup.sh 有没 source

# 2. 检查时间戳修复节点是否在发
ros2 topic hz /utlidar/cloud_fixed
# 没输出 → pointcloud_timestamp_fix 没启,看 launch 日志

# 3. 检查 pointcloud_to_laserscan 节点
ros2 node list | grep pointcloud_to_laserscan
# 没有 → launch 里的节点配置有误
```

### Q2: 地图出现"鬼影"(同一墙壁画在多个位置)

**典型原因**:机器人被抱起/猛推/滑了一下,里程计瞬间跳变,SLAM 没跟上。

**应对**:

- 建图时**不要强行抬起或推机器人**
- 若已经出现鬼影,可以用 SLAM Toolbox 的手动清除服务:
  ```bash
  ros2 service call /slam_toolbox/clear_changes std_srvs/srv/Empty "{}"
  ```
- 终极方案:IMU 融合(用 `robot_localization` 做 EKF),不在本章范围,留作进阶

### Q3: RViz 报 "Transform data too old" 或 "Lookup would require extrapolation"

**原因**:TF 时间戳和查询时间对不上,常见于时间戳修复节点没起来。

**快速判断**:

```bash
# 看看 cloud_fixed 的 header.stamp 是不是接近系统时间
ros2 topic echo /utlidar/cloud_fixed --no-arr --once
# 对比 `date +%s.%N` 的输出,差值应该在 1 秒内
```

如果差值很大(几百秒),说明你订到的是没修过的 `cloud_deskewed`,检查 launch 里 `remappings` 的配置。

### Q4: SLAM Toolbox 收不到 `/scan`

**原因 9 成是 QoS 不匹配**。SLAM Toolbox 默认用 RELIABLE 订阅,而我们的 `/scan` 是 BEST_EFFORT 发布。

**验证**:

```bash
# -v 展开显示 QoS 信息
ros2 topic info /scan -v
# 找 "Reliability: BEST_EFFORT" 和 "Reliability: RELIABLE" 的发订者
# 发布 BEST_EFFORT + 订阅 RELIABLE = 兼容不上
```

**修复**:确认 `slam_toolbox_params.yaml` 里有 `qos_overrides` 段,并重启 SLAM 节点。

### Q5: RViz 里看不到地图,但终端里 slam_toolbox 节点明明起来了

这是最容易被误判的一类问题 —— 别急着以为"建图挂了"。先分两步定位:

**第一步:确认建图链路真的在工作**(跟 RViz 完全无关)

```bash
# /map 话题应该在持续推送
ros2 topic hz /map
# 看到 ~0.2 Hz 就是在建图,只是你 RViz 没显示而已

# TF map → odom 应该存在
ros2 run tf2_ros tf2_echo map odom
# 能持续打印平移/旋转 = SLAM 输出正常
```

两个都通 → 建图链路没问题,接着看第二步。

**第二步:RViz 配置问题**

大概率是这三种情况之一:

1. **Fixed Frame 没改成 `map`** —— RViz 左上角 Global Options,默认值常是 `odom` 或 `base`,不是 `map` 就看不到地图图层
2. **没有 Map Display** —— 左下 Add → By topic → `/map` → Map
3. **`slam.rviz` 还没保存过** —— 教材 `mapping.launch.py` 在文件不存在时会起一个**空 rviz**,需要你自己加 Map、LaserScan、RobotModel,然后 File → Save Config As 存到 `~/unitree_go2_ws/install/go2_slam/share/go2_slam/config/slam.rviz`(或 src 目录下对应位置然后重新 `colcon build`)。下次启动就自动加载了

!!! tip "RViz 没显示地图 ≠ SLAM 没工作"
    这是初学者**最容易掉的坑**。终端上每一行"slam_toolbox … started"看似一切正常,但 RViz 是另一个独立程序,它的配置文件只管显示、不管建图本身。分清这两层再排查,能省一小时。

### Q6: 机器人不动 / 动了但 /odom 不变

检查第 6 章的 `go2_driver_py` 有没有起来:

```bash
ros2 node list | grep driver
ros2 topic hz /odom
```

没起就单独 launch 驱动,再看一遍本章步骤。

---

## 本章小结

- SLAM = **前端(扫描匹配) + 后端(图优化) + 回环检测**,让机器人边走边画边修正
- Go2 的 3D 点云需要**过滤高度 + 压扁成 2D 扫描**才能喂给 SLAM Toolbox
- 实战中踩了三个典型坑:**时间戳延迟、QoS 不对齐、TF 要用 `base` 不是 `base_link`** —— 记住这三点能省一天调试
- 建完地图用 `nav2_map_server` 保存成 pgm + yaml,下一章 Nav2 要加载它

---

## 下一步

本章的地图只是"看着像样"。真正的考验是 —— **让 Go2 加载这张地图、自己找到起点、然后自主走到你指定的目标**。那就是 [第 12 章 Nav2 基线导航](12-nav2-baseline.md) 的任务。

第 12 章我们会先绕个小弯,对比 3D SLAM 的几个主流方案(KISS-ICP / FAST-LIO / Point-LIO),给想进一步折腾的同学一条路径。只想把 Go2 用起来的同学可以跳过 12 章直奔 13 章。

---

## 拓展阅读

- **SLAM Toolbox 官方仓库** —— <https://github.com/SteveMacenski/slam_toolbox>
- **`pointcloud_to_laserscan` 官方包** —— <https://github.com/ros-perception/pointcloud_to_laserscan>
- **《概率机器人》(Probabilistic Robotics)** —— Thrun 等著,想啃算法底层的必读
- **ROS2 TF2 教程** —— <https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html>
- **激光 SLAM 综述(中文)** —— 《视觉 SLAM 十四讲》高翔,虽然主讲视觉 SLAM,但前端后端那套通用
