# hik_camera_driver

一个面向海康机器人/Hikrobot MVS 工业相机的 ROS 2 驱动。每个节点管理一台相机，支持
USB3、GigE、序列号选机、断线重连、运行时曝光/增益调整、软件触发、相机内参以及标准
ROS diagnostics。

## 设计特点

- MVS 句柄、取流和帧缓存由 RAII 管理，所有关键 SDK 返回值均会检查。
- 没有相机时不阻塞 ROS executor；相机上线或拔插后自动连接。
- 图像转换前按实际帧尺寸分配缓冲区，转换失败的帧不会发布。
- 一台相机对应一个节点；namespace、序列号、`frame_id` 和内参一一绑定。
- 内参分辨率与图像分辨率不一致时，发布未标定的 `CameraInfo` 并报告诊断告警，避免静默使用错误内参。

## 前置条件与构建

目标平台是装有 ROS 2 的 Linux x86_64 或 aarch64。仓库的 `third_party/MVS` 已包含构建
所需的官方头文件，以及 amd64、arm64 两种架构的运行库，因此默认不需要安装 MVS SDK 或
配置 `HIK_MVS_ROOT`：

```bash
colcon build --packages-select hik_camera_driver --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DHIK_MVS_INSTALL_RUNTIME=ON
source install/setup.bash
```

CMake 依次查找显式的 `HIK_MVS_ROOT`、环境变量、仓库内的 `third_party/MVS` 和
`/opt/MVS`。默认会把当前架构目录中的厂商 `.so` 安装到 ROS 包的 `lib`，所以执行
`source install/setup.bash` 后无需手动设置 `LD_LIBRARY_PATH`。如需强制使用系统安装的
新版 SDK，可以设置 `HIK_MVS_ROOT=/opt/MVS`；如不希望安装厂商运行库，可以使用
`-DHIK_MVS_INSTALL_RUNTIME=OFF`。

## 单相机启动

不使用 namespace 时，核心话题是 `/image_raw` 和 `/camera_info`：

```bash
ros2 launch hik_camera_driver hik_camera.launch.py \
  serial_number:=DA123456 \
  camera_name:=front_camera \
  frame_id:=front_camera_optical_frame \
  camera_info_url:=file:///home/robot/calibration/front_camera.yaml
```

增加 `camera_namespace:=front_camera` 后，话题变为：

```text
/front_camera/image_raw       sensor_msgs/msg/Image
/front_camera/camera_info     sensor_msgs/msg/CameraInfo
```

驱动还向标准 `/diagnostics` 发布连接状态、实时 FPS、帧超时、SDK 错误、丢包和重连次数。
安装 `image_transport_plugins` 后，可按需订阅 `image_raw/compressed` 等传输话题。

## 每台相机的内参

内参使用 `camera_info_manager` 的标准 YAML 格式。建议按用途或序列号命名：

```text
calibration/
├── front_DA123456.yaml
└── rear_DA654321.yaml
```

可以从任意绝对路径加载：

```text
file:///home/robot/calibration/front_DA123456.yaml
```

也可以把文件放进本包的 `config/calibration`，使用：

```text
package://hik_camera_driver/config/calibration/front_DA123456.yaml
```

内参文件中的 `camera_name` 应与节点的 `camera_name` 参数一致；`image_width` 和
`image_height` 必须与实际发布分辨率一致。`config/calibration/example_camera.yaml` 只是未
标定的格式示例，不能作为真实内参使用。

## 配置文件的职责

- `config/camera_params.yaml` 是单相机节点的通用默认参数，由 `hik_camera.launch.py` 的
  `params_file` 加载。它适合单机调试，也可以用自己的文件覆盖：
  `params_file:=/path/to/camera_params.yaml`。
- `config/cameras.yaml` 是 `multi_camera.launch.py` 的多相机清单，负责每个节点的启停、
  namespace、选机条件和参数覆盖；它不会再读取 `camera_params.yaml`。
- `config/calibration/*.yaml` 保存相机内参，由各相机的 `camera_info_url` 单独引用。

`hik_camera.launch.py` 适合快速验证一台相机；`multi_camera.launch.py` 同时支持一台、两台
或更多相机，更适合作为项目的统一启动入口。驱动仓库自带的配置是默认值和示例，不建议
在作为子模块使用时持续写入具体设备信息。

## 任意数量相机启动

复制并编辑 `config/cameras.yaml`。`defaults` 是所有相机共用的参数，`cameras` 列表中每
增加一项就会创建一个相机节点：

```yaml
defaults:
  transport: usb
  output_encoding: rgb8
  use_sensor_data_qos: true
  auto_exposure: false
  exposure_time: 6000.0

cameras:
  - namespace: front_camera
    serial_number: "DA123456"
    camera_name: front_camera
    frame_id: front_camera_optical_frame
    camera_info_url: "calibration/front_DA123456.yaml"

  - namespace: rear_camera
    serial_number: "DA654321"
    camera_name: rear_camera
    frame_id: rear_camera_optical_frame
    camera_info_url: "calibration/rear_DA654321.yaml"

  - namespace: left_camera
    serial_number: "DA999999"
    camera_name: left_camera
    frame_id: left_camera_optical_frame
    camera_info_url: "calibration/left_DA999999.yaml"
```

启动安装在包内的配置：

```bash
ros2 launch hik_camera_driver multi_camera.launch.py
```

也可以使用机器人自己的配置文件，无需修改或重新构建驱动包：

```bash
ros2 launch hik_camera_driver multi_camera.launch.py \
  cameras_file:=/home/robot/config/hik_cameras.yaml
```

`multi_camera.launch.py` 没有设置两台相机的上限。它会在创建节点前拒绝重复 namespace、
重复序列号、缺失选机条件、错误参数类型和未知参数名。需要临时停用某台相机时，在对应项
中添加 `enabled: false`。

`camera_info_url` 可以直接写相对路径。相对路径始终以 `cameras.yaml` 所在目录为基准，
与执行 `ros2 launch` 时的工作目录无关。例如配置文件位于
`/home/robot/config/cameras.yaml` 时，`calibration/front.yaml` 会解析成
`file:///home/robot/config/calibration/front.yaml`。绝对路径、`file://` 和 `package://`
URL 也继续支持。

示例 `config/cameras.yaml` 中的相机默认处于禁用状态，必须先替换序列号和内参 URL，再将
`enabled` 改成 `true` 或删掉该字段。

## 作为 Git 子模块集成

建议把本仓库作为只负责通用驱动能力的子模块，把机器人或项目专用配置放在主仓库自己的
bringup 包中。相机序列号、namespace、`frame_id`、曝光/增益以及标定文件都不需要提交到
驱动子模块。

推荐的工作空间结构如下：

```text
pnx_robot/
└── src/
    ├── hik_camera_driver/          # Git 子模块
    └── pnx_camera_bringup/         # 主项目维护
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── cameras.launch.py
        └── config/
            ├── cameras.yaml
            └── calibration/
                ├── front_DA123456.yaml
                └── rear_DA654321.yaml
```

`pnx_camera_bringup/package.xml` 需要声明构建工具和运行依赖：

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<exec_depend>hik_camera_driver</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

`pnx_camera_bringup/CMakeLists.txt` 安装 launch 和配置目录：

```cmake
cmake_minimum_required(VERSION 3.8)
project(pnx_camera_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

主项目的 `launch/cameras.launch.py` 只需包含驱动的通用多相机启动文件，并传入外部配置：

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_launch = PathJoinSubstitution([
        FindPackageShare("hik_camera_driver"),
        "launch",
        "multi_camera.launch.py",
    ])
    cameras_file = PathJoinSubstitution([
        FindPackageShare("pnx_camera_bringup"),
        "config",
        "cameras.yaml",
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={"cameras_file": cameras_file}.items(),
        )
    ])
```

构建并 source 工作空间后，从主项目启动：

```bash
ros2 launch pnx_camera_bringup cameras.launch.py
```

这样新增或删除相机、替换序列号和内参、修改 namespace、`frame_id`、曝光、增益或帧率时，
只需修改 `pnx_camera_bringup/config`，不需要修改或提交子模块。运行时的
`ros2 param set` 适合临时调试；确认后的稳定参数应写回主项目的 `cameras.yaml`。

只有在需要新增驱动参数或接口、修复驱动问题、升级 MVS SDK，或者适配新的 ROS 版本时，
才需要修改并更新 `hik_camera_driver` 子模块提交。生产环境也可以把 `cameras.yaml` 和标定
文件放在 `/etc/pnx/...` 等部署目录，并通过 `cameras_file:=绝对路径` 传入，从而让设备配置
完全独立于两个源码仓库。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `transport` | `usb` | `usb`、`gige` 或 `any` |
| `serial_number` | 空 | 按序列号选机；多相机时必须设置 |
| `user_defined_name` | 空 | 也可按 MVS 用户名称选机 |
| `camera_name` | `camera` | 标定名称 |
| `camera_info_url` | 空 | 内参 URL；多相机配置还支持相对或绝对路径 |
| `frame_id` | `camera_optical_frame` | 图像与内参的 TF frame |
| `output_encoding` | `rgb8` | `mono8`、`rgb8` 或 `bgr8` |
| `auto_exposure` | `false` | 连续自动曝光 |
| `exposure_time` | `6000.0` | 手动曝光时间，微秒 |
| `auto_gain` | `false` | 连续自动增益 |
| `gain` | `0.0` | 手动增益 |
| `frame_rate` | `0.0` | 大于 0 时启用相机帧率控制 |
| `trigger_mode` | `off` | `off` 或 `software` |
| `grab_timeout_ms` | `1000` | 单帧等待超时 |
| `reconnect_interval_ms` | `1000` | 重连间隔 |
| `max_consecutive_timeouts` | `5` | 自由运行模式触发重连的连续超时数 |
| `sdk_buffer_count` | `4` | MVS 内部图像缓存节点数 |
| `use_sensor_data_qos` | `true` | 使用 sensor-data QoS |

曝光、增益、自动曝光、自动增益和帧率支持运行时更新：

```bash
ros2 param set /front_camera/camera exposure_time 8000.0
ros2 param set /front_camera/camera gain 8.0
```

参数写入相机失败时更新会被拒绝，并尝试恢复之前的配置。

## 软件触发

以 `trigger_mode:=software` 启动后调用节点私有服务：

```bash
ros2 service call /front_camera/camera/trigger std_srvs/srv/Trigger "{}"
```

自由运行模式下调用该服务会返回失败，不会改变相机状态。

## 运行检查与排障

```bash
ros2 topic hz /front_camera/image_raw
ros2 topic echo /diagnostics
```

如果找不到相机，请依次检查 MVS 客户端能否取流、厂商 udev 规则、USB3 带宽、SDK 与驱动
版本，以及 `serial_number`。GigE 相机还需检查主机网卡地址、相机 IP 和 MTU。

## 许可证

本包原创代码采用 MIT。`third_party/MVS` 中的 Hikrobot MVS SDK 不属于本许可证，仍由
厂商许可条款约束；提交或分发包含这些文件的仓库前，请确认具有相应权限。SDK 文件清单、
架构和校验值见 `third_party/MVS/README.md` 与 `SHA256SUMS`。
