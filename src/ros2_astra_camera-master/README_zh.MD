# ros2_astra_camera

适用于奥比中光 3D 相机的 ROS2 驱动

## 安装依赖

### ROS2

● 请参考官方 [ROS 2 安装指南](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

### 其他依赖

● 安装依赖（请注意您的 ROS 发行版）

```bash
# 假设已经 source 了 ROS 环境，下同
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
```

● 安装 libuvc

```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # 刷新链接库
```

## 快速开始

### 使用步骤

● 创建新的 ROS2 工作空间

```bash
mkdir -p ~/ros2_ws/src
```

● 解压并复制 openNISDk_ROS2_xxx.tar.gz 到 ~/ros2_ws/src/
● 安装 libusb 规则

```bash
cd ~/ros2_ws/src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

● 编译

```bash
cd ~/ros2_ws
source /opt/ros/galactic/setup.bash 
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

## 启动相机

● 终端1

```bash
source /opt/ros/galactic/setup.bash 
source ./install/setup.bash 
ros2 launch astra_camera astra.launch.xml
```

● 终端2

```bash
source /opt/ros/galactic/setup.bash 
source ./install/setup.bash 
rviz2
```

● 列出主题/服务/参数（打开新终端）

```bash
source /opt/ros/galactic/setup.bash 
ros2 topic list
ros2 service list 
ros2 param list
```

选择要显示的主题

● 获取相机参数

```bash
ros2 service call /camera/get_camera_info astra_camera_msgs/srv/GetCameraInfo '{}'
```

● 查看相机参数，具体字段含义请参考 ROS 文档 [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)

```bash
ros2 topic echo /camera/depth/camera_info
ros2 topic echo /camera/color/camera_info
```

● 查看设备信息

```bash
ros2 service call /camera/get_device_info astra_camera_msgs/srv/GetDeviceInfo '{}'
```

● 获取 SDK 版本

```bash
ros2 service call /camera/get_sdk_version  astra_camera_msgs/srv/GetString "{}"
```

● 设置/获取（自动）曝光

```bash
 # 自动曝光开关。如需手动设置曝光，请先关闭自动曝光
 ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}' 
 ros2 service call /camera/set_uvc_auto_exposure std_srvs/srv/SetBool '{data: false}'
 ros2 service call /camera/set_ir_auto_exposure std_srvs/srv/SetBool "{data: false}"
    
# 设置曝光值
ros2 service call /camera/set_ir_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
ros2 service call /camera/set_color_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
ros2 service call /camera/set_uvc_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
 # 获取曝光
 ros2 service call /camera/get_ir_exposure astra_camera_msgs/srv/GetInt32 "{}"
 ros2 service call /camera/get_color_exposure astra_camera_msgs/srv/GetInt32 "{}"
 ros2 service call /camera/get_uvc_exposure astra_camera_msgs/srv/GetInt32 "{}"
```

● 设置/获取增益

```bash
# 获取增益
ros2 service call /camera/get_color_gain astra_camera_msgs/srv/GetInt32 '{}' # OpenNI 相机
ros2 service call /camera/get_ir_gain astra_camera_msgs/srv/GetInt32 '{}' # OpenNI 相机
ros2 service call /camera/get_uvc_gain astra_camera_msgs/srv/GetInt32 "{}" # UVC 相机
 # 设置增益
ros2 service call /camera/set_color_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
ros2 service call /camera/set_ir_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
ros2 service call /camera/set_uvc_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
```

● 设置镜像模式

```bash
ros2 service call /camera/set_color_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_ir_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_depth_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_uvc_color_mirror std_srvs/srv/SetBool '{data: true}'
```

● 设置/获取（自动）白平衡

```bash
ros2 service call /camera/get_color_auto_white_balance astra_camera_msgs/srv/GetInt32 '{}' # 返回 0 或 1
ros2 service call /camera/set_color_auto_white_balance std_srvs/srv/SetBool '{data: false}'
```

● 开启/关闭激光

```bash
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}' # 开启
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: false}' # 关闭
```

● 开启/关闭风扇

```bash
 ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: true}' # 开启
 ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: false}' # 关闭
```

● 开启/关闭 LDP

```bash
ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: false}'
```

● 开启/关闭传感器

```bash
ros2 service call  /camera/toggle_ir std_srvs/srv/SetBool "{data: true}"
ros2 service call  /camera/toggle_color std_srvs/srv/SetBool "{data: true}"
ros2 service call  /camera/toggle_depth std_srvs/srv/SetBool "{data: true}"
ros2 service call /camera/toggle_uvc_camera std_srvs/srv/SetBool "{data : true}"
```

## 多相机

● 首先获取相机的序列号，插上相机后运行

```bash
ros2 run astra_camera list_devices_node
```

● 设置参数 `device_num` 为相机数量

● 前往 `ros2_astra_camera/launch/multi_xxx.launch` 并修改序列号。目前不同相机只能通过序列号区分，

```xml
<launch>
    <!-- 唯一的相机名称 -->
    <arg name="camera_name" default="camera"/>
    <!-- 硬件深度配准 -->
    <arg name="3d_sensor" default="astra"/>
    <!-- stereo_s_u3, astrapro, astra -->
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <arg name="camera1_serila_number" default="ADA611300CE"/>
    <arg name="camera2_serila_number" default="sn123456789"/>
    <arg name="device_num" default="2"/>
    <node name="camera" pkg="astra_camera" type="cleanup_shm_node" output="screen"/>
    <include file="$(find-pkg-share astra_camera)/launch/$(arg 3d_sensor).launch.xml">
        <arg name="camera_name" value="$(var camera_name)_$(var camera1_prefix)"/>
        <arg name="serial_number" value="$(var camera1_serila_number)"/>
        <arg name="device_num" value="$(var device_num)"/>
    </include>

    <include file="$(ind-pkg-share astra_camera)/launch/$(var 3d_sensor).launch.xml">
        <arg name="camera_name" value="$(var camera_name)_$(var camera2_prefix)"/>
        <arg name="serial_number" value="$(var camera2_serila_number)"/>
        <arg name="device_num" value="$(var device_num)"/>
    </include>
    <node pkg="tf2_ros" exec="static_transform_publisher" name="camera_tf"
          args="0 0 0 0 0 0 camera01_link camera02_link"/>
</launch>
```

* astra 相机将使用信号量进行进程同步，如果相机启动失败， `/dev/shm` 中可能残留信号量文件，导致下次启动卡住。
  启动前，请运行

```bash
ros2 run astra_camera cleanup_shm_node 
```

清理 `/dev/shm/`。

* 启动

```bash
ros2 launch astra_camera multi_astra.launch.xml
```

## 使用标定的相机参数

* 设置相机信息 url，前往 `xxx.launch`

```xml
<launch>
    <!--...-->
    <arg name="ir_info_url" default="file:///you_ir_camera_calib_path/depth_camera.yaml"/>
    <arg name="color_info_url" default="file:///you_depth_camera_calib_path/rgb_camera.yaml"/>
    <!--...-->
</launch>
```

* 标定文件应如下所示

> 注意相机名称是固定的。彩色相机为 rgb_camera，深度/IR 相机名称为 ir_camera

```yaml
image_width: 640
image_height: 480
# 相机名称是固定的。彩色相机为 rgb_camera，深度/IR 相机名称为 ir_camera
camera_name: rgb_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [517.301, 0, 326.785, 0, 519.291, 244.563, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.41527, 0.31874, -0.00197, 0.00071, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [0.999973, 0.00612598, -0.00406652, -0.00610201, 0.999964, 0.00588094, 0.0041024, -0.00585596, 0.999974 ]
projection_matrix:
  rows: 3
  cols: 4
  data: [517.301, 0, 326.785, -25.3167, 0, 519.291, 244.563, 0.282065, 0, 0, 1, 0.0777703]

```

## 启动参数

* `connection_delay`：重新打开设备的延迟时间（秒）。某些设备初始化需要较长时间（例如 Astra mini），热插拔时立即重新打开设备会导致固件崩溃。
* `enable_point_cloud`：是否启用点云。
* `enable_colored_point_cloud`：是否启用彩色点云。
* `point_cloud_qos`，`[color|depth|ir]_qos`，`[color|depth|ir]_camera_info_qos`：ROS2 消息 QoS，可设置为 `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA`，分别对应 `rmw_qos_profile_system_default`，`rmw_qos_profile_default`，`rmw_qos_profile_parameter_events`，`rmw_qos_profile_services_default`，`rmw_qos_profile_parameters`，`SENSOR_DATA`，不区分大小写。
* `enable_d2c_viewer`：发布 D2C 叠加图像（仅用于测试）。
* `device_num`：设备数量，多相机时需要填写。
* `color_width`，`color_height`，`color_fps`：彩色流分辨率和帧率。
* `ir_width`，`ir_height`，`ir_fps`：IR 流分辨率和帧率。
* `depth_width`，`depth_height`，`depth_fps`：深度流分辨率和帧率。
* `enable_color`：是否启用 RGB 相机，当 RGB 相机为 UVC 协议时此参数无效。
* `enable_depth`：是否启用深度相机。
* `enable_ir`：是否启用 IR 相机。
* `depth_registration`：启用硬件深度到彩色对齐，需要开启 RGB 点云。
* `depth_scale`：深度图像缩放比例，例如设为 2 表示将深度 320x240 对齐到 RGB 640x480。
* `color_roi_x`，`color_roi_y`，`color_roi_width`，`color_roi_height`：是否裁剪 RGB 图像，默认值为 -1，仅在 RGB 分辨率大于深度分辨率且需要对按时使用。例如，需要将深度 640x400 对齐到 RGB 640x480，则需要设置 color_roi_x: 0，color_roi_y: 0，color_roi_width: 640，color_roi_height: 400，这将裁剪 RGB 的前 400 像素，并对应深度 ROI。
* `color_depth_synchronization`：启用 RGB 与深度同步。
* `use_uvc_camera`：如果 RGB 相机是 UVC 协议，设置为 true，UVC 协议目前包括 dabai、dabai_dcw 等。
* `uvc_product_id`：UVC 相机的 PID。
* `uvc_camera_format`：UVC 相机的图像格式。
* `uvc_retry_count`：有时 UVC 协议相机热插拔时无法成功重连，需要多次重试。
* `enable_publish_extrinsic`：启用发布相机外参。
* `oni_log_level`：OpenNI 的日志级别（verbose/info/warning/error/none）。
* `oni_log_to_console`：是否将 OpenNI 日志输出到控制台。
* `oni_log_to_file`：是否将 OpenNI 日志输出到文件，默认会保存在当前运行程序路径下的 Log 文件夹中。

## DDS 调优

默认的 DDS 设置（Galactic）可能对数据传输效率不高。不同的 DDS 设置会有不同的性能。这里以 CycloneDDS 为例。详细信息请参考 [ROS DDS 调优](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)。

● 编辑 cyclonedds 配置文件

```bash
sudo gedit /etc/cyclonedds/config.xml
```

添加

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/confighttps://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Internal>
            <MinimumSocketReceiveBufferSize>16MB</MinimumSocketReceiveBufferSize>
        </Internal>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="localhost"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

● 设置环境变量，添加到 .zshrc 或 .bashrc 中

```bash
export ROS_DOMAIN_ID=42 # 数字范围 0 到 232
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

提示：要了解为什么最大 ROS_DOMAIN_ID 是 232，请访问 [ROS DOMAIN ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)

● 增加 UDP 接收缓冲区大小
编辑

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

添加

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```

## 常见问题

● Rviz2 上不显示点云或图像

* 默认发布的消息 QoS 为 sensor_data，即可靠性策略 = Best Effort，请检查 Rviz2 上是否将可靠性策略设置为 Best Effort，同时通过 `ros2 topic info -v /xxx/topic` 检查消息的 QoS 设置是否有问题。

● 多相机无图像

* 可能是供电不足，不要将所有相机连接到同一个集线器，建议使用带电源的集线器。
* 也可能是分辨率过高，尝试调低分辨率。
* 如果彩色相机是 UVC 相机，UVC 的序列号可能未设置。

## 许可证

版权所有 2023 奥比中光有限公司。

根据 Apache 许可证 2.0 版（“许可证”）授权；您可以在遵守许可证的前提下使用本项目。您可以在以下地址获取许可证副本：

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

除非适用法律要求或书面同意，按许可证分发软件是按“原样”分发的，不附带任何明示或暗示的保证或条件。有关许可证下的特定语言管理权限和限制，请参阅许可证。

**其他名称和品牌可能被视为他人财产**