# HIKROBOTICS工业相机ROS非官方驱动

海康机器人工业相机ROS驱动包，包含海康MVS-SDK v3.0.1 静态库，开箱即用。

- 支持参数化配置，支持修改帧率、ROI、触发模式、曝光、增益、数字偏移等参数。
- 支持外部触发时无法调用相机自动曝光，驱动程序根据亮度控制曝光时间。
- 本例默认支持`aarch64 和 x86_64`的库

## 安装
安装 MVS 驱动：（选用Linux V3.0.1）：
https://www.hikrobotics.com/cn/machinevision/service/download/?module=0

点击下载后解压，根据系统架构使用dpkg选装不同的deb包
Jetson 可装 aarch64
 
```
# 这里因为海康的BUG，篡改了系统的路径，导致很多问题。请针对不同的系统架构，删除对应的包
sudo rm /opt/MVS/lib/aarch64/libusb-1.0.so.0

# 或者
sudo rm /opt/MVS/lib/32/libusb-1.0.so.0
sudo rm /opt/MVS/lib/64/libusb-1.0.so.0
```

```bash
# 下载并编译本项目
mkdir -p ~/hikrobotics_camera_ws/src
cd ~/hikrobotics_camera_ws
git clone https://github.com/emNavi/hik-camera-ros-driver.git
catkin_make
```

## 使用

```bash
source ./devel/setup.bash
roslaunch hik_camera_driver camera.launch
```

## Params 说明

```yaml
# 相机名称
camera_name: camera
Camera:
  # 相机序列号（留空默认设置为第一个相机）
  serial_number: xxxxxxxxxxx
  # 相机内参文件 package://my_cameras/calibrations/${NAME}.yaml
  cam_info_url: package://hik_camera_driver/config/ost.yaml
  # 帧率
  frame_rate: 60.0

  # ROI区域设置
  Height: 2048
  Width: 3072
  OffsetX: 0
  OffsetY: 0

  # 触发模式
  Trigger: false
  # 触发源
  # 0:Line0
  # 1:Line1
  # 2:Line2
  # 3.Line3
  # 4:Counter0
  # 7:Software
  # 8:FrequencyConverter
  Tigger_line: 2
  # 触发上升沿、下降沿、高电平、低电平等
  # 0:RisingEdge
  # 1:FallingEdge
  # 2:LevelHigh
  # 3:LevelLow
  Trigger_action: 0
  # 触发延时 ≥0.0 ，单位 us
  Trigger_delay: 0.0

  # 自动曝光
  # 0:Off
  # 1:Once
  # 2:Continuous
  Exposure: 2
  # 曝光时间 ≥0.0 ，单位 us
  Exposure_time: 5000
  # 曝光时间上限
  ExposureTimeUp: 10000
  # 曝光时间下限
  ExposureTimeLow: 100
  # 程序控制曝光，仅当自动曝光设为手动时启用
  Exposure_control: true

  # 自动增益
  # 0:Off
  # 1:Once
  # 2:Continuous
  Gain_mode: 2
  # 增益值 ≥0.0 ，单位 dB
  Gain_value: 0.0

  # 数字偏移使能
  Digital_shift_mode: true
  # 数字偏移调节 ≥ 0.0 
  Digital_shift: 3.0
  # 亮度 0 <= L <= 255
  brightneess: 80

  # Gamma 使能
  GammaEnable: true
  # 伽马调节
  Gamma_value: 1.0
  # Gamma 选择
  # 1:User
  # 2:sRGB
  Gamma_selector: 1
```
