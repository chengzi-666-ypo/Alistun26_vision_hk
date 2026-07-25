# 广东工商职业技术大学Alistun战队26赛季自瞄算法
    (基于同济大学SuperPower战队25赛季自瞄算法开源二次开发)
# 海康摄像头版
相机型号：海康MV-CS016-10UC
镜头型号：海康官方6mm镜头
# 环境配置

| **操作系统** | Ubuntu 22.04 |
| **GCC** | 11.4.0 | 
| **CMake** | 3.22.1 | 
| **OpenCV** | 4.8.0 | 
| **Eigen3** | 3.4.0 | 
| **fmt** | 8.1.1 | 
| **spdlog** | 1.9.2 | 
| **yaml-cpp** | 0.7.0 | 
| **libusb** | 1.0.25 | 
| **can-utils** | 已安装 | 
| **OpenVINO** | 2025.3.0 | 
| **Ceres** | 2.0.0 |
[HikRobot SDK] https://www.hikrobotics.com/cn/machinevision/service/download/?module=0

其余：

    ```bash
    sudo apt install -y \
        git \
        g++ \
        cmake \
        can-utils \
        libopencv-dev \
        libfmt-dev \
        libeigen3-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        libusb-1.0-0-dev \
        nlohmann-json3-dev \
        openssh-server \
        screen
    ```

*其他具体环境配置看同济大学开源的readme.md *


### 编译 ###
build:（Alistun26_vision_hk路径下）：

```bash
rm -rf build
cmake -B build .
make -C build -j$(nproc)
```



### 文件结构

# 主程序
 src/mt_standard.cpp
# 配置文件
configs/standard_serial.yaml



### 启动 ###

# 视觉编译完开启（在Alistun26_vision路径下）
    pkill -f mt_standard; cd Alistun26_vision_hk/build && make -j4 && ./mt_standard ../configs/standard_serial.yaml

# 已经开启视觉过再次开启指令（在build路径下）：
    ./mt_standard ../configs/standard_serial.yaml


- 查看当前c板串口号：  ls -la /dev/ttyACM*| head -5

*(c板复位后串口号会变动，需重新查看修改,修改路径在:Alistun26_vision/Alistun26_vision_hk/configs/standard_serial.yaml里的serial_port: "/dev/ttyACM0" )*

查看当前电脑usb设备数: lsusb  (查看相机和c板是否已经连接上电脑)


-----------------------------------------------------------------------------------------------------------------------

### 上位机与C板之间的串口通信协议 ###

**注意**：本项目所有数据通过 USB 虚拟串口（如 `/dev/ttyACM0`）传输。

### 一、串口配置

- **波特率**: 115200
- **数据位**: 8
- **停止位**: 1
- **校验位**: None
- **流控**: None

### 二、发送协议（上位机 -> C 板）

- **帧总长**: 14 字节

0xFF + control + shoot +yaw + pitch + reserved + 0x0D

#### Payload 定义 (12 字节)

| Payload 偏移 | 字段 | 类型 | 说明 |
| :--- | :--- | :--- | :--- |
| 0 | **Control** | `uint8` | 1: 接管控制, 0: 不控制 |
| 1 | **Shoot** | `uint8` | 1: 射击, 0: 不射击 |
| 2-5 | **Yaw** | `float` | 目标 Yaw 角度 (弧度制, Little Endian) |
| 6-9 | **Pitch** | `float` | 目标 Pitch 角度 (弧度制, Little Endian) |
| 10-11 | **Reserved** | `uint8[2]` | 保留 (全0) |

*注意：Yaw 和 Pitch 发送的是弧度值（例如 0.5236 rad ≈ 30°），采用 IEEE 754 float 格式。*


### 三、接收协议（C 板 -> 上位机）

- **帧总长**: 14 字节

0xFF + w + x + y + z + Bullet Speed + Mode(uint8) + Enemy Color + 0x0D

#### Payload 定义 (12 字节)

| Payload 偏移 | 字段 | 类型 | 说明 |
| :--- | :--- | :--- | :--- |
| 0-1 | **w** | `int16` | 四元数 W 分量 (Little Endian, 缩放 10000) |
| 2-3 | **x** | `int16` | 四元数 X 分量 (Little Endian, 缩放 10000) |
| 4-5 | **y** | `int16` | 四元数 Y 分量 (Little Endian, 缩放 10000) |
| 6-7 | **z** | `int16` | 四元数 Z 分量 (Little Endian, 缩放 10000) |
| 8-9 | **Bullet Speed** | `int16` | 子弹速度 (m/s * 100) |
| 10 | **Mode** | `uint8` | 0:Idle, 1:AutoAim, 2:SmallBuff, 3:BigBuff, 4:Outpost |
| 11 | **Enemy Color** | `uint8` | 0:Both, 1:Red, 2:Blue |

*注意：四元数使用 int16 传输，实际值 = 原始值 / 10000.0。子弹速度为 int16 类型，实际值 = 原始值 / 100.0。*


### 四、配置项说明 (`configs/*.yaml`)

虽然已禁用 CAN，但在配置文件中仍保留了 ID 配置项以兼容内部逻辑：
- `quaternion_canid`: `0x100` (代码逻辑仅使用低 8 位 `0x00` 进行匹配)
- `bullet_speed_canid`: `0x101` (代码逻辑仅使用低 8 位 `0x01` 进行匹配)
- `serial_port`: 串口设备路径 (如 `/dev/ttyACM0`)


------------------------------------------------------------------------------------------------------------------------

# 相机标定使用:

    使用：calibration\capture.cpp 进行拍照

*两个程序分别拍摄50张图片左右进行标定*

- 自动标定程序：
    calibration/calibrate_camera.cpp (云台动，标定板不动)
    calibration/calibrate_robotworld_handeye.cpp （标定板动，云台不动）

*拍摄的照片会在\assets\img_with_q目录下*


# 开机自启动服务 ###

首次使用时，必须先安装并启用服务：
在Alistun25_vision路径下:

```bash
sudo ./install_alistun_service.sh
```
安装脚本会根据当前用户名和文件夹实际位置，自动创建、启用并启动
`alistun_vision.service`。

安装成功后，使用以下命令管理服务：

## 查看运行状态
    sudo systemctl status alistun_vision.service
## 启动程序
    sudo systemctl start alistun_vision.service
## 停止程序
    sudo systemctl stop alistun_vision.service
## 重启程序
    sudo systemctl restart alistun_vision.service
## 实时查看程序输出日志
    sudo journalctl -u alistun_vision.service -f
## 取消开机自启动并立即停止程序
    sudo systemctl disable --now alistun_vision.service




## 项目成员
黄子成


## 参考文献

[1] 同济大学SuperPower战队25赛季自瞄算法开源(https://github.com/TongjiSuperPower/sp_vision_25/)
