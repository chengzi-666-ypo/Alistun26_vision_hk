#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== 继续更新软件环境 (尝试 git clone 加速) ===${NC}"

# --- 1. OpenVINO 检查 ---
echo -e "${GREEN}[1/2] 检查 OpenVINO...${NC}"
if dpkg -l | grep -q "openvino-2025.3.0"; then
    echo -e "${GREEN}OpenVINO 2025.3.0 已安装。${NC}"
else
    echo "OpenVINO 未完全安装，重试安装..."
    # 添加 Intel GPG Key
    wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB -O GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
    sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
    # 添加源
    echo "deb https://apt.repos.intel.com/openvino/2025 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2025.list
    sudo apt-get update
    sudo apt-get install -y openvino-2025.3.0
fi

# --- 2. OpenCV 更新 ---
echo -e "${GREEN}[2/2] 正在处理 OpenCV 4.8.0...${NC}"

# 检查当前版本
CURRENT_OPENCV=$(pkg-config --modversion opencv4 2>/dev/null || echo "none")
if [ "$CURRENT_OPENCV" == "4.8.0" ]; then
    echo -e "${GREEN}OpenCV 4.8.0 已安装，跳过编译。${NC}"
else
    echo "当前 OpenCV 版本: $CURRENT_OPENCV"
    echo "准备编译安装 OpenCV 4.8.0。"
    
    # 安装依赖
    sudo apt-get install -y build-essential cmake git pkg-config libgtk2.0-dev \
        libavcodec-dev libavformat-dev libswscale-dev libjpeg-dev libpng-dev \
        libtiff-dev libgphoto2-dev

    # 创建工作目录
    mkdir -p opencv_build
    cd opencv_build

    # 清理旧文件
    rm -rf opencv opencv_contrib opencv.zip opencv_contrib.zip

    # 使用 git clone 加速
    echo "正在克隆 OpenCV 源码 (使用 gitclone.com 加速)..."
    if git clone https://gitclone.com/github.com/opencv/opencv.git; then
        cd opencv
        echo "切换到 4.8.0 版本..."
        git checkout 4.8.0
        cd ..
    else
        echo -e "${RED}OpenCV 克隆失败。${NC}"
        exit 1
    fi

    echo "正在克隆 OpenCV Contrib 源码 (使用 gitclone.com 加速)..."
    if git clone https://gitclone.com/github.com/opencv/opencv_contrib.git; then
        cd opencv_contrib
        echo "切换到 4.8.0 版本..."
        git checkout 4.8.0
        cd ..
    else
        echo -e "${RED}OpenCV Contrib 克隆失败。${NC}"
        exit 1
    fi

    # 编译
    cd opencv
    mkdir -p build
    cd build
    
    CONTRIB_DIR="../../opencv_contrib/modules"

    echo "配置 CMake (Contrib: $CONTRIB_DIR)..."
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH="$CONTRIB_DIR" \
          -D OPENCV_ENABLE_NONFREE=ON \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF ..

    echo "开始编译 (使用 $(nproc) 个核心)..."
    make -j$(nproc)

    echo "安装..."
    sudo make install
    sudo ldconfig
    
    echo -e "${GREEN}OpenCV 4.8.0 安装完成！${NC}"
fi

echo -e "${GREEN}=== 所有操作完成 ===${NC}"
