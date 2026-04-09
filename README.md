# rbs_manus_xhand_teleop

Manus 手套 → GeoRT retargeting → XHand 灵巧手 实时遥操作管线。

基于 [GeoRT (Meta Research)](https://github.com/facebookresearch/GeoRT) 的几何 retargeting 算法，将 Manus 手套采集的人手骨骼数据实时映射到 XHand v1.3 右手的 12-DOF 关节角度。

## 架构

```
Manus Glove (USB Dongle)
    │  Integrated SDK / ROS2
    ▼
manus_right  (ROS2 C++ node)   ← src/manus_bridge.py (ZMQ/Sim 也可)
    │  /manus_quats topics
    ▼
run_mocap_core.py  (FK → ZMQ broadcast, port 8765)
    │  (21, 3) float32 keypoints
    ▼
GeoRT IK model  (xhand_v1)
    │  (12,) joint angles [rad]
    ▼
XHand Right Hand              ← src/xhand_controller.py
```

## 项目结构

```
rbs_manus_xhand_teleop/
├── teleop.py               # 端到端遥操作主脚本
├── train_xhand.py          # GeoRT retargeting 训练入口
├── run_mocap_core.py       # Manus → FK → ZMQ 广播
├── config/
│   └── xhand_right.json   # XHand 12-DOF GeoRT 配置
├── assets/xhand_right/
│   ├── xhand_right_nocol.urdf   # 无碰撞 URDF（SAPIEN 3 兼容）
│   └── meshes/            # 30 个 STL 文件
├── checkpoints/
│   ├── fk_model_xhand_right.pth
│   └── xhand_right_*/
│       ├── last.pth        # 训练好的 IK 模型
│       └── config.json
├── src/
│   ├── sapien_compat.py   # SAPIEN 2 API → SAPIEN 3 兼容层
│   ├── manus_bridge.py    # Manus 数据源（ZMQ / SDK / 模拟）
│   ├── xhand_controller.py
│   └── retargeting.py
├── scripts/
│   ├── prepare_xhand_urdf.py  # URDF 处理工具
│   ├── 02_train.sh
│   └── 03_teleop.sh
└── third_party/
    └── GeoRT -> symlink   # 需手动建立
```

## 安装

### 0. 系统依赖

```bash
# Ubuntu 22.04，ROS2 Humble（Manus ZMQ 模式需要）
sudo apt install libc-ares-dev

# uv（Python 包管理）
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.local/bin/env
```

### 1. 克隆项目 + 链接 GeoRT

```bash
git clone https://github.com/lmz-rbs/rbs_manus_xhand_teleop.git
cd rbs_manus_xhand_teleop

# GeoRT 以 symlink 引入（不作为子模块）
git clone https://github.com/facebookresearch/GeoRT.git third_party/GeoRT
```

### 2. 创建虚拟环境

```bash
uv venv --python 3.10
source .venv/bin/activate
```

### 3. 安装 Python 依赖

```bash
# PyTorch（CUDA 11.8，适配 RTX 4090）
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# 核心依赖（numpy<2 为 ROS2 Humble pinocchio 要求）
uv pip install "numpy<2" scipy pyzmq tqdm trimesh open3d sapien

# GeoRT（editable install）
uv pip install -e third_party/GeoRT
```

### 4. 安装 XHand SDK

```bash
# wheel 在 xhand_control_sdk_py 目录，cp310 对应 Python 3.10
uv pip install /path/to/xhand_controller-*cp310*.whl
```

### 5. 编译 Manus ROS2 客户端（Integrated SDK 模式）

> 如果只用 ZMQ 模拟模式，跳过此步骤。

```bash
# 替换 manus_client 里的旧版 SDK（需要 Manus SDK 新版文件）
cp /path/to/ManusSDK/lib/libManusSDK_Integrated.so \
   third_party/GeoRT/geort/mocap/manus_client/lib/libManusSDK.so
cp /path/to/ManusSDK/include/*.h \
   third_party/GeoRT/geort/mocap/manus_client/include/

# 编译
source /opt/ros/humble/setup.bash
cd third_party/GeoRT/geort/mocap
colcon build --base-paths manus_client --symlink-install
cd -
```

### 6. 验证安装

```bash
python test_project.py
# 预期输出：=== ALL TESTS PASSED ✓ ===
```

## 使用

### SOP 1：模拟测试（无需硬件）

```bash
source .venv/bin/activate
python teleop.py --mocap sim --hand sim
```

### SOP 2：Manus 手套 + 模拟手

```bash
# Terminal 1：启动 Manus 数据采集
source /opt/ros/humble/setup.bash
source third_party/GeoRT/geort/mocap/install/setup.bash
LD_LIBRARY_PATH=third_party/GeoRT/geort/mocap/manus_client/lib \
  ros2 run manus_client manus_right

# Terminal 2：FK 解算 + ZMQ 广播
source .venv/bin/activate
source /opt/ros/humble/setup.bash
python run_mocap_core.py

# Terminal 3：遥操作
source .venv/bin/activate
python teleop.py --mocap zmq --ckpt xhand_v1 --hand sim
```

### SOP 3：完整硬件（Manus + XHand 实物）

```bash
# USB 权限（每次插拔后需要）
sudo chmod 666 /dev/hidraw* /dev/bus/usb/003/009

# Terminal 1 + 2：同 SOP 2

# Terminal 3：
source .venv/bin/activate
sudo chmod 666 /dev/ttyUSB0
python teleop.py --mocap zmq --ckpt xhand_v1 --hand real --hand-port /dev/ttyUSB0
```

## XHand 关节顺序（12-DOF）

| # | 关节 | 范围 (rad) |
|---|------|-----------|
| 0 | right_hand_thumb_bend_joint | 0 ~ 1.832 |
| 1 | right_hand_thumb_rota_joint1 | -0.698 ~ 1.57 |
| 2 | right_hand_thumb_rota_joint2 | 0 ~ 1.57 |
| 3 | right_hand_index_bend_joint | -0.174 ~ 0.174 |
| 4 | right_hand_index_joint1 | 0 ~ 1.919 |
| 5 | right_hand_index_joint2 | 0 ~ 1.919 |
| 6 | right_hand_mid_joint1 | 0 ~ 1.919 |
| 7 | right_hand_mid_joint2 | 0 ~ 1.919 |
| 8 | right_hand_ring_joint1 | 0 ~ 1.919 |
| 9 | right_hand_ring_joint2 | 0 ~ 1.919 |
| 10 | right_hand_pinky_joint1 | 0 ~ 1.919 |
| 11 | right_hand_pinky_joint2 | 0 ~ 1.919 |

## 已知问题

| 问题 | 解决方案 |
|------|---------|
| SAPIEN segfault（headless） | 使用 SAPIEN 3.x + `src/sapien_compat.py` |
| URDF inertia 报错 | 已修复（ixx/iyy/izz ≥ 1e-6） |
| numpy 2 与 pinocchio 不兼容 | 固定 `numpy<2` |
| Manus SDK 连接失败 | 确认使用 `libManusSDK_Integrated.so` + `CoreSdk_InitializeIntegrated()` |

## 依赖版本

| 包 | 版本 |
|----|------|
| Python | 3.10 |
| torch | 2.7.1+cu118 |
| sapien | 3.0.3 |
| numpy | <2 (1.26.x) |
| pyzmq | ≥22.0 |
| GeoRT | editable |

## License

本项目代码 MIT。GeoRT 遵循 [Meta Research LICENSE](https://github.com/facebookresearch/GeoRT/blob/main/LICENSE)。
