# Manus → GeoRT → XHand 遥操作系统

基于 [GeoRT](https://github.com/facebookresearch/GeoRT) 的 Manus 手套到 XHand 灵巧手的实时遥操作管线。

## 系统架构

```
┌─────────────┐     ZMQ (21,3)     ┌─────────────┐     qpos (12,)     ┌──────────────┐
│  Manus 手套  │ ──────────────────→│  GeoRT 模型  │ ──────────────────→│  XHand 灵巧手 │
│  (右手)      │  手部关键点         │  Retargeting │  关节角度           │  (右手, 12DOF) │
└─────────────┘                     └─────────────┘                     └──────────────┘
       │                                   │                                    │
  manus_bridge.py                   retargeting.py                    xhand_controller.py
  (ZMQ/SDK/模拟)                    (训练/推理)                        (硬件/模拟)
```

## 项目结构

```
manus_xhand_teleop/
├── teleop.py                    # 🚀 端到端遥操作主脚本
├── setup.sh                     # 环境安装
├── config/
│   └── xhand_right.json         # XHand GeoRT 配置 (12关节 + 5指尖)
├── assets/
│   └── xhand_right/
│       ├── xhand_right_nocol.urdf  # 无碰撞 URDF (SAPIEN 兼容)
│       └── meshes/                  # STL mesh 文件 (30个)
├── src/
│   ├── manus_bridge.py          # Manus 手套数据桥接层
│   ├── xhand_controller.py      # XHand 控制器包装
│   └── retargeting.py           # GeoRT 训练/推理包装
├── scripts/
│   ├── 01_collect_data.sh       # 步骤1: 采集人手数据
│   ├── 02_train.sh              # 步骤2: 训练 retargeting 模型
│   └── 03_teleop.sh             # 步骤3: 启动遥操作
└── checkpoints/                 # 训练好的模型存放处
```

## 快速开始

### 1. 环境安装

```bash
# 克隆项目
git clone https://github.com/lmz-rbs/rbs_manus_xhand_teleop.git
cd rbs_manus_xhand_teleop

# 链接 GeoRT
git clone https://github.com/facebookresearch/GeoRT.git third_party/GeoRT

# 一键安装 (使用 uv, 自动创建 .venv)
bash setup.sh

# 激活环境
source .venv/bin/activate
```

### 2. 测试模式（无需硬件）

```bash
# 模拟 mocap + 模拟手 (验证管线)
python3 teleop.py --mocap sim --hand sim

# 带 SAPIEN 可视化
python3 teleop.py --mocap sim --hand sim --viz
```

### 3. 完整流程

#### Step 1: 采集人手数据
```bash
# 使用 Manus 手套 (需先启动 manus_mocap_core.py)
bash scripts/01_collect_data.sh zmq human_xhand 30

# 或使用模拟数据测试流程
bash scripts/01_collect_data.sh sim human_xhand_test 10
```

#### Step 2: 训练 Retargeting 模型
```bash
bash scripts/02_train.sh human_xhand xhand_geort_v1 3000
```

#### Step 3: 启动遥操作
```bash
# 模拟测试
bash scripts/03_teleop.sh sim sim xhand_geort_v1

# 真实硬件
bash scripts/03_teleop.sh zmq real xhand_geort_v1 /dev/ttyUSB0
```

## 核心模块说明

### manus_bridge.py - Manus 数据桥接

支持三种数据源模式：

| 模式 | 说明 | 依赖 |
|------|------|------|
| `zmq` | 从 GeoRT `manus_mocap_core.py` 接收 ZMQ 广播 | pyzmq, ROS2 |
| `integrated` | 直接调用 `libManusSDK.so` (Integrated 模式) | libManusSDK.so |
| `sim` | 生成合成手部运动数据（测试用） | 无 |

```python
from src.manus_bridge import ManusGloveBridge

bridge = ManusGloveBridge(mode="zmq", port=8765)
data = bridge.get()  # {"result": np.array(21,3), "status": "recording"}
```

### xhand_controller.py - XHand 控制

```python
from src.xhand_controller import create_controller

# 自动选择硬件/模拟
ctrl = create_controller(sim=False, port="/dev/ttyUSB0")

# 发送关节角度 (12维, 弧度)
ctrl.send_joint_positions(qpos)

# 预设动作
ctrl.send_preset("palm")  # fist, palm, v, ok
```

### retargeting.py - GeoRT Retargeting

```python
from src.retargeting import XHandRetargeter

# 加载训练好的模型
retargeter = XHandRetargeter.load("xhand_geort_v1")

# 推理: (21,3) 人手关键点 → (12,) XHand 关节角度
qpos = retargeter.forward(keypoints)
```

## XHand 关节映射

| 序号 | 关节名称 | 手指 | 类型 | 范围 (rad) |
|------|---------|------|------|-----------|
| 0 | right_hand_thumb_bend_joint | 拇指 | 弯曲 | 0 ~ 1.832 |
| 1 | right_hand_thumb_rota_joint1 | 拇指 | 旋转1 | -0.698 ~ 1.57 |
| 2 | right_hand_thumb_rota_joint2 | 拇指 | 旋转2 | 0 ~ 1.57 |
| 3 | right_hand_index_bend_joint | 食指 | 侧摆 | -0.174 ~ 0.174 |
| 4 | right_hand_index_joint1 | 食指 | 屈伸1 | 0 ~ 1.919 |
| 5 | right_hand_index_joint2 | 食指 | 屈伸2 | 0 ~ 1.919 |
| 6 | right_hand_mid_joint1 | 中指 | 屈伸1 | 0 ~ 1.919 |
| 7 | right_hand_mid_joint2 | 中指 | 屈伸2 | 0 ~ 1.919 |
| 8 | right_hand_ring_joint1 | 无名指 | 屈伸1 | 0 ~ 1.919 |
| 9 | right_hand_ring_joint2 | 无名指 | 屈伸2 | 0 ~ 1.919 |
| 10 | right_hand_pinky_joint1 | 小指 | 屈伸1 | 0 ~ 1.919 |
| 11 | right_hand_pinky_joint2 | 小指 | 屈伸2 | 0 ~ 1.919 |

## GeoRT 坐标系约定

```
XHand base_link (right_hand_link) 坐标系:
  +Y: 掌心 → 拇指方向
  +Z: 掌心 → 中指方向  
  +X: 掌面法线方向

与 GeoRT 约定完全一致 ✓ (无需虚拟 base_link)
```

## 人手关键点编号 (MediaPipe 21点)

```
指尖 ID 映射:
  4  → Thumb tip    → right_hand_thumb_rota_tip
  8  → Index tip    → right_hand_index_rota_tip
  12 → Middle tip   → right_hand_mid_tip
  16 → Ring tip     → right_hand_ring_tip
  20 → Pinky tip    → right_hand_pinky_tip
```

## Manus 手套连接方式

### 方式 A: ROS2 + ZMQ (推荐，已有代码)

```bash
# Terminal 1: 启动 Manus ROS2 客户端
cd /path/to/GeoRT/geort/mocap/manus_client
bash colcon_build.sh
# 按照 manus_client README 启动

# Terminal 2: 启动 ZMQ 桥接
cd /path/to/GeoRT
python3 -m geort.mocap.manus_mocap_core

# Terminal 3: 启动遥操作
cd /path/to/manus_xhand_teleop
python3 teleop.py --mocap zmq --ckpt xhand_geort_v1 --hand real --viz
```

### 方式 B: Integrated SDK (无需 ROS2)

```bash
# 广播模式 (替代 manus_mocap_core.py)
python3 -m src.manus_bridge --mode broadcast --sdk-path /path/to/libManusSDK.so

# 直接模式
python3 teleop.py --mocap integrated --ckpt xhand_geort_v1 --hand real --viz
```

## 依赖

| 包 | 版本 | 用途 |
|----|------|------|
| numpy | >= 1.21 | 数值计算 |
| scipy | >= 1.7 | FK 求解 |
| torch | >= 1.12 | GeoRT 模型 |
| pyzmq | >= 22.0 | Manus 数据传输 |
| sapien | >= 2.2 | 物理仿真可视化 |
| xhand_controller | >= 1.1 | XHand SDK |
| pybind11 | >= 2.10 | C++ 绑定 (可选) |

## 已知限制

1. **ManusSDK ctypes 绑定**: 当前 Integrated 模式的 ctypes 绑定为框架代码，需要 `libManusSDK.so` 的读取权限来完成实现。建议先使用 ROS2+ZMQ 方案。
2. **12-DOF vs 人手**: XHand 仅 12 个自由度（中指/无名指/小指各只有 2 个关节），retargeting 精度受限于机械手本身的运动能力。
3. **碰撞检测**: 无碰撞 URDF 用于 SAPIEN 仿真，实际控制需关注物理自碰撞。

## License

遥操作管线代码自由使用。GeoRT 遵循 Meta 原始 LICENSE。
