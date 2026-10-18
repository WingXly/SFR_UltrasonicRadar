# 基于 FreeRTOS 与 ROS2 的分布式超声波扫描系统

一个由 STM32 下位机和 PC 上位机组成的超声波雷达扫描系统。

下位机使用 STM32F103C8T6 + FreeRTOS，驱动 HC-SR04、SG90 舵机、OLED 和 JDY-31-SPP 蓝牙模块；上位机提供 Windows 串口版和 ROS2 Lyrics 版两种实现，可接收下位机二进制数据、进行滤波与插值，并绘制极坐标雷达图或在 RViz2 中显示 LaserScan。

---

## 目录结构

```text
UltrasonicRadar/
├── firmware/                    # STM32 下位机工程
│   ├── Core/
│   │   ├── Inc/
│   │   └── Src/
│   ├── Drivers/
│   ├── Middlewares/             # FreeRTOS / CMSIS
│   ├── CMakeLists.txt
│   └── UltrasonicRadar.ioc      # STM32CubeMX 工程
│
├── host_windows/                # Windows 上位机
|   ├── ...
│   ├── MVS_SPP.py               # pyserial + 蓝牙 SPP 版本
│   ├── MVS.py                   # BLE bleak 版本（旧）
│   └── ...
│
├── laser_scan/                  # ROS2 上位机功能包 (主程序)
│   ├── laser_scan/
│   │   ├── stm32.py             # 串口桥接节点
│   │   ├── cmd_controller.py    # CLI 指令解析节点
│   │   ├── visualizer.py        # LaserScan 可视化节点
│   │   └── radar_static_tf.py   # 静态 TF 发布节点
│   ├── package.xml
│   └── setup.py
│
├── my_interfaces/                  # ROS2 上位机功能包 (自定义节点)
│   ├── msg/
│   │   └── PolarCoordinates.msg   # 静态 TF 发布节点
│   ├── package.xml
│   └── CMakeLists.txt
|
├── README.md
└── LICENSE
```

---

## 功能特性

- 上下位机二进制通信协议：帧头、长度、数据、XOR 校验、帧尾。
- 蓝牙 SPP / USB 串口双向通信。
- 下位机基于 FreeRTOS，使用 DMA + USART 空闲中断接收。
- 下位机代码按模块面向对象封装：LED、OLED、定时器、串口、蓝牙。
- 使用函数指针数组管理 MainTask 状态机。
- 支持按键暂停/恢复、LED 状态指示、OLED 开关。
- HC-SR04 + SG90 实现 0~180° 扫描。
- 上位机支持重复采样、均值/中值滤波、角度插值。
- ROS2 版发布 `/radar/sensor_data`、`/radar/laser_scan`，可在 RViz2 中实时显示。
- Windows 版使用 pyserial + Matplotlib/控制台，方便单独测试硬件和算法。

---

## 技术栈

### 下位机

| 项目 | 说明 |
|---|---|
| MCU | STM32F103C8T6 |
| 蓝牙 | JDY-31-SPP |
| OLED | CH1116，I2C |
| 舵机 | SG90 |
| 超声波 | HC-SR04 |
| 开发 | STM32CubeMX + CMake + arm-none-eabi-gcc |
| RTOS | FreeRTOS / CMSIS-RTOS2 |
| 通信 | USART3 + DMA + IDLE 中断 |
| PWM | TIM4 CH3 |
| 输入捕获 | TIM1 CH3/CH4 |
| 烧录 | ST-Link V2 |

### 上位机

| 平台 | 说明 |
|---|---|
| Windows | Python 3.10+、pyserial、NumPy、Matplotlib；`host_windows/MVS_SPP.py` |
| ROS2 Lyrics | rclpy、sensor_msgs、std_msgs、geometry_msgs、tf2_ros、RViz2 |
| 可视化 | Matplotlib 极坐标图 / RViz2 LaserScan |
| 滤波 | 均值滤波或中值滤波 |
| 插值 | 线性插值或 PCHIP 平滑插值 |

---

## 硬件连接

### STM32 引脚分配

| STM32 引脚 | 外设 | 说明 |
|---|---|---|
| PA10 | HC-SR04 Echo | TIM1_CH3，上升沿捕获 |
| PA11 | HC-SR04 Trig | GPIO 输出 |
| PB8 | SG90 PWM | TIM4_CH3，50Hz PWM |
| PB6 | OLED SCL | I2C1_SCL |
| PB7 | OLED SDA | I2C1_SDA |
| PB10 | 蓝牙 RX / USART3_TX | 接蓝牙模块 RX |
| PB11 | 蓝牙 TX / USART3_RX | 接蓝牙模块 TX |
| PA6 | 蓝色 LED | 输出 |
| PA7 | 绿色 LED | 输出 |
| PB0 | 红色 LED | 输出 |
| PB12 | KEY1 | 暂停/恢复 |
| PB13 | KEY2 | OLED 开关 |

> 蓝牙模块与 STM32 必须共地，TX/RX 交叉连接。

---

## 快速开始

### 下位机编译

```bash
cd firmware
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

生成的固件：

```text
firmware/build/Debug/UltrasonicRadar.elf
```

使用 STM32CubeProgrammer 或 ST-Link Utility 烧录。


---

## Windows 上位机

Windows 版位于：

```text
host_windows/MVS_SPP.py
```

### 环境

```bash
pip install pyserial
```

如果需要滤波、插值和绘图，再安装：

```bash
pip install numpy scipy matplotlib
```

### 运行

确认设备管理器中蓝牙 SPP 的 COM 口，并确保串口调试助手已关闭。

修改 `MVS_SPP.py`：

```python
client = SppReceiver(
    port="COM9",
    baudrate=9600,
    on_bytes=callback_when_receive_data
)
```

运行：

```bash
cd host_windows
python MVS_SPP.py
```

程序会解析下位机二进制帧，并输出：

```text
d=1.234, a=45.00
```

---

## ROS2 上位机

ROS2 功能包位于：

```text
laser_scan/
my_interfaces/
```

### 编译

> 假设你的 ROS2 工作区是 `~/ros2_ws`

```bash
cd ros2_ws
colcon build --packages-select my_interfaces
source install/setup.bash
colcon build --packages-select laser_scan
source install/setup.bash
```

### 运行

分别打开终端：

```bash
ros2 run laser_scan stm32
ros2 run laser_scan cmd_controller
ros2 run laser_scan visualizer
ros2 run laser_scan radar_static_tf
```

然后在 RViz2 中添加 LaserScan 显示，订阅：

```text
/radar/laser_scan
```

### 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/radar/sensor_data` | `my_interfaces/PolarCoordinates` | 原始极坐标点 |
| `/radar/laser_scan` | `sensor_msgs/LaserScan` | 可供 RViz2 显示的数据 |
| `/radar/command` | `std_msgs/Int32MultiArray` | 上位机指令 |

---

## 指令表

- CLI指令

| 文本指令 | 机器码 | 作用 |
|---|---|---|
| `start` | `0 0` | 解除暂停 |
| `stop` | `0 1` | 暂停 |
| `oled on` | `1 1` | 打开 OLED |
| `oled off` | `1 0` | 关闭 OLED |
| `speed set N` | `2 1 N` | 设置舵机速度 |
| `speed add N` | `2 2 N` | 增加舵机速度 |
| `speed remove N` | `2 3 N` | 减少舵机速度 |
| `cooling_time set N` | `3 1 N` | 设置冷却时间 |
| `cooling_time add N` | `3 2 N` | 增加冷却时间 |
| `cooling_time remove N` | `3 3 N` | 减少冷却时间 |
| `resample N` | `4 N` | 设置重复采样次数 |

- 原始数据

```text
cmd 0 0        # 解除暂停
cmd 0 1        # 暂停
cmd 1 0        # 关闭 OLED
cmd 1 1        # 打开 OLED
cmd 2 1 X      # 设置舵机速度
cmd 3 1 X     # 设置冷却时间
cmd 4 X        # 设置重复采样次数
q              # 退出
```

> X 必须为正整数

### 错误码

| 错误码 | 说明 |
|---|---|
| `0x00` | 参数无效 |
| `0x01` | 参数个数与指令不匹配 |
| `0x02` | 无效指令 |

---

## 常见问题

### 1. 蓝牙收到乱码

- 确认蓝牙模块处于 SPP 数据模式；
- 确认 STM32 和 PC 两端波特率一致，当前为 9600；
- 确认帧头（`0XAA`、`0X55`）、帧尾（`0x0D`）与下位机一致；

### 2. Python 收不到数据

- 串口调试助手和 Python 不能同时占用同一个 COM 口；
- Windows 蓝牙 SPP 可能有多个 COM 口，使用 Outgoing 口；
- 优先用 `ser.read(1)` 验证原始字节，再接入解析器；
- 固件必须发送 `\r\n` 或使用二进制帧解析。

### 3. 舵机不转

- 确认 `pause == false`；
- 检查 PB8 是否接舵机信号线；
- 确认 TIM4 PWM 周期为 20ms；
- 检查舵机以 5V 供电；
- 确认舵机与 STM32 是否共地。

### 4. 超声波不测距

- 检查 PA11 Trig、PA10 Echo 接线；
- 触发高电平需要至少 10us；
- 检查 TIM1 捕获中断和 TIM3 计时；
- 注意 Echo 电平，必要时做电平转换。

### 5. OLED 不亮

- 检查 I2C 地址，常见 `0x78` / `0x7A`；
- 在 CubeMX 中检查 I2C1 所用引脚为 PB6/PB7（上拉）；
- 确认 CH1116 与 SSD1306 初始化命令差异。

---

## 许可证

本仓库包含以下不同许可的部分：

- **STM32CubeMX 生成代码（`SFR_UltrasonicRadar_Firmware 下的 Core/`、`Drivers/`、`Middlewares/` 等）**：
  版权归 STMicroelectronics 所有，适用 BSD-3-Clause 许可证。
  详见各文件头部注释。

- **项目自有代码（`用户自定义部分）**：
  适用 MIT 许可证。详见 [LICENSE](LICENSE)。
