# ButterflyKeil

基于 STM32G0 的蝴蝶飞行器控制系统，使用 Keil 开发。

## 功能特性

- **电机控制**：支持 4 路直流电机 PWM 调速，带方向控制
- **CRSF 协议**：支持 ELRS 遥控器 CRSF 协议接收
- **LED 指示**：系统状态指示灯
- **霍尔传感器**：双通道霍尔传感器模拟量读取，用于翅膀位置检测
- **扑翼控制**：基于霍尔传感器的自动扑翼运动控制，支持对向扇动模式
- **PID同步控制**：双电机相位同步控制，确保翅膀运动协调

## 硬件配置

### MCU

- 芯片：STM32G0 系列
- 开发环境：Keil MDK

### 引脚分配

| 功能   | 引脚   | 说明              |
| ---- | ---- | --------------- |
| LED  | PB4  | 状态指示灯（低电平有效）    |
| PWM1 | PC6  | 电机1 PWM 输出      |
| PWM2 | PA7  | 电机2 PWM 输出      |
| PWM3 | PB0  | 电机3 PWM 输出      |
| PWM4 | PB1  | 电机4 PWM 输出      |
| DIR1 | PA8  | 电机1 方向控制        |
| DIR2 | PA11 | 电机2 方向控制        |
| DIR3 | PA12 | 电机3 方向控制        |
| DIR4 | PA15 | 电机4 方向控制        |
| TX   | PB6  | USART1 发送（调试） |
| RX   | PB7  | USART1 接收（调试） |
| TX4  | PA0  | USART4 发送（CRSF） |
| RX4  | PA1  | USART4 接收（CRSF） |
| HALL1 | PA3 | 霍尔传感器1（ADC3） |
| HALL2 | PA4 | 霍尔传感器2（ADC4） |
| VBAT  | PA2 | 电池电压（ADC2）   |

## 模块说明

### 电机控制模块 (motor.c/h)

```c
// 初始化
MotorControl_Init();

// 设置电机速度 (-1000 ~ 1000，正值正转，负值反转)
Motor_SetSpeed(MOTOR_1, 500);

// 停止电机
Motor_Stop(MOTOR_1);
Motor_StopAll();

// 获取电机状态
Motor_Status_t status = Motor_GetStatus(MOTOR_1);
```

### CRSF 协议模块 (crsf.c/h)

解析 ELRS 遥控器发送的 CRSF 协议数据。

```c
// 遥控器数据结构
crsf_data.Left_X      // 左摇杆 X 轴 (-100 ~ 100)
crsf_data.Left_Y      // 左摇杆 Y 轴 (0 ~ 100)
crsf_data.Right_X     // 右摇杆 X 轴 (-100 ~ 100)
crsf_data.Right_Y     // 右摇杆 Y 轴 (-100 ~ 100)
crsf_data.S1          // 左滑块 (0 ~ 100)
crsf_data.S2          // 右滑块 (0 ~ 100)
crsf_data.A           // 左按键 A (0/1)
crsf_data.B           // 拨杆 B (0/1/2)
crsf_data.C           // 拨杆 C (0/1/2)
crsf_data.D           // 右按键 D (0/1)
crsf_data.E           // 拨杆 E (0/1/2)
crsf_data.F           // 拨杆 F (0/1/2)
```

### 扑翼控制模块

实现基于霍尔传感器的翅膀运动控制：

#### 控制模式

- **对向扇动模式**：两个电机相位差180度交替扇动，形成推进力
- **动态阈值**：根据速度动态调整霍尔传感器阈值，提高边界检测准确性
- **PID同步**：使用PID控制器确保双电机运动协调

#### 关键参数

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| hall_threshold_low | 1.2V | 电机1低位阈值 |
| hall_threshold_high | 2.2V | 电机1高位阈值 |
| hall_threshold_low_m4 | 1.3V | 电机4低位阈值 |
| hall_threshold_high_m4 | 2.3V | 电机4高位阈值 |

#### 扑翼控制API

```c
// 扑翼控制逻辑（电机1）
Flap_Control_Logic(int16_t speed);

// 扑翼控制逻辑（电机4，对向扇动）
Flap_Control_Logic_Motor4(int16_t speed);
```

## 工具说明

### 摇杆数据监控工具 (tools/joystick_monitor.py)

这是一个基于Python的实时数据监控工具，用于显示遥控器摇杆数据、电池电压和霍尔传感器数据。

![CRSF Joystick Data Monitor](picture/Tools.png)

**功能特点：**

- **实时数据显示**：左摇杆、右摇杆、滑块位置
- **开关状态监控**：B(SA)、F(SD)、E(SB)、C(SC)开关状态
- **电池电压监控**：实时显示电池电压和原始ADC值
- **霍尔传感器监控**：双通道霍尔传感器数据显示
- **电机状态显示**：实时显示两个电机的扇动方向（UP/DOWN）
- **历史曲线**：显示摇杆数据和电池电压的变化曲线
- **帧率显示**：实时显示数据更新帧率

**使用方法：**

```bash
# 安装依赖
pip install pyserial matplotlib

# 运行监控工具
python tools/joystick_monitor.py -p COM4 -b 420000

# 查看可用串口
python tools/joystick_monitor.py -l
```

**数据格式：**

工具接收来自STM32的CSV格式数据：

```
Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F,BatteryVoltage,ADCValue,Hall1,Hall1ADC,Hall2,Hall2ADC,PhaseError,PhaseLeft,PhaseRight,Motor1State,Motor4State
```

## 编译与烧录

1. 使用 Keil MDK 打开项目文件
2. 编译项目（Build）
3. 连接 ST-Link 下载器
4. 点击 Download 烧录

## 目录结构

```
ButterflyKeil/
├── Core/
│   ├── Inc/                # 头文件
│   │   ├── main.h
│   │   ├── motor.h         # 电机控制模块
│   │   ├── crsf.h          # CRSF 协议模块
│   │   └── ...
│   └── Src/                # 源文件
│       ├── main.c          # 主程序（含扑翼控制逻辑）
│       ├── motor.c         # 电机控制实现
│       ├── crsf.c          # CRSF 协议实现
│       ├── adc.c           # ADC配置与数据读取
│       └── ...
├── Drivers/                # STM32 HAL 驱动
├── tools/                  # 工具脚本
│   └── joystick_monitor.py # 摇杆数据监控工具
└── ButterflyKeil.ioc       # STM32CubeMX 配置文件
```

## 注意事项

1. LED 为**低电平有效**，使用 `LED_ON()` 点亮，`LED_OFF()` 熄灭
2. 电机速度范围：-1000 ~ 1000
3. CRSF 数据通过 USART4 + DMA 接收
4. ADC 使用扫描模式，依次采样：电池电压、霍尔传感器1、霍尔传感器2
5. 扑翼控制使用对向扇动模式，确保左右翅膀交替运动

## 许可证

MIT License