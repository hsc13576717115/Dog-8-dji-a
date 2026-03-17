# Robocon2024

基于 `STM32F427IIHx` 的四足/并联腿机器人控制工程，使用 `STM32 HAL + FreeRTOS(CMSIS-RTOS v2)` 搭建底层框架，使用摆线步态生成器、腿部逆运动学和 RS485 电机通信实现整机运动控制。

这个工程明显是比赛机器人项目，不是通用模板。很多参数、状态切换、起立动作、跳跃动作和自动流程都已经针对当前机构尺寸、场地任务和传感器布局做了实机标定，所以把它当成“可直接运行的整机控制工程”理解会更准确。

## 1. 项目概览

### 1.1 核心功能

- 基于 FreeRTOS 的多任务机器人控制框架
- 8 个关节电机的位置控制与状态回读
- 四足步态生成：`WALK / TROT / RUN / MINCE / CREEP / CLIMBING`
- 起立、趴下、跳跃、爬坡等比赛动作
- 遥控器控制与状态机切换
- JY901 姿态角读取与姿态修正
- 激光测距、视觉串口、按钮输入等外围接口
- 预留自动模式逻辑

### 1.2 软硬件平台

- MCU: `STM32F427IIHx`
- 内核: `Cortex-M4F`
- 主频: `180 MHz`
- 工程工具链: `Keil uVision5`
- 编译器: `ARMCC 5.06 update 7`
- Cube 配置文件: `Robocon2024.ioc`
- 中间件: `FreeRTOS V10.3.1`
- HAL 包: `Keil.STM32F4xx_DFP.2.17.1`

### 1.3 当前工程模式

当前代码默认编译为遥控模式：

```c
// Task/Inc/posture_task.h
#define smart 0 // 遥控0 自主1
```

也就是说，仓库里虽然保留了较多自主逻辑，但默认生效的是“遥控控制整机”的分支。

## 2. 工程目录

```text
Robocon2024/
├─ Automatic/      自动流程与自主动作
├─ BSP/            板级支持与外设协议解析
├─ Core/           CubeMX 生成的启动、时钟、GPIO、DMA、串口、FreeRTOS 入口
├─ Drivers/        STM32 HAL / CMSIS 驱动
├─ Lib/            算法库：PID、步态生成、逆解、VMC 等
├─ MDK-ARM/        Keil 工程、启动文件、编译输出
├─ Middlewares/    FreeRTOS 中间件
├─ Task/           各个业务任务：姿态、步态、电机、跳跃、遥控等
├─ .mxproject      CubeMX 工程元数据
└─ Robocon2024.ioc CubeMX 外设配置文件
```

### 2.1 各目录作用

- `Core/`
  - `main.c`: 程序入口，负责硬件初始化、初始电机读数、启动调度器
  - `freertos.c`: 创建各个任务
  - `gpio.c / dma.c / usart.c`: GPIO、DMA、串口底层初始化
- `Task/`
  - `Ahrs_Task.c`: IMU 姿态读取与腿长/姿态修正
  - `Motor_Task.c`: 将目标关节角发给 8 个电机并接收反馈
  - `posture_task.c`: 核心步态与状态机任务
  - `remoctrl_task.c`: 遥控器数据解析和模式切换
  - `Standup_Task.c`: 起立/趴下/高度切换
  - `jump_task.c`: 跳跃动作
  - `Gait_Param.c`: 步态参数表和状态枚举
- `BSP/`
  - `GO-MOTOR.c / GO-M8010-6.c`: 电机对象初始化、协议打包和 RS485 通信
  - `serial.c`: 遥控器 SBUS 风格数据解析
  - `jy901s.c`: JY901 串口姿态模块驱动
  - `Vision.c`: 视觉串口帧解析
  - `laser.c`: 激光测距串口解析
  - `Callback.c`: 串口 DMA 接收完成回调
- `Lib/`
  - `cycloid_generator.c`: 摆线/足端轨迹生成
  - `stanford_type.c`: 腿部逆运动学
  - `pid.c`: PID 控制
  - `VMC.c`: 虚拟模型控制相关试验代码
- `Automatic/`
  - `auto_action.c`: 自动前进、转向、视觉/陀螺仪闭环动作
  - `auto_action_t.c`: 另一套自动动作接口
- `MDK-ARM/`
  - `Robocon2024.uvprojx`: Keil 主工程
  - `startup_stm32f427xx.s`: 启动文件
  - `Robocon2024/Robocon2024.hex`: 编译输出的烧录文件

## 3. 总体软件框架

### 3.1 启动流程

程序上电后的主要流程如下：

1. `HAL_Init()`
2. `SystemClock_Config()` 将系统时钟配置到 `180 MHz`
3. 初始化 `GPIO / DMA / USART6 / UART4 / UART7`
4. 调用自定义 `USART1_Init()` 初始化遥控串口
5. 调用 `JY901_Init()` 初始化姿态模块
6. 调用 `Motor_Init()` 初始化 8 个电机对象、PID 和腿模型参数
7. 初始化激光、遥控接收、视觉串口
8. 连续读取 8 个电机当前位置，保存为 `Initial_Angle[]`
9. 创建 FreeRTOS 任务
10. 启动调度器 `osKernelStart()`

### 3.2 控制链路

工程的主控制链可以概括为：

```text
遥控器 / 按钮 / 视觉 / 激光 / IMU
            ↓
      串口 DMA 回调解析
            ↓
     全局状态与传感器数据更新
            ↓
 Posture_Task 选择状态并生成足端目标轨迹
            ↓
   Stanford_Type_Lite 逆运动学求关节角
            ↓
        Aim_Angle[8] 目标关节角
            ↓
       Motor_Task 发送电机位置命令
            ↓
   USART6 + RS485 与 8 个电机收发数据
            ↓
      电机回读 / IMU 修正 / 下一控制周期
```

### 3.3 分层理解

可以把整个工程看成 4 层：

1. 硬件接口层  
   `Core + BSP`，负责串口、DMA、GPIO、协议收发。

2. 算法层  
   `Lib`，负责 PID、轨迹生成、逆运动学、极坐标/笛卡尔变换。

3. 任务调度层  
   `Task`，负责实时调度、状态切换、步态计算、起立/跳跃控制。

4. 比赛策略层  
   `Automatic`，负责自主前进、转向和传感器联动流程。

## 4. FreeRTOS 任务框架

`Core/Src/freertos.c` 中创建了以下任务：

| 任务名 | 优先级 | 周期/触发方式 | 作用 |
| --- | --- | --- | --- |
| `AHRS` | `osPriorityNormal4` | 约 `3 ms` | 读取 JY901 姿态角，计算姿态修正量 |
| `RemoCtrl` | `osPriorityNormal3` | 约 `10 ms` | 解析遥控器数据，切换模式和动作状态 |
| `Motor` | `osPriorityNormal2` | 约 `3 ms` | 将目标角写入电机，读取 8 路反馈 |
| `Standup` | `osPriorityNormal1` | 一次性执行后挂起 | 完成起立流程 |
| `Jump` | `osPriorityNormal1` | 被恢复后执行一次 | 完成跳跃流程 |
| `Posture` | `osPriorityNormal` | 约 `6 ms` | 步态生成、逆解、更新目标角 |
| `Process` | `osPriorityBelowNormal7` | 一次性逻辑 | 起立完成后将系统切到 `STOP` 并自挂起 |

### 4.1 任务间关系

- `Standup_Task` 负责把机构从初始姿态抬起来，完成后置 `flag_Standup = 1`
- `Process_Task` 检测到起立完成后，强制进入 `STOP`，防止系统一启动就运动
- `Posture_Task` 是运动控制核心，它根据 `NowState` 选择步态或动作
- `Motor_Task` 不直接做高层决策，只负责把 `Aim_Angle[8]` 发到电机
- `Jump_Task` 平时挂起，只有 `NowState` 被切到跳跃态时才被恢复执行
- `AHRS_Task` 独立更新姿态修正量，供步态和电机控制读取

## 5. 状态机与运动模式

### 5.1 主要状态枚举

工程在 `Task/Inc/Gait_Param.h` 中定义了整机状态：

- `STOP`
- `WALK`
- `TROT`
- `RUN`
- `MINCE`
- `CREEP`
- `CLIMBING`
- `JUMP_GROUND`
- `JUMP_UPHILL`
- `JUMP_UP`
- `JUMP_TEST`
- `JUMP_UP_SLOPE`
- `RELEASE`
- `START`
- `END`

其中真正常用的是 `STOP / WALK / TROT / RUN / MINCE / CREEP / CLIMBING / JUMP_*` 这一组。

### 5.2 状态切换规则

状态切换集中在两个地方：

- `Change_NowState()`  
  用于安全地更新 `NowState` 并置位 `Mode_Change_Flag`

- `Ctrl_byRemoctrl()`  
  将遥控器开关组合映射为运动状态

### 5.3 步态参数来源

不同状态对应的默认步态参数存放在 `Task/Src/Gait_Param.c` 的 `state_gait_params[]` 中。  
每组参数对应：

- `body_height`: 身体高度
- `step_length`: 默认步长
- `up_amp`: 抬腿高度
- `down_amp`: 落腿补偿
- `flight_percent`: 摆动相比例
- `freq`: 步频
- `change_step_length`: 遥控/自动模式下动态调整用的步长基准

如果你要调机器人“走得更快、更高、更稳”，最先改的通常就是这里。

## 6. 步态、逆解与电机控制

### 6.1 足端轨迹生成

`Lib/Src/cycloid_generator.c` 使用摆线参数生成每条腿的目标足端轨迹 `(x, y)`：

- 支持设置步长、步频、抬腿高度、落腿补偿
- 不同腿通过相位差 `0.01 / 0.51` 错峰运行
- 当状态切换时，`Mode_Change_Flag` 会重新初始化相位

### 6.2 腿部逆运动学

`Target_toMotor()` 的流程是：

1. 根据当前 4 条腿的 `aim_x[] / aim_y[]` 设置目标足端坐标
2. 调用 `Stanford_Type_Lite_Inverse_Kinematics()` 计算每条腿的 2 个关节角
3. 生成 `Aim_Angle[8]`
4. 置位 `Is_Aim_Angle_Get = 1`

该工程使用的是简化版 Stanford 并联腿模型，初始化参数在 `Motor_Init()` 中写死为：

- `L1 = 15.0`
- `L2 = 27.0`

如果机构尺寸改了，这里必须同步修改，否则逆解会直接失真。

### 6.3 电机控制

`Motor_Task()` 是关节控制核心：

- 读取 `Aim_Angle[8]`
- 叠加 `Initial_Angle[]`、起立偏置、姿态修正量
- 调用 `GO_motor_position()` 生成电机位置控制指令
- 通过 `SERVO_Send_recv()` 与 8 个电机逐个收发

实际电机总线使用：

- `USART6`
- 波特率 `4000000`
- RS485 方向控制脚 `PE6 / PE12`

### 6.4 PID 与增益

电机、IMU 相关 PID 在 `BSP/Src/GO-MOTOR.c` 中初始化：

- `PID_Position[8]`
- `PID_Speed[8]`
- `PID_IMU_Line`
- `PID_IMU_Climbing`

但当前主循环更偏“直接位置模式 + 电机内部刚度阻尼控制”，所以对整机行为影响更大的，往往是：

- `Motor_T[i].K_P`
- `Motor_T[i].K_W`
- `Standup_Task.c` 中的 `CHANGE_KPW()`
- `Jump_Task.c` 中各阶段的刚度/阻尼

## 7. 起立、趴下与跳跃动作

### 7.1 起立

`Standup_Task()` 在系统启动后自动执行一次：

- 先用较低的刚度/阻尼慢慢抬腿
- 逐步修改 `Standup_LegL_Offset` 和 `Standup_LegR_Offset`
- 最终切到比赛用刚度阻尼
- 置位 `flag_Standup = 1`
- 自行挂起

这意味着机器人正常上电后，会自动完成一次“站起来”的过程。

### 7.2 趴下

`Setdown()` 会：

- 降低刚度和阻尼
- 清零左右腿的起立偏置
- 清零 `CREEP_LegL / CREEP_LegR`
- 将 `flag_Standup` 置 0

### 7.3 跳跃

`Jump_Task()` 平时挂起，在切到跳跃状态后恢复执行一次。  
当前代码中实现了多种跳跃流程：

- 原地跳 `JUMP_GROUND`
- 上坡/高台跳 `JUMP_UPHILL`
- 大跳坡 `JUMP_UP_SLOPE`
- 测试跳 `JUMP_TEST`

每一种跳跃都分为若干阶段：

- 预压缩
- 发力起跳
- 空中收腿
- 落地缓冲
- 稳定恢复

这些动作高度依赖当前机械结构、重心和电机能力，移植到别的机器上时需要重新标定。

## 8. 外设与接口说明

### 8.1 串口分配

| 接口 | 波特率 | 作用 | 备注 |
| --- | --- | --- | --- |
| `USART6` | `4000000` | 电机 RS485 总线 | 电机控制主通道 |
| `USART1` | `100000` | 遥控器接收 | 使用自定义 `USART1_Init()` |
| `UART4` | `9600` | 激光测距 | DMA 接收 11 字节帧 |
| `UART7` | `9600` | 视觉串口 | DMA 接收 6 字节帧 |
| `UART8` | `9600` | JY901 IMU | DMA 接收 11 字节帧 |

### 8.2 引脚信息

当前代码里明确出现的关键引脚如下：

- `PE6`: `RS485_DE`
- `PE12`: `RS485_RE`
- `PG14 / PG9`: `USART6_TX / USART6_RX`
- `PB7 / PA9`: `USART1_RX / USART1_TX`
- `PA1 / PA0`: `UART4_RX / UART4_TX`
- `PE7 / PE8`: `UART7_RX / UART7_TX`
- `PE0 / PE1`: `UART8_RX / UART8_TX`
- `PD12 / PD13 / PD14 / PD15`: `Buttom_1 ~ Buttom_4`
- `PH10`: `Buttom_5`
- `PH11`: `ON_OFF`
- `PA3`: 附加输入按键

### 8.3 遥控器

遥控数据由 `BSP/Src/serial.c` 解包成 `RC_ctrl_t`，并在 `RemoCtrl_Task()` 中周期性更新。

主要通道含义可按当前代码理解为：

- `CH3`: 前进/后退主控制
- `CH2`: 转向差速
- `CH0 / CH1`: 微调或特殊状态输入
- `CH4`: IMU 直线修正开关

### 8.4 IMU

JY901 初始化流程包括：

- 解锁
- 设为 6 轴算法
- 加速度校准
- Z 轴回零
- 设置回传内容
- 设置回传频率

`AHRS_Task()` 每约 `3 ms` 读取一次：

- `roll`
- `pitch`
- `yaw`

并计算 `imu_Leg_offset` 用于腿部姿态补偿。

### 8.5 视觉

视觉串口当前使用 `UART7`，代码里解析出的字段包括：

- `fb_speed`
- `lr_speed`
- `vision_state`
- `x_error`
- `action`

在自主模式下，`vision_state` 和 `x_error` 会参与 `Smart_State` 切换和自动前进/转向控制。

### 8.6 激光

激光模块使用 `UART4`：

- 初始化时会发送连续测量命令
- 接收 11 字节数据帧
- 通过校验和后解析距离值

## 9. 遥控模式用法

### 9.1 编译前确认

确保：

```c
#define smart 0
```

### 9.2 上电行为

1. 板子上电
2. 串口和传感器初始化
3. 电机回读初始角度
4. `Standup_Task` 自动起立
5. `Process_Task` 将系统切到 `STOP`
6. 机器人等待遥控器切模式

### 9.3 遥控器开关组合

代码中通过 `RC_11 ~ RC_33` 判定双开关状态，实际行为如下：

| 开关状态 | 实际代码行为 |
| --- | --- |
| `RC_11` | 趴下并切到 `STOP` |
| `RC_12` | `WALK`，开启 IMU 修正 |
| `RC_13` | 当前代码仍切到 `WALK`，但关闭 IMU |
| `RC_21` | `WALK` |
| `RC_22` | `TROT` |
| `RC_23` | 跳跃模式，根据摇杆组合进入不同 `JUMP_*` |
| `RC_31` | 爬坡/侧爬准备逻辑，后续可能切到 `RUN` |
| `RC_32` | `CREEP` 匍匐/低姿态模式 |
| `RC_33` | `MINCE` 小步态模式 |

注意：代码注释里有一些旧描述，但 README 这里写的是当前分支真实执行逻辑。

### 9.4 摇杆控制方式

在 `Change_RmVAL()` 中：

- 当 `CH0` 和 `CH1` 在中位附近时  
  使用 `CH3` 控制前后步长，`CH2` 控制左右差速

- 当 `CH0` 或 `CH1` 偏离中位时  
  进入低频微调模式，使用更小的步长精调

### 9.5 IMU 直线修正

- `CH4 > 1600` 时开启直线修正，并把当前 `yaw` 作为目标角
- `CH4 < 400` 时关闭直线修正

开启后，步态生成器会对左右步长做差分补偿。

## 10. 自主模式用法

### 10.1 开启方法

将 `Task/Inc/posture_task.h` 改为：

```c
#define smart 1
```

然后重新编译烧录。

### 10.2 自主模式行为

自主模式下 `Posture_Task()` 会：

- 读取按键输入 `Buttom_1 ~ Buttom_5`
- 读取 `vision_Data.vision_state`
- 根据遥控器和按钮设置 `flag_go`
- 选择 `Smart_State`
- 调用 `ALL_AUTO_4()` 等自动流程

当前代码里真正启用的是 `ALL_AUTO_4()`，说明这套自主逻辑也带有明显的比赛场景绑定，不是通用导航框架。

### 10.3 建议

如果你要继续做自主，建议优先确认：

- 视觉帧格式是否与当前板端解析一致
- `ALL_AUTO_4()` 是否仍然适配现在的比赛流程
- 按键输入和场地传感器是否仍然按原线束连接

## 11. 构建与烧录

### 11.1 使用 Keil MDK 编译

1. 打开 `MDK-ARM/Robocon2024.uvprojx`
2. 选择目标 `Robocon2024`
3. 点击 `Build`
4. 点击 `Download`

编译产物默认输出到：

```text
MDK-ARM/Robocon2024/
├─ Robocon2024.axf
├─ Robocon2024.hex
├─ Robocon2024.map
└─ Robocon2024.htm
```

### 11.2 使用 CubeMX 维护外设

如果你修改了：

- 引脚
- 串口
- DMA
- 时钟
- FreeRTOS 配置

请在 `Robocon2024.ioc` 中修改并重新生成代码。  
但重新生成前要特别注意：

- 自定义初始化逻辑是否写在 `USER CODE` 区域
- `usart.c` 中的 `USART1_Init()`、`UART8_Init()` 这类手写函数是否会被覆盖
- `freertos.c` 中任务创建和用户 include 是否会被 Cube 重写

## 12. 关键可调参数位置

如果你后面要继续调车，最常改的地方通常是这些：

### 12.1 步态参数

- 文件: `Task/Src/Gait_Param.c`
- 关注对象: `state_gait_params[]`

用于调整：

- 身高
- 步长
- 抬腿高度
- 步频
- 摆动相比例

### 12.2 起立与高度

- 文件: `Task/Src/Standup_Task.c`

用于调整：

- 起立角度偏置
- 起立速度
- 刚度阻尼切换
- 匍匐高度

### 12.3 跳跃动作

- 文件: `Task/Src/jump_task.c`

用于调整：

- 压缩时间
- 起跳时间
- 收腿时间
- 落地缓冲参数
- 各阶段 `K_P / K_W`

### 12.4 电机控制参数

- 文件: `BSP/Src/GO-MOTOR.c`

用于调整：

- 电机对象默认参数
- PID 初值
- 腿长模型参数

### 12.5 遥控映射

- 文件: `Task/Src/remoctrl_task.c`

用于调整：

- 开关状态对应动作
- 摇杆与步长/转向映射
- IMU 修正开关逻辑

### 12.6 自主流程

- 文件: `Task/Src/posture_task.c`
- 文件: `Automatic/Src/auto_action.c`

用于调整：

- 自动前进/转向
- 视觉闭环
- 比赛流程
- 按键触发条件

## 13. 维护建议

### 13.1 先理解再改

这不是纯粹“模块独立”的教学工程，而是已经揉进了很多比赛临场经验的代码。  
很多值虽然看起来像魔法数，但实际上往往对应：

- 某个具体机构角度
- 某段跳跃时长
- 某个场地障碍高度
- 某次电机刚度调参结果

所以不建议直接大范围重构，最好先在以下顺序上理解：

1. `main.c`
2. `freertos.c`
3. `posture_task.c`
4. `Motor_Task.c`
5. `Standup_Task.c`
6. `jump_task.c`
7. `GO-MOTOR.c / GO-M8010-6.c`

### 13.2 改机构时必须同步修改

如果机器人机构变了，至少要同步检查：

- `Stanford_Type_Lite_Init()` 的腿长参数
- `Initial_Angle[]` 的零位逻辑
- `Standup_LegL_Offset / Standup_LegR_Offset`
- 跳跃各阶段的极坐标目标

### 13.3 编码与注释

源码里部分中文注释存在编码混杂现象，说明历史上可能在不同编辑器/不同编码设置下维护过。  
建议后续统一使用 `UTF-8` 保存文档，源码如果继续由 Keil 维护，最好先确认当前工程对中文注释的编码兼容性，再批量转换。

## 14. 一句话总结

这个工程的本质是：

“以 `Posture_Task` 为核心状态机，以摆线步态和逆运动学生成 8 个关节目标角，再由 `Motor_Task` 通过 `USART6 + RS485` 驱动 8 个电机，同时结合遥控、IMU、视觉和激光实现比赛机器人整机控制。”

如果你后续要继续接手这个项目，最值得优先读懂的三个文件是：

- `Task/Src/posture_task.c`
- `Task/Src/remoctrl_task.c`
- `Task/Src/Motor_Task.c`

