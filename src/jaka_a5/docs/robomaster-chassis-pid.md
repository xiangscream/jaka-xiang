# RoboMaster 底盘 PID 控制详解

> 本文档详细解释 RoboMaster 比赛中底盘控制的具体实现
> 内容涵盖：整体控制链路、PID 算法实现、速度解算、CAN 通信
> 适用于 STM32/MCU 嵌入式开发

---

## 目录

1. [整体控制链路](#1-整体控制链路)
2. [代码结构](#2-代码结构)
3. [麦克纳姆轮速度解算](#3-麦克纳姆轮速度解算)
4. [PID 控制实现](#4-pid-控制实现)
5. [完整控制周期](#5-完整控制周期1ms-周期)
6. [编码器速度计算](#6-编码器速度计算)
7. [CAN 通信协议](#7-can-通信协议)
8. [参数调试指南](#8-参数调试指南)

---

## 1. 整体控制链路

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌────────────┐    ┌─────────┐
│ 遥控器/决策  │───>│ 底盘控制器    │───>│ 速度解算    │───>│ PID控制器  │───>│ CAN总线  │
│  (输入指令) │    │  (上层逻辑)  │    │ (麦轮逆解)  │    │ (电流环)   │    │  电机   │
└─────────────┘    └──────────────┘    └─────────────┘    └────────────┘    └─────────┘
      │                  │                   │                  │               │
      v                  v                   v                  v               v
  手柄/键鼠        机器人运动学          四个轮子速度        PWM/CAN发送      电机编码器
  速度期望 m/s     麦克纳姆轮解算         期望 rad/s         电机电流         反馈 rad/s
```

### 控制链路详细说明

| 环节 | 输入 | 输出 | 说明 |
|------|------|------|------|
| 上层指令 | vx, vy, wz | 速度期望 | 来自遥控器/键鼠/决策 |
| 速度解算 | vx, vy, wz | 4个轮子期望速度 | 逆运动学 |
| PID控制 | 期望速度, 反馈速度 | 电流值 | 闭环控制 |
| CAN发送 | 电流值 | CAN帧 | 发送给电机驱动 |
| 电机 | 电流 | 实际转速 | 执行器 |
| 编码器 | 角度变化 | 实际速度 | 反馈 |

---

## 2. 代码结构

### 2.1 主控制结构体

```c
/* 底盘控制器结构体 */
typedef struct {
    /* ===== 输入（上层指令）===== */
    float vx;           // X方向速度期望 (m/s)  前进/后退
    float vy;           // Y方向速度期望 (m/s)  左右平移
    float wz;           // 旋转角速度期望 (rad/s)

    /* ===== 速度解算结果（四个轮子期望速度）===== */
    float wheel_speed[4];  // 轮子期望速度 rad/s

    /* ===== PID结构体（速度环）===== */
    PidTypeDef motor_speed_pid[4];

    /* ===== 编码器反馈（实际速度）===== */
    float motor_speed_feedback[4];

    /* ===== 输出（电流）===== */
    float motor_current[4];

    /* ===== CAN发送数据 ===== */
    uint8_t can_send_data[8];
} Chassis_Control_t;

Chassis_Control_t chassis;
```

### 2.2 电机类型（以 GM6020 为例）

```c
/* 电机参数（GM6020 直流无刷电机）*/
#define MOTOR_REDUCTION_RATIO   19.0f     // 减速比 19:1
#define MOTOR_ENCODER_RESOLUTION 4096    // 编码器分辨率 4096 CPR
#define MOTOR_CAN_ID_BASE       0x140     // CAN ID 基础地址

/* 电机ID定义（RoboMaster 标准）*/
typedef enum {
    MOTOR_FL = 0,   // 前左轮 Front-Left
    MOTOR_FR = 1,   // 前右轮 Front-Right
    MOTOR_BL = 2,    // 后左轮 Back-Left
    MOTOR_BR = 3,    // 后右轮 Back-Right
} Motor_ID_e;
```

---

## 3. 麦克纳姆轮速度解算

### 3.1 麦克纳姆轮原理

麦克纳姆轮（Mecanum Wheel）由瑞典工程师 Bengt Ilon 于 1973 年发明，通过斜向滚轮实现全向移动。

```
     WHEEL_0 ┌─────────┐ WHEEL_1
            │ robot   │
            │    ↑    │
            │    Y    │
            │    →X   │
     WHEEL_2└─────────┘ WHEEL_3

轮子排列：0=前左, 1=前右, 2=后左, 3=后右
每个轮子与机器人坐标系的关系：
- 轮子0: 与X轴夹角 45°，逆时针
- 轮子1: 与X轴夹角 45°，顺时针
- 轮子2: 与X轴夹角 45°，顺时针
- 轮子3: 与X轴夹角 45°，逆时针
```

### 3.2 逆运动学公式

```c
#define WHEEL_RADIUS   0.0762f    // 轮子半径 76.2mm (3寸轮)
#define WHEEL_BASE_X   0.2f       // X方向轮距 (m) 前后轮中心距/2
#define WHEEL_BASE_Y   0.17f      // Y方向轮距 (m) 左右轮中心距/2

/**
 * 麦克纳姆轮速度解算（逆运动学）
 *
 * @param vx     X方向速度 (m/s) 正=右
 * @param vy     Y方向速度 (m/s) 正=前
 * @param wz     旋转角速度 (rad/s) 正=逆时针
 * @param wheel_speed 输出数组[4] 轮子角速度 rad/s
 */
void chassis_speed解算(float vx, float vy, float wz, float *wheel_speed)
{
    float rotate_x = WHEEL_BASE_X / 2.0f;  // 0.1m
    float rotate_y = WHEEL_BASE_Y / 2.0f;  // 0.085m

    /* 轮子速度 = vx×cos(θ) + vy×sin(θ) + wz×d
     * 其中 d 是该轮到机器人中心的垂直距离
     *
     * 简化后：
     * wheel_speed[i] = (±vx ± vy ± wz × (rotate_x + rotate_y)) / WHEEL_RADIUS
     */

    /* 轮子0（前左）— 公式：-vx + vy + wz×d */
    wheel_speed[MOTOR_FL] = (-vx + vy + wz * (rotate_x + rotate_y)) / WHEEL_RADIUS;

    /* 轮子1（前右）— 公式：+vx + vy + wz×d */
    wheel_speed[MOTOR_FR] = ( vx + vy + wz * (rotate_x + rotate_y)) / WHEEL_RADIUS;

    /* 轮子2（后左）— 公式：-vx - vy + wz×d */
    wheel_speed[MOTOR_BL] = (-vx - vy + wz * (rotate_x + rotate_y)) / WHEEL_RADIUS;

    /* 轮子3（后右）— 公式：+vx - vy + wz×d */
    wheel_speed[MOTOR_BR] = ( vx - vy + wz * (rotate_x + rotate_y)) / WHEEL_RADIUS;
}
```

### 3.3 运动模式示例

| vx (m/s) | vy (m/s) | wz (rad/s) | 运动效果 |
|----------|----------|------------|----------|
| 1.0 | 0 | 0 | 右侧平移 |
| -1.0 | 0 | 0 | 左侧平移 |
| 0 | 1.0 | 0 | 前进 |
| 0 | -1.0 | 0 | 后退 |
| 0 | 0 | 1.0 | 原地左转 |
| 0.5 | 0.5 | 0 | 前进右斜 |

---

## 4. PID 控制实现

### 4.1 PID 结构体定义

```c
/* PID控制器结构体 */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数

    float target;       // 目标值（期望值）
    float feedback;     // 反馈值（实际值）

    float error;        // 当前误差
    float error_last;   // 上一次误差
    float error_sum;    // 误差积分（积分项）

    float output;       // PID输出
    float output_max;   // 输出限幅

    float deadband;     // 死区（避免震荡）

    /* 微分项滤波 */
    float d_filter;     // 微分项低通滤波系数
    float d_error;     // 滤波后的微分项
} PidTypeDef;
```

### 4.2 PID 初始化

```c
/* 底盘电机速度环 PID 参数（GM6020 典型值）*/
void chassis_pid_init(void)
{
    for (int i = 0; i < 4; i++) {
        chassis.motor_speed_pid[i].Kp = 20.0f;      // P：响应速度
        chassis.motor_speed_pid[i].Ki = 0.5f;       // I：消除静差
        chassis.motor_speed_pid[i].Kd = 0.0f;       // D：一般不用（噪声大）

        chassis.motor_speed_pid[i].output_max = 16000;  // CAN发送上限16384
        chassis.motor_speed_pid[i].deadband = 10;       // 编码器死区
        chassis.motor_speed_pid[i].d_filter = 0.8f;      // 微分滤波系数
    }
}
```

### 4.3 位置式 PID 计算

```c
/**
 * 位置式PID计算
 *
 * u(k) = Kp×e(k) + Ki×Σe(k) + Kd×[e(k) - e(k-1)]
 */
float pid_calculate(PidTypeDef *pid, float target, float feedback)
{
    /* ===== 1. 记录误差 ===== */
    pid->target = target;
    pid->feedback = feedback;
    pid->error = target - feedback;

    /* ===== 2. 死区处理 ===== */
    // 避免在零点附近震荡
    if (fabs(pid->error) < pid->deadband) {
        pid->error = 0;
    }

    /* ===== 3. 积分项（带积分限幅）=====
     * 防止积分饱和导致系统响应变慢
     */
    pid->error_sum += pid->error;

    // 积分限幅：防止积分项过大
    float integral_max = pid->output_max / pid->Ki + 0.5f;
    if (pid->error_sum >  integral_max) pid->error_sum =  integral_max;
    if (pid->error_sum < -integral_max) pid->error_sum = -integral_max;

    /* ===== 4. 微分项（带低通滤波）=====
     * 直接微分对噪声敏感，加滤波减小抖动
     */
    float delta_error = pid->error - pid->error_last;
    float d_term = delta_error / 0.001f; // dt = 1ms

    // 一阶低通滤波：y(n) = α×x(n) + (1-α)×y(n-1)
    pid->d_error = pid->d_filter * d_term + (1 - pid->d_filter) * pid->d_error;
    pid->error_last = pid->error;

    /* ===== 5. PID 计算 ===== */
    pid->output = pid->Kp * pid->error                    // 比例项
                + pid->Ki * pid->error_sum                // 积分项
                + pid->Kd * pid->d_error;                  // 微分项

    /* ===== 6. 输出限幅 ===== */
    if (pid->output >  pid->output_max) pid->output =  pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;

    return pid->output;
}
```

---

## 5. 完整控制周期（1ms 周期）

```c
/* 定时器中断（1ms）— 底盘控制主函数 */
void chassis_control_loop(void)
{
    /* ===== 步骤1：获取上层指令 ===== */
    // 从遥控器/决策模块获取 vx, vy, wz
    // 实际从 CAN 或串口接收，此处简化处理
    chassis.vx = get_remote_vx();   // 单位：m/s
    chassis.vy = get_remote_vy();
    chassis.wz = get_remote_wz();    // 单位：rad/s

    /* ===== 步骤2：速度解算 ===== */
    chassis_speed解算(chassis.vx, chassis.vy, chassis.wz,
                      chassis.wheel_speed);

    /* ===== 步骤3：获取编码器反馈 ===== */
    // 读取电机编码器，计算实际速度
    for (int i = 0; i < 4; i++) {
        chassis.motor_speed_feedback[i] = get_motor_speed(i); // rad/s
    }

    /* ===== 步骤4：PID计算 ===== */
    for (int i = 0; i < 4; i++) {
        chassis.motor_current[i] = pid_calculate(
            &chassis.motor_speed_pid[i],
            chassis.wheel_speed[i],          // 目标速度
            chassis.motor_speed_feedback[i]   // 反馈速度
        );
    }

    /* ===== 步骤5：CAN发送 ===== */
    can_send_motor_current(chassis.motor_current);
}
```

### 5.1 CAN 发送函数

```c
/* CAN发送函数 */
void can_send_motor_current(float *current)
{
    uint8_t data[8];

    // 四个电机的电流值（每个 int16_t）
    int16_t current_0 = (int16_t)current[0];
    int16_t current_1 = (int16_t)current[1];
    int16_t current_2 = (int16_t)current[2];
    int16_t current_3 = (int16_t)current[3];

    // 封装CAN数据（高字节在前）
    data[0] = (current_0 >> 8) & 0xFF;
    data[1] =  current_0        & 0xFF;
    data[2] = (current_1 >> 8) & 0xFF;
    data[3] =  current_1        & 0xFF;
    data[4] = (current_2 >> 8) & 0xFF;
    data[5] =  current_2        & 0xFF;
    data[6] = (current_3 >> 8) & 0xFF;
    data[7] =  current_3        & 0xFF;

    // 发送CAN标准帧（ID: 0x200）
    // ID=0x200 是 RoboMaster 标准底盘电机控制ID
    can_tx(0x200, data, 8);
}
```

---

## 6. 编码器速度计算

```c
/* 获取电机实际速度（编码器）*/
float get_motor_speed(uint8_t motor_id)
{
    static int32_t last_angle[4] = {0};  // 上次角度值
    int32_t now_angle;                   // 当前角度值

    // 读取CAN总线反馈的编码器值（16位，0-65535）
    now_angle = can_rx_motor_angle(motor_id);

    // ===== 角度差计算（处理回绕）=====
    int32_t delta = now_angle - last_angle[motor_id];

    // 处理编码器回绕（16位：0 → 65535 → 0）
    if (delta >  32768) delta -= 65536;  // 正向过零
    if (delta < -32768) delta += 65536;  // 反向过零

    last_angle[motor_id] = now_angle;

    /* ===== 角度差转速度 (rad/s) =====
     * 编码器分辨率: 4096 CPR (每转脉冲数)
     * 电机减速比: 19:1 (GM6020)
     * 采样周期: dt = 0.001s (1ms)
     *
     * 速度 = 角度差 / 编码器分辨率 / 减速比 / dt
     */
    float speed = delta * 2 * PI / MOTOR_ENCODER_RESOLUTION
                          / MOTOR_REDUCTION_RATIO
                          / 0.001f;

    return speed;
}
```

---

## 7. CAN 通信协议

### 7.1 RoboMaster 电机 CAN 协议

| 帧类型 | CAN ID | 数据长度 | 说明 |
|--------|--------|----------|------|
| 发送(控制) | 0x200 | 8字节 | 4个电机电流值 |
| 接收(反馈) | 0x20X | 8字节 | X=电机ID(1-4)，角度+转速 |

### 7.2 CAN 反馈数据格式

```c
/* CAN接收中断处理 */
void can_rx_handler(uint32_t can_id, uint8_t *data)
{
    if (can_id >= 0x201 && can_id <= 0x204) {
        uint8_t motor_idx = can_id - 0x201;  // 电机索引 0-3

        /* 反馈数据格式（每个电机2字节）：
         * Byte[0-1]: 角度值 (uint16_t, 0-8191 对应 0-360°)
         * Byte[2-3]: 转速值 (int16_t, rpm)
         */
        int16_t angle = (data[0] << 8) | data[1];
        int16_t rpm   = (data[2] << 8) | data[3];

        // 存储角度值（供速度计算用）
        motor_angle[motor_idx] = angle;

        // 可选：将RPM转换为 rad/s
        float speed_rpm = rpm / 60.0f;  // rpm → rps
        float speed_rad = speed_rpm * 2 * PI;  // rps → rad/s
    }
}
```

---

## 8. 参数调试指南

### 8.1 PID 参数对照表

| 参数 | 太小 | 正常 | 太大 |
|------|------|------|------|
| **Kp** | 响应慢，跟不上 | 快速响应，无超调 | 抖动、震荡 |
| **Ki** | 静差消除慢 | 静差消除快 | 积分饱和、震荡 |
| **Kd** | 无 | 抑制超调 | 噪声放大、抖动 |

### 8.2 调试步骤

```
1. 先调 Kp：从小到大，逐渐增大直到出现震荡
2. 再调 Ki：在 Kp 基础上，逐渐增大消除静差
3. 最后调 Kd：通常不需要，若震荡严重可适当添加
4. 微调：反复调整直到满意
```

### 8.3 典型问题排查

| 现象 | 原因 | 解决方法 |
|------|------|----------|
| 电机抖动/异响 | Kp 过大 | 减小 Kp 10%~20% |
| 响应慢 | Kp 过小 | 增大 Kp |
| 停止后慢慢移动 | 静差 | 增大 Ki |
| 低速时抖动静止 | 死区过小 | 增大 deadband |
| 高速时失控 | 积分饱和 | 减小积分限幅 |
| 电机发热严重 | 电流过大 | 减小 output_max |

### 8.4 典型参数值（参考）

| 电机型号 | Kp | Ki | Kd | output_max |
|----------|-----|-----|-----|------------|
| GM6020 | 15~25 | 0.3~0.8 | 0 | 16000 |
| M3508 | 10~20 | 0.2~0.5 | 0 | 10000 |
| DM4310 | 8~15 | 0.1~0.3 | 0 | 8000 |

---

## 附录：示波器测量 CAN 总线

### 连接方式

```
示波器探头（AC耦合）:
- CH1: CAN_H (控制板 CAN_H 引脚)
- CH2: CAN_L (控制板 CAN_L 引脚)
- 地线: 共地

或使用差分探头：
- CH1: CAN_H - CAN_L（差分信号）
```

### CAN 信号特征

| 参数 | 正常值 | 说明 |
|------|--------|------|
| CAN_H 静态电平 | 2.5V | 隐性电平 |
| CAN_L 静态电平 | 2.5V | 隐性电平 |
| CAN_H 显性峰值 | 3.5V | 差分显性 |
| CAN_L 显性峰值 | 1.5V | 差分显性 |
| 显性宽度 | 2μs | 标准CAN 250Kbps |

### 波形观察点

1. **CAN_H 和 CAN_L**：观察电平变化是否正常
2. **差分信号**：CH1 - CH2（如果有数学运算功能）
3. **时序**：显性/隐性宽度是否满足 CAN 标准

### 常见故障波形

| 波形 | 问题 | 原因 |
|------|------|------|
| 无波形 | CAN 不通 | 接线错误/波特率不对 |
| 单边扁平 | CAN_H 或 CAN_L 断线 | 一方为0V |
| 畸形脉冲 | 终端电阻不匹配 | 需要120Ω终端电阻 |
| 反射 | 阻抗不匹配 | 缩短线缆/加终端电阻 |

---

*本文档由 Claude (蒠芷) 整理，基于 RoboMaster 嵌入式开发经验*
