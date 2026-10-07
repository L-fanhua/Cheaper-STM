# STM 上位机通信协议 v1.0

适用于 ESP32-S3 + AD5761 + ADS8685 + TMC2209 硬件平台。

---

## 1. 物理层

| 项 | 值 |
|---|---|
| 接口 | USB CDC (虚拟串口) |
| 速率 | USB 全速 12 Mbps，实测吞吐 ~1 MB/s |
| 字节序 | 小端 (Little-Endian) |
| 浮点格式 | IEEE 754 float32 |

---

## 2. 帧格式

```
偏移  长度  字段        说明
─────────────────────────────────────────────
0     2     HEADER     帧头，固定 0x55 0xAA (LE 读作 0xAA55)
2     2     LENGTH     从 CMD 到 CRC 末尾的字节数 (LE)
4     1     CMD        命令码
5     N     PAYLOAD    负载，长度 = LENGTH - 3
5+N   2     CRC16      CRC-16/MODBUS，覆盖 CMD + PAYLOAD (LE)
7+N   1     TAIL       帧尾，固定 0x0A
```

- **总帧长** = 7 + N = 7 + (LENGTH - 3) = LENGTH + 4
- **LENGTH 范围** = 3 (无 payload) ~ 1027 (payload 1024B)

### 2.1 CRC-16/MODBUS 参数

| 项 | 值 |
|---|---|
| 多项式 | 0x8005 (反向 0xA001) |
| 初始值 | 0xFFFF |
| 输入反转 | 是 |
| 输出反转 | 是 |
| 异或值 | 0x0000 |
| 校验范围 | CMD + PAYLOAD (不含帧头、长度、自身、帧尾) |

### 2.2 帧同步策略

接收方状态机：
```
SEARCH_HEADER:
    收到 0x55 → 进入 HEADER_1
    否则丢弃
HEADER_1:
    收到 0xAA → 进入 READ_LENGTH
    否则回 SEARCH_HEADER
READ_LENGTH:
    收满 2 字节 → 校验 LENGTH 范围 [3, 1027]
        合法 → 进入 READ_BODY (读取 LENGTH 字节)
        非法 → 回 SEARCH_HEADER，记录错误 FRAME_LENGTH_INVALID
READ_BODY:
    收满 LENGTH 字节 → 校验帧尾 (倒数第 1 字节 == 0x0A)
        合法 → 校验 CRC
            通过 → 派发帧
            失败 → 丢弃，记录错误 CRC_MISMATCH
        非法 → 丢弃，记录错误 TAIL_MISSING
```

---

## 3. 命令码总表

### 3.1 上位机 → 下位机

| CMD  | 名称             | Payload 长度 | 说明 |
|------|------------------|--------------|------|
| 0x01 | PING             | 0            | 心跳探测 |
| 0x02 | ESTOP            | 0            | 急停 (最高优先级，硬中断) |
| 0x10 | SET_MODE         | 1            | 设置工作模式 |
| 0x11 | SET_SETPOINT     | 4            | 设定隧道电流 |
| 0x12 | SET_PID          | 12           | PID 参数 |
| 0x13 | SET_Z_LIMITS     | 8            | z 软限幅 |
| 0x14 | SET_PID_FREQ     | 4            | PID 执行频率 |
| 0x15 | SET_OVERCURRENT  | 4            | 过流保护阈值 |
| 0x20 | MOVE_XYZ         | 12           | 手动定位 (反馈关时) |
| 0x21 | SET_Z            | 4            | 单独设 z |
| 0x22 | RETRACT_Z        | 6            | tip 全退（步数+速度） |
| 0x23 | SET_BIAS         | 4            | 设置样品偏压电压 |
| 0x24 | STEP_MOTOR       | 6            | 手动步进电机控制 |
| 0x25 | SET_SCAN_MODE    | 1            | 设置扫描模式（恒流/恒高） |
| 0x30 | SCAN_START       | 31           | 启动扫描 |
| 0x31 | SCAN_STOP        | 0            | 停止扫描 |
| 0x32 | SCAN_PAUSE       | 0            | 暂停扫描 (可恢复) |
| 0x33 | SCAN_RESUME      | 0            | 恢复扫描 |
| 0x40 | APPROACH_START   | 27 或 29     | 两段式 approach (长度自适应, 见 4.15) |
| 0x41 | APPROACH_STOP    | 0            | 停止 approach |
| 0x50 | SET_CALIB        | 16           | 标定参数 (写 RAM) |
| 0x51 | SAVE_CALIB       | 0            | 标定持久化到 NVS |
| 0x52 | LOAD_CALIB       | 0            | 从 NVS 加载标定 |
| 0x60 | GET_STATUS       | 0            | 主动查询状态 |
| 0x61 | GET_FW_VERSION  | 0            | 查询固件版本号 |

### 3.2 下位机 → 上位机

| CMD  | 名称             | Payload 长度 | 说明 |
|------|------------------|--------------|------|
| 0x81 | PONG             | 0            | 心跳响应 |
| 0x82 | FW_VERSION       | 不定长       | 固件版本号 (ASCII) |
| 0x90 | STATUS           | 22           | 周期上报状态 (50~100Hz) |
| 0x91 | SCAN_ROW         | 8 + 8*nx     | 整行扫描数据 |
| 0x92 | SCAN_END         | 4            | 扫描完成 |
| 0x93 | APPROACH_DATA    | 9            | approach 过程数据 |
| 0x94 | APPROACH_DONE    | 9            | approach 结束 |
| 0xA0 | ERROR            | 2 + msg_len  | 错误上报 |
| 0xB0 | ACK              | 1            | 命令确认 |
| 0xB1 | NACK             | 2            | 命令拒绝 |

---

## 4. 命令详解

### 4.1 PING / PONG (0x01 / 0x81)

**用途**：心跳检测，确认通信链路存活。

- 上位机周期性发送 PING (建议 1Hz)
- 下位机收到后立即回 PONG
- 超时 3 秒未收到响应 → 上位机判定断连，触发重连
- 下位机若 5 秒未收到 PING → 触发通信超时保护 (ERROR 0x04)

### 4.2 ESTOP (0x02)

**用途**：紧急停止，最高优先级。

**Payload**：无

**下位机响应**：
1. 立即关闭反馈
2. z DAC 输出 z_min (tip 全退)
3. 停止扫描
4. 停止步进电机
5. 进入 idle 模式
6. 上报 ERROR (code=0x01, msg="ESTOP triggered")

**注意**：此命令在固件中应由独立高优先级中断处理，不进命令队列。

### 4.3 SET_MODE (0x10)

**Payload**：
```
偏移  长度  字段  类型  说明
0     1     mode  u8   0=idle, 1=feedback_on, 2=scanning, 3=approach
```

**模式切换约束**：
- 任意模式 → idle: 允许
- idle → feedback_on: 允许
- idle → approach: 允许
- feedback_on → scanning: 允许
- 其他切换: 拒绝 (NACK, reason=INVALID_TRANSITION)

### 4.4 SET_SETPOINT (0x11)

**Payload**：
```
偏移  长度  字段       类型   说明
0     4     I_setpoint f32   隧道电流设定值 (nA)
```

**约束**：I_setpoint ∈ [0.01, 100] nA，超出范围 NACK。

### 4.5 SET_PID (0x12)

**Payload**：
```
偏移  长度  字段  类型  说明
0     4     Kp   f32   比例系数
4     4     Ki   f32   积分系数
8     4     Kd   f32   微分系数
```

### 4.6 SET_Z_LIMITS (0x13)

**Payload**：
```
偏移  长度  字段    类型  说明
0     4     z_min  f32   z 电压下限 (V)，DAC 输出不低于此值
4     4     z_max  f32   z 电压上限 (V)，DAC 输出不高于此值
```

**约束**：-15 ≤ z_min < z_max ≤ 15，否则 NACK。

### 4.7 SET_PID_FREQ (0x14)

**Payload**：
```
偏移  长度  字段       类型  说明
0     4     freq_Hz    u32  PID 执行频率 (Hz)
```

**约束**：freq_Hz ∈ [100, 50000]，超出 NACK。

### 4.8 SET_OVERCURRENT (0x15)

**Payload**：
```
偏移  长度  字段       类型  说明
0     4     I_limit    f32  过流保护阈值 (nA)
```

**作用**：电流超过此值时硬件中断 + tip 快退。约束：I_limit ∈ [1, 200] nA。

### 4.9 MOVE_XYZ (0x20)

**Payload**：
```
偏移  长度  字段  类型  说明
0     4     x    f32   x DAC 电压 (V)
4     4     y    f32   y DAC 电压 (V)
8     4     z    f32   z DAC 电压 (V)
```

**约束**：
- 仅在 idle 模式允许，否则 NACK
- 各值 ∈ [-15, 15] V

### 4.10 SET_Z (0x21)

**Payload**：
```
偏移  长度  字段  类型  说明
0     4     z    f32   z DAC 电压 (V)
```

**约束**：z ∈ [z_min, z_max]，且仅在 idle 或 feedback_on 模式下允许。

### 4.11 RETRACT_Z (0x22)

**Payload** (6 字节)：
```
偏移  长度  字段    类型  说明
0     2     steps  u16   步进电机后退步数
2     4     speed  u32   后退速度 (steps/s)，0=用固件默认速度
```

**作用**：步进电机后退指定步数 (粗退) + z DAC 设为 z_min (细退)。

**速度字段**：
- speed = 0：固件用内部默认速度（向后兼容旧固件）
- speed > 0：固件按指定速度后退（需固件支持，否则忽略用默认）

### 4.12 SET_SCAN_MODE (0x25)

**用途**：设置扫描模式（恒流/恒高）。

**Payload**：
```
偏移  长度  字段  类型  说明
0     1     mode  u8   0=恒流 (PID 开), 1=恒高 (PID 关, z 固定)
```

**说明**：
- 恒流模式 (0)：扫描时 PID 反馈开启，z 随表面起伏自动调节，为默认模式
- 恒高模式 (1)：扫描时 PID 关闭，z 固定在当前值（或由 SET_Z 预设），仅适用于原子级平坦表面
- 恒高模式下表面突起可能导致 tip 撞击样品，请谨慎使用

### 4.13 SCAN_START (0x30)

**Payload**：
```
偏移  长度  字段         类型  说明
0     4     cx           f32  扫描中心 x (V)
4     4     cy           f32  扫描中心 y (V)
8     4     range_x      f32  x 方向峰峰值 (V)
12    4     range_y      f32  y 方向峰峰值 (V)
16    2     nx           u16  每行像素数
18    2     ny           u16  行数
20    4     speed_pps    u32  每秒采样点数 (points/s)
24    1     direction    u8   0=forward, 1=backward, 2=bidirectional
25    2     settle_ms    u16  行间 settling time (ms)
27    4     overshoot_V  f32  行端点预压电压 (V)，用于补偿压电迟滞
```

**约束**：
- 仅在 feedback_on 模式允许
- nx, ny ∈ [1, 2048]
- speed_pps ∈ [10, 10000]
- range_x, range_y ∈ (0, 30]
- 各像素点 x/y 电压经 cx ± range_x/2, cy ± range_y/2 计算后须 ∈ [-15, 15]

**执行流程**：
1. 接收参数，校验合法性
2. 切换到 scanning 模式
3. 生成扫描坐标序列
4. 逐行扫描：每行先到端点预压 → settling → 正向采集 → (双向时) 反向采集
5. 每行结束上报 SCAN_ROW
6. 全部完成上报 SCAN_END

### 4.14 SCAN_STOP / SCAN_PAUSE / SCAN_RESUME (0x31 / 0x32 / 0x33)

- **STOP**: 立即停止，已扫数据保留，切回 feedback_on 模式
- **PAUSE**: 暂停扫描，当前位置保持，可 RESUME
- **RESUME**: 从暂停位置继续

### 4.15 APPROACH_START (0x40)

**Payload**：
```
偏移  长度  字段              类型  说明
0     2     step_pulse        u16  每次步进脉冲数
2     4     step_speed_sps    u32  步进频率 (steps/s)
6     4     thresh_pre_nA     f32  预检电流阈值 (nA)，达此值切换到 DAC 细逼近
10    4     thresh_lock_nA    f32  锁定电流阈值 (nA)，达此值完成 approach
14    4     z_start_V         f32  DAC 细逼近起点电压 (V)
18    4     z_speed_Vps       f32  DAC 细逼近速度 (V/s)
22    2/4  max_steps         u16/u32  最大步进步数 (安全上限)
24/26 1     retry             u8   细逼近失败后步进再走的脉冲数
25/27 2     microstep         u16  TMC2209 微步配置 (1/2/4/8/16/32/64/128/256)
```

**max_steps 长度自适应编码 (v1.1)**：
- `max_steps ≤ 65535`：用 **u16** 编码，总长 **27** 字节 —— 与 v1.0 固件完全兼容
- `max_steps > 65535`：用 **u32** 编码，总长 **29** 字节 —— 需固件 v1.1+ 同步支持，
  旧固件会截断读取低 16 位（例如 1000000 → 16960，表现为 approach 提前停止）

**两段式状态机**：
```
PHASE_COARSE (步进粗逼近):
    循环:
        1. 步进 step_pulse 步
        2. 等待 1ms 稳定
        3. 读电流
        4. 上报 APPROACH_DATA (phase=0, step=当前步数, I=电流)
        5. 若 I >= thresh_pre → 切换 PHASE_FINE
        6. 若总步数 >= max_steps → 失败，上报 APPROACH_DONE(success=0)

PHASE_FINE (DAC 细逼近):
    循环:
        1. z DAC 增加 (z_speed * dt)
        2. 读电流
        3. 上报 APPROACH_DATA (phase=1, step=z_mV, I=电流)
        4. 若 I >= thresh_lock → 成功，上报 APPROACH_DONE(success=1)
        5. 若 z >= z_max → 失败，退回 z_start，步进 retry 步，回 PHASE_COARSE
```

### 4.16 SET_CALIB / SAVE_CALIB / LOAD_CALIB (0x50 / 0x51 / 0x52)

**SET_CALIB Payload**：
```
偏移  长度  字段      类型  说明
0     4     tia_R     f32   TIA 反馈电阻 (Ω)
4     4     x_nmV     f32   x 压电灵敏度 (nm/V)
8     4     y_nmV     f32   y 压电灵敏度 (nm/V)
12    4     z_nmV     f32   z 压电灵敏度 (nm/V)
```

- SET_CALIB 写入 RAM，立即生效
- SAVE_CALIB 持久化到 NVS
- LOAD_CALIB 从 NVS 加载并应用

### 4.17 GET_STATUS (0x60)

### 4.18 GET_FW_VERSION (0x61)

**用途**：查询设备固件版本号。

**Payload**：无

**下位机响应**：FW_VERSION (0x82)

**FW_VERSION Payload**：
```
偏移  长度       字段      类型     说明
0     N         version   char[]   ASCII 版本字符串，如 "STM_V1.2.3"
```

**说明**：
- 版本字符串长度不超过 64 字节
- 建议固件在启动时通过编译时宏注入版本号
- 上位机在连接成功后自动请求并显示在状态栏

上位机主动查询，下位机立即回 STATUS (0x90)。

---

## 5. 下行帧详解

### 5.1 STATUS (0x90)

**周期上报**：50~100 Hz 自动发送，无需请求。

**Payload**：
```
偏移  长度  字段         类型  说明
0     1     mode         u8   当前模式
1     4     cur_I_nA     f32  当前隧道电流 (nA)
5     4     z_V          f32  当前 z 电压 (V)
9     4     x_V          f32  当前 x 电压 (V)
13    4     y_V          f32  当前 y 电压 (V)
17    4     step_pos     u32  步进电机累计步数 (带符号)
21    1     err          u8   当前错误码 (0=无错误)
```

### 5.2 SCAN_ROW (0x91)

**Payload**：
```
偏移  长度       字段      类型        说明
0     2          y_idx     u16         行号 (0 ~ ny-1)
2     2          nx        u16         本行像素数
4     4*nx       I_data    f32[nx]     电流数组 (nA)
4+4*nx 4*nx      z_data    f32[nx]     z 反馈电压数组 (V)
```

**注意**：
- 双向扫描时，反向行的数据顺序仍按 x_idx 升序排列 (下位机负责重排)
- 单帧最大 payload = 8 + 8*2048 = 16488 字节，超出 LENGTH 上限 → nx 限制为 1024

### 5.3 SCAN_END (0x92)

**Payload**：
```
偏移  长度  字段           类型  说明
0     4     total_points   u32  实际采集的总点数
```

### 5.4 APPROACH_DATA (0x93)

**Payload**：
```
偏移  长度  字段          类型  说明
0     1     phase         u8   0=粗逼近, 1=细逼近
1     4     step_or_z     u32  phase=0: 步数; phase=1: z 电压 (mV, 有符号)
5     4     I_nA          f32  当前电流 (nA)
```

### 5.5 APPROACH_DONE (0x94)

**Payload**：
```
偏移  长度  字段           类型  说明
0     1     success        u8   1=成功, 0=失败
1     4     total_steps    u32  总步数
5     4     final_I_nA     f32  最终电流 (nA)
```

### 5.6 ERROR (0xA0)

**Payload**：
```
偏移  长度       字段       类型        说明
0     1          err_code   u8         错误码 (见第 6 节)
1     1          msg_len    u8         消息长度
2     msg_len    msg        char[]     ASCII 可读消息
```

### 5.7 ACK / NACK (0xB0 / 0xB1)

**ACK Payload**：
```
偏移  长度  字段       类型  说明
0     1     cmd_ack    u8   被确认的命令码
```

**NACK Payload**：
```
偏移  长度  字段       类型  说明
0     1     cmd_nack   u8   被拒绝的命令码
1     1     reason     u8   拒绝原因码 (见 6.3)
```

---

## 6. 错误码与诊断

### 6.1 硬件错误码 (err 字段 / ERROR 帧 err_code)

| 值   | 常量名                   | 含义                          | 严重程度 | 触发后行为 |
|------|--------------------------|-------------------------------|----------|------------|
| 0x00 | ERR_NONE                 | 无错误                        | -        | - |
| 0x01 | ERR_OVERCURRENT          | 过流 (tip 撞样品)             | 致命     | ESTOP 流程 + 上报 |
| 0x02 | ERR_Z_OVER_HIGH          | z 电压超上限                  | 严重     | ESTOP 流程 |
| 0x03 | ERR_Z_OVER_LOW           | z 电压超下限                  | 严重     | ESTOP 流程 |
| 0x04 | ERR_COMM_TIMEOUT         | 通信超时 (5s 未收到 PING)     | 严重     | ESTOP 流程 |
| 0x05 | ERR_ADC_FAULT            | ADC 故障 (读取异常)           | 严重     | ESTOP 流程 |
| 0x06 | ERR_DAC_FAULT            | DAC 故障 (写入异常)           | 严重     | ESTOP 流程 |
| 0x07 | ERR_STEP_OVERFLOW        | 步进超 max_steps              | 警告     | 停 approach，不上报 ESTOP |
| 0x08 | ERR_FINE_APPROACH_FAIL   | 细逼近失败 (z 达限未锁定)     | 警告     | 自动重试或上报 |
| 0x09 | ERR_SCAN_ABORT           | 扫描异常中止                  | 警告     | 停扫，保留数据 |
| 0x0A | ERR_MODE_INVALID         | 模式非法                      | 提示     | NACK |
| 0x0B | ERR_PARAM_OUT_OF_RANGE   | 参数越界                      | 提示     | NACK |
| 0x0C | ERR_NVS_WRITE            | NVS 写入失败                  | 警告     | NACK + 日志 |
| 0x0D | ERR_NVS_READ             | NVS 读取失败                  | 警告     | NACK + 使用默认值 |
| 0x0E | ERR_I2C_FAULT            | I2C 通信故障 (ADS8685 配置)   | 严重     | ESTOP 流程 |
| 0x0F | ERR_SPI_FAULT            | SPI 通信故障 (AD5761)         | 严重     | ESTOP 流程 |
| 0x10 | ERR_OVERTEMP             | MCU 过温                      | 严重     | ESTOP 流程 |
| 0x11 | ERR_TASK_WDT             | 任务看门狗触发                | 致命     | 系统重启 |
| 0x12 | ERR_LOOP_UNSTABLE        | PID 反馈环路发散              | 严重     | ESTOP 流程 |
| 0x13 | ERR_SCAN_TIMEOUT         | 扫描点采集超时                | 警告     | 停扫 |

### 6.2 协议层错误 (仅记录，不上报)

| 值   | 常量名                   | 含义                          |
|------|--------------------------|-------------------------------|
| 0x80 | PROTO_ERR_HEADER         | 帧头错误                      |
| 0x81 | PROTO_ERR_TAIL_MISSING   | 帧尾缺失                      |
| 0x82 | PROTO_ERR_CRC_MISMATCH   | CRC 校验失败                  |
| 0x83 | PROTO_ERR_LENGTH_INVALID | 长度字段非法                  |
| 0x84 | PROTO_ERR_CMD_UNKNOWN    | 未知命令码                    |
| 0x85 | PROTO_ERR_PAYLOAD_SIZE   | payload 长度与命令不符        |

### 6.3 NACK 拒绝原因码

| 值   | 常量名                        | 含义 |
|------|-------------------------------|------|
| 0x01 | NACK_INVALID_TRANSITION       | 模式切换非法 |
| 0x02 | NACK_PARAM_OUT_OF_RANGE       | 参数越界 |
| 0x03 | NACK_BUSY                     | 设备忙 (扫描中/approach 中) |
| 0x04 | NACK_NOT_IN_FEEDBACK_MODE     | 当前非 feedback 模式 |
| 0x05 | NACK_NOT_IN_IDLE_MODE         | 当前非 idle 模式 |
| 0x06 | NACK_CALIB_NOT_LOADED         | 标定未加载 |
| 0x07 | NACK_HARDWARE_FAULT           | 硬件故障 |
| 0x08 | NACK_UNKNOWN_CMD              | 未知命令 |
| 0x09 | NACK_INTERNAL_ERROR           | 内部错误 |

### 6.4 错误诊断建议

固件开发时建议：
1. **每个错误码对应一个日志条目**，包含触发位置 (文件:行号)、上下文变量
2. **ERROR 帧的 msg 字段**填入简短诊断信息，如 `"ADC read timeout at spi.c:142, reg=0x03"`
3. **致命错误触发 ESTOP 后**，下位机进入锁定状态，需上位机发送 `SET_MODE(idle)` 解锁
4. **协议层错误统计**：固件维护计数器 (header_err_count, crc_err_count 等)，可通过扩展命令查询

### 6.5 心跳超时与断连处理

- **心跳超时**：上位机通过 PING/PONG 机制监控连接状态。如果 `pong_timeout` (默认 3s) 内未收到 PONG，判定为设备无响应。
- **断连行为**：
  1. 上位机立即停止所有后台线程（读循环、心跳循环）。
  2. 清理所有等待 ACK 的命令队列，防止死锁。
  3. 通知 UI 进入"已断开"状态，停止所有扫描/Approach 操作。
  4. 如果 `auto_reconnect` 为 True，上位机将启动自动重连循环。
- **自动重连**：
  - 重连间隔由 `reconnect_interval` (默认 1s) 决定。
  - 重连时会向 UI 报告 "reconnecting" 状态。
  - 重连成功后恢复正常操作。
- **强制断开**：用户可以通过 UI 主动断开连接，此时 `_handle_disconnect` 会被调用，但不会触发自动重连（因为是主动行为）。

---

## 7. 状态机

### 7.1 主状态机

```
                  ┌──────────┐
                  │  IDLE    │ ◄────────────────────────┐
                  └────┬─────┘                          │
                       │ SET_MODE(1)                    │
                       ▼                                │
                  ┌──────────┐                          │
        ┌────────►│ FEEDBACK │◄─────────────┐           │
        │         └────┬─────┘              │           │
        │              │ SCAN_START         │ SCAN_STOP │
        │              ▼                    │           │
        │         ┌──────────┐              │           │
        │         │ SCANNING │──────────────┘           │
        │         └────┬─────┘                          │
        │              │ SCAN_END / SCAN_STOP          │
        │              ▼                                │
        │         ┌──────────┐                          │
        │         │ FEEDBACK │                          │
        │         └──────────┘                          │
        │                                                │
        │ SET_MODE(3)                                    │
        │              ┌──────────┐                      │
        └──────────────│ APPROACH │                      │
                       └────┬─────┘                      │
                            │ APPROACH_DONE              │
                            ▼                            │
                       ┌──────────┐                      │
                       │  IDLE    │──────────────────────┘
                       └──────────┘   (approach 成功后切 idle,
                                       等待上位机决定是否开反馈)

  任何状态 + ESTOP ──► IDLE (locked, 需 SET_MODE(idle) 解锁)
  任何状态 + 致命错误 ──► IDLE (locked)
```

### 7.2 Approach 子状态机

见 4.14 节。

---

## 8. 典型时序示例

### 8.1 启动 + 标定 + Approach + 扫描

```
上位机                              下位机
  │                                    │
  │ PING                               │
  ├───────────────────────────────────►│
  │                              PONG  │
  │◄───────────────────────────────────┤
  │                                    │
  │ SET_CALIB(tia_R=100M, x/y/z_nmV)   │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │ SAVE_CALIB                         │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │ SET_MODE(1=feedback)               │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │ SET_SETPOINT(1.0 nA)               │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │ SET_PID(0.1, 0.01, 0.001)          │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │ SET_MODE(3=approach)               │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │ APPROACH_START(...)                │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │              APPROACH_DATA(phase=0, step=1, I=0.01) │
  │◄───────────────────────────────────┤
  │              APPROACH_DATA(phase=0, step=2, I=0.02) │
  │◄───────────────────────────────────┤
  │              ...                   │
  │              APPROACH_DATA(phase=0, step=15, I=0.1) │
  │◄───────────────────────────────────┤
  │              APPROACH_DATA(phase=1, z=0mV, I=0.15)  │
  │◄───────────────────────────────────┤
  │              ...                   │
  │              APPROACH_DATA(phase=1, z=120mV, I=1.0) │
  │◄───────────────────────────────────┤
  │              APPROACH_DONE(success=1, steps=15, I=1.0) │
  │◄───────────────────────────────────┤
  │                                    │
  │ SET_MODE(1=feedback)   (approach 成功后自动或手动开反馈) │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │ SCAN_START(cx=0, cy=0, rx=1V, ...) │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
  │              SCAN_ROW(y=0, I=[...], z=[...])        │
  │◄───────────────────────────────────┤
  │              SCAN_ROW(y=1, ...)    │
  │◄───────────────────────────────────┤
  │              ...                   │
  │              SCAN_END(total=65536) │
  │◄───────────────────────────────────┤
  │                                    │
```

### 8.2 错误处理时序

```
上位机                              下位机
  │                                    │
  │              STATUS(err=0x01, ...) │ (过流触发)
  │◄───────────────────────────────────┤
  │              ERROR(0x01, "Overcurrent at z=2.3V")   │
  │◄───────────────────────────────────┤
  │                                    │
  │ (下位机自动 ESTOP: 反馈关, z=z_min, 模式=idle locked) │
  │                                    │
  │ SET_MODE(0=idle)  (解锁)           │
  ├───────────────────────────────────►│
  │                              ACK   │
  │◄───────────────────────────────────┤
  │                                    │
```

---

## 9. 数值范围参考

### 9.1 电压

| 信号 | 范围 | 分辨率 |
|---|---|---|
| x/y/z DAC (AD5761) | ±15 V | 30/65536 ≈ 0.458 mV/LSB |
| 隧道电流 ADC | 待确认 (假设 ±10V) | 20/65536 ≈ 0.305 mV/LSB |

### 9.2 电流换算

- TIA 反馈电阻 R = 100 MΩ
- I = V_adc / R
- 满量程 (±10V): ±100 nA
- 分辨率: 0.305mV / 100MΩ ≈ 3.05 pA/LSB

### 9.3 位移换算

| 信号 | 单位位移 | 说明 |
|---|---|---|
| 步进电机单脉冲 | 0.417 nm | 250μm / 20000 / 30 (杠杆) |
| 步进 1/16 微步 | 0.026 nm | 单脉冲 / 16 |
| z DAC 1 LSB | 待标定 | 0.458mV × z_nmV |

### 9.4 推荐起步参数

| 参数 | 推荐值 |
|---|---|
| PID 频率 | 1000 Hz |
| setpoint | 1.0 nA |
| 过流保护 | 10 nA |
| z_min | -15 V |
| z_max | 15 V |
| 扫描分辨率 | 256×256 |
| 扫描速度 | 1000 points/s |
| approach 预检阈值 | 0.1 nA |
| approach 锁定阈值 | 0.9 nA |
| approach 步进速度 | 100 steps/s |
| approach max_steps | 10000 |
| 微步 | 1/16 |

---

## 10. 版本历史

| 版本 | 日期 | 变更 |
|---|---|---|
| 1.0 | 2026-07-28 | 初始版本 |
