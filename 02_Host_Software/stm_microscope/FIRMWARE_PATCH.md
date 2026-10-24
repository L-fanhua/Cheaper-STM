# 固件修改方案:APPROACH_START 支持 max_steps > 65535 (协议 v1.1)

## 问题回顾

| 现象 | 原因 |
|---|---|
| approach 步数卡在 ~17000 停止 | 上位机发 `max_steps=1000000` (u32),固件按 u16 只读到低 16 位 `1000000 & 0xFFFF = 16960` |
| NACK(APPROACH_START) PARAM_OUT_OF_RANGE | 固件更新后仍未完整支持 29 字节格式(见下方排查清单) |

**v1.1 协议规则(长度自适应,见 PROTOCOL.md 4.15)**:
- `max_steps ≤ 65535` → u16 编码,**27 字节**(与 v1.0 完全兼容)
- `max_steps > 65535` → u32 编码,**29 字节**
- `retry`/`microstep` 用**尾部相对偏移**定位(两种格式下位置不同)

```
共同前缀 (0~21 字节):
偏移 0:  step_pulse      u16
偏移 2:  step_speed_sps  u32
偏移 6:  thresh_pre_nA   f32
偏移 10: thresh_lock_nA  f32
偏移 14: z_start_V       f32
偏移 18: z_speed_Vps     f32

u16 格式 (27B)          u32 格式 (29B)
偏移 22: max_steps u16    偏移 22: max_steps u32
偏移 24: retry     u8     偏移 26: retry     u8
偏移 25: microstep u16    偏移 27: microstep u16
```

---

## 修改点 1:APPROACH_START 解析(长度自适应)

不要再用固定 27 字节的 packed 结构体直接强转,改为按长度分派:

```c
static void handle_approach_start(const uint8_t *p, uint16_t len)
{
    uint32_t max_steps;
    uint8_t  retry;
    uint16_t microstep;

    if (len == 27) {            /* v1.0: max_steps = u16 */
        max_steps = p[22] | ((uint32_t)p[23] << 8);
        retry     = p[24];
        microstep = p[25] | ((uint16_t)p[26] << 8);
    } else if (len == 29) {     /* v1.1: max_steps = u32 */
        max_steps = p[22] | ((uint32_t)p[23] << 8)
                  | ((uint32_t)p[24] << 16) | ((uint32_t)p[25] << 24);
        retry     = p[26];
        microstep = p[27] | ((uint16_t)p[28] << 8);
    } else {
        send_nack(CMD_APPROACH_START, NACK_PARAM_OUT_OF_RANGE);  /* 长度非法 */
        return;
    }

    /* 前缀字段两种格式完全一致 */
    uint16_t step_pulse = p[0] | ((uint16_t)p[1] << 8);
    uint32_t step_speed = p[2] | ((uint32_t)p[3] << 8)
                        | ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 24);
    float thresh_pre  = le_f32(p + 6);
    float thresh_lock = le_f32(p + 10);
    float z_start     = le_f32(p + 14);
    float z_speed     = le_f32(p + 18);

    /* ... 校验 + 启动状态机 ... */
}

/* ESP32 (Xtensa/ESP32-S3) 本身是小端,f32 可直接 memcpy: */
static inline float le_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}
```

## 修改点 2:参数校验(max_steps 上限放宽)

```c
/* 旧: if (max_steps < 1 || max_steps > 65535) → NACK      ← 最常见的漏改点!
 * 新: */
if (max_steps < 1 || max_steps > 1000000) {
    send_nack(CMD_APPROACH_START, NACK_PARAM_OUT_OF_RANGE);
    return;
}
```

其余字段校验与 v1.0 相同:
`step_pulse ∈ [1,65535]`、`step_speed ∈ [1,100000]`、`0 ≤ thresh_pre < thresh_lock ≤ 100`、
`z_start ∈ [-15,15]`、`z_speed ∈ (0,100]`、`microstep ∈ {1,2,4,8,16,32,64,128,256}`。

## 修改点 3:变量类型(u16 → u32)

```c
/* 旧 */
static uint16_t approach_max_steps;
static uint16_t approach_total_steps;

/* 新 —— 所有涉及步数计数/上限的变量都必须是 32 位 */
static uint32_t approach_max_steps;
static uint32_t approach_total_steps;
```

## 修改点 4:状态机比较逻辑(PHASE_COARSE)

```c
approach_total_steps += step_pulse;                    /* u32 累加,不回绕 */
if (approach_total_steps >= approach_max_steps) {
    /* ERR_STEP_OVERFLOW → APPROACH_DONE(success=0) */
}
```

## 修改点 5:APPROACH_DATA 上报(step 字段本来就是 u32)

协议 0x93 payload = `phase u8 + step u32 + I f32` 共 9 字节(上位机按 `<BIf>` 解析)。
确保上报时 step 用 32 位计数器打包:

```c
uint8_t buf[9];
buf[0] = 0x00;                                  /* PHASE_COARSE */
buf[1] = (approach_total_steps      ) & 0xFF;   /* u32 小端 */
buf[2] = (approach_total_steps >>  8) & 0xFF;
buf[3] = (approach_total_steps >> 16) & 0xFF;
buf[4] = (approach_total_steps >> 24) & 0xFF;
memcpy(buf + 5, &current_nA, 4);                /* f32 小端 */
send_frame(REPLY_APPROACH_DATA, buf, 9);
```

APPROACH_DONE (0x94) 若含 total_steps 字段,同样确认是 u32。

---

## 排查清单:为什么你上次更新固件后仍然 NACK

按命中率排序,逐条检查固件中 APPROACH_START 的处理路径:

1. **payload 长度预检查写死** `len != 27` → 收到 29 字节直接 NACK。
   改为 `len != 27 && len != 29` 才拒绝(或按上面分派逻辑)。
2. **max_steps 范围校验没改**:仍保留 `if (max_steps > 65535) → NACK`。
   解析改了 u32,但校验还卡 65535(只改了一半的典型症状)。
3. **packed 结构体还是 27 字节版**:按结构体强转解析 29 字节帧 → 字段全部错位
   → `microstep` 读到非法值(如 1280)→ 触发 NACK。必须弃用固定结构体,改按偏移解析。
4. **dispatch 表里有第二处长度校验**:命令分发层(如 `case CMD_APPROACH_START:`)
   与 handler 内部各有一处长度检查,只改了其中一处。

## 联调验证步骤

1. **改完固件烧录后**,建议把固件版本号升级(如 `1.0.0` → `1.1.0`),
   上位机顶部状态条会显示,便于确认烧的是新固件。
2. 上位机设 `max_steps = 60000` → TX 应为 **27 字节** → 固件 ACK,跑满 60000 或锁定。
3. 上位机设 `max_steps = 1000000` → TX 应为 **29 字节** → 固件 ACK,
   且**不再**在 16960 步停止(检查 UI 顶部「步: 」计数可超过 65535)。
4. 若仍 NACK:对照 UI 日志 `[TX ] APPROACH_START payload=<hex>` 的 hex 长度
   (54 字符 = 27B,58 字符 = 29B),再回到排查清单逐条核对。

---

# 固件实现清单 (二):扫描数据上报 SCAN_ROW (0x91)

上位机全链路已就绪 (UI / 协议 / 数据引擎 / 图像 / 示波器),固件按下述实现后即可出图。
**实测结论 (2026-10-24)**:SCAN_START 已被固件接受且流程跑完 (收到 SCAN_END),
但全程无任何 SCAN_ROW 上行 → 图像 0 行空白。

## 协议变化:MAX_PAYLOAD 1024 → 8200 (v1.1)

为容纳 nx=1024 的整行数据 (8 + 8×1024 = 8200 字节):
- **固件帧收发缓冲区至少 8210 字节** (8200 + 帧头 2 + 长度 2 + 帧尾 1 + 余量)
- LENGTH 合法范围变为 **[3, 8203]**
- 其余命令 payload 均远小于此,不受影响

## SCAN_ROW 发送时机与格式 (PROTOCOL.md 5.2)

每扫完一行 (y 步进到下一行**之前**) 发一帧,payload = 4 + 8×nx 字节:

```
偏移 0     2     y_idx   u16   行号 (0 ~ ny-1)
偏移 2     2     nx      u16   本行像素数
偏移 4     4*nx  I_data  f32[] 隧道电流 (nA)
偏移 4+4nx  4*nx  z_data f32[] z 反馈电压 (V)
```

## 参考实现 (ESP32, C)

```c
static uint8_t row_buf[4 + 8 * 1024];        /* 8196 字节, 静态分配 */

void send_scan_row(uint16_t y_idx, uint16_t nx,
                   const float *I, const float *z)
{
    row_buf[0] = y_idx & 0xFF;  row_buf[1] = (y_idx >> 8) & 0xFF;
    row_buf[2] = nx & 0xFF;     row_buf[3] = (nx >> 8) & 0xFF;
    memcpy(row_buf + 4, I, 4 * nx);               /* ESP32 小端, f32 直接拷 */
    memcpy(row_buf + 4 + 4 * nx, z, 4 * nx);
    send_frame(REPLY_SCAN_ROW /* 0x91 */, row_buf, 4 + 8 * (uint32_t)nx);
}

/* send_frame = 0x55 0xAA + LENGTH(u16, = 1+payload_len+2) + CMD + payload
 *            + CRC16_MODBUS(CMD+payload, 小端) + 0x0A, 一次性写入 USB CDC */
```

## 注意事项

1. 全部行扫完后发 **SCAN_END (0x92, payload: total_points u32)**
2. 恒高模式下 z_data 填当前固定 z 电压值
3. 双向扫描时反向行数据按 x 升序重排后再发 (协议 5.2)
4. 吞吐估算:USB CDC ~1MB/s;nx=256 一行 2056B,1000pps 扫一行约 0.26s,带宽充裕
5. 上位机验证方法:扫描时 UI 日志应出现 `[RX ] SCAN_ROW`,stm.log 出现
   `first SCAN_ROW received: y_idx=0 nx=...`;完成时显示
   `[扫描完成] 总点数 X | 图像 256/256 行`
