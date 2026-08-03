/**
 * @file register.h
 * @brief 共享寄存器/命令定义 (AD57X1 DAC + ADS8689 ADC)
 */

#ifndef REGISTER_H
#define REGISTER_H

/* ================================================================== *
 *  Input Shift Register Commands for AD57X1 (AD5761/AD5721)
 *  位于 24-bit 帧的 DB[19:16] (A3~A0)
 * ================================================================== */
#define CMD_NOP                 0x0   ///< 无操作
#define CMD_WR_TO_INPUT_REG     0x1   ///< 写入输入寄存器 (不更新输出)
#define CMD_UPDATE_DAC_REG      0x2   ///< 从输入寄存器更新 DAC 寄存器
#define CMD_WR_UPDATE_DAC_REG   0x3   ///< 写入并更新 DAC 寄存器
#define CMD_WR_CTRL_REG         0x4   ///< 写控制寄存器
#define CMD_NOP_ALT_1           0x5   ///< 无操作
#define CMD_NOP_ALT_2           0x6   ///< 无操作
#define CMD_SW_DATA_RESET       0x7   ///< 软件数据复位
#define CMD_RESERVED            0x8   ///< 保留
#define CMD_DIS_DAISY_CHAIN     0x9   ///< 禁用菊花链
#define CMD_RD_INPUT_REG        0xA   ///< 回读输入寄存器
#define CMD_RD_DAC_REG          0xB   ///< 回读 DAC 寄存器
#define CMD_RD_CTRL_REG         0xC   ///< 回读控制寄存器
#define CMD_NOP_ALT_3           0xD   ///< 无操作
#define CMD_NOP_ALT_4           0xE   ///< 无操作
#define CMD_SW_FULL_RESET       0xF   ///< 软件完全复位

/* ================================================================== *
 *  AD57X1 控制寄存器设置 (DB[15:0])
 *    DB[15:11] = 无关
 *    DB[10:9]  = CV  CLEAR 电压选择 (00=零, 01=中间, 10/11=满量程)
 *    DB8       = OVR 5% 超范围
 *    DB7       = B2C 双极性二进制补码编码
 *    DB6       = ETS 热关断报警
 *    DB5       = IRO 内部基准电压源使能
 *    DB[4:3]   = PV  上电电压选择 (00=零, 01=中间, 10/11=满量程)
 *    DB[2:0]   = RA  输出范围选择 (000=±10V, ...)
 *
 *  CONTROL_REG_VAL:
 *    ETS=1 (热关断使能), PV=01 (上电中间电平), RA=000 (±10V)
 *    = 0b0000 0000 0100 1000 = 0x0048
 * ================================================================== */
#define CONTROL_REG_VAL 0b0000000001001000   ///< 无内部基准版本
#define CONTROL_REG_VAL_R 0b0000000001101000 ///< 带内部基准版本 (IRO=1)

/* ================================================================== *
 *  ADS8689 寄存器地址
 * ================================================================== */
#define ADS8689_DEVICE_ID_REG   0x00
#define ADS8689_RST_PWRCTL_REG  0x04
#define ADS8689_SDI_CTL_REG     0x08
#define ADS8689_SDO_CTL_REG     0x0C
#define ADS8689_DATAOUT_CTL_REG 0x10
#define ADS8689_RANGE_SEL_REG   0x14
#define ADS8689_ALARM_REG       0x20
#define ADS8689_ALARM_H_TH_REG  0x24
#define ADS8689_ALARM_L_TH_REG  0x28

/* ================================================================== *
 *  ADS8689 SPI 命令
 * ================================================================== */
#define ADS8689_NOP         0b0000000
#define ADS8689_CLEAR_HWORD 0b1100000
#define ADS8689_READ_HWORD  0b1100100
#define ADS8689_READ        0b0100100
#define ADS8689_WRITE_FULL  0b1101000   ///< 写 16-bit 到寄存器
#define ADS8689_WRITE_MS    0b1101001   ///< 写高 8-bit
#define ADS8689_WRITE_LS    0b1101010   ///< 写低 8-bit

#endif /* REGISTER_H */