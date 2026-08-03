/**
 * @file AD5761.h
 * @brief AD5761R/AD5721R 16/12-bit 单通道 DAC 驱动 (ESP-IDF 风格, 简单 C 接口)
 * @date 2026-07-27
 *
 * 简化版驱动, 管理 4 路 DAC, 共用同一 SPI 总线, 通过 4 个 GPIO 片选。
 */

#ifndef AD5761_H
#define AD5761_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "register.h"

/* 选择使用的 SPI 外设 */
#define AD5761_USED_SPI    SPI3_HOST

/* 选择用作 SPI 的 GPIO 引脚 (ESP32-S3 常用安全引脚) */
#define AD5761_MOSI_PIN    11
#define AD5761_CLK_PIN     12

/* 定义 SPI 时钟频率 (GPIO 矩阵路由引脚建议 ≤40MHz) */
#define AD5761_CLOCK_RATE  40000000

/* 定义片选引脚 (ESP32-S3 常用安全引脚, 均在 GPIO1~18 范围内) */
#define csPin0  7
#define csPin1  8
#define csPin2  9
#define csPin3  10

/**
 * @brief 拉高片选 (结束 SPI 传输)
 * @param cs 0~3 选择单路 DAC; 4 选择全部 DAC
 */
void ad5761_pull_up_ss(char cs);

/**
 * @brief 拉低片选 (开始 SPI 传输)
 * @param cs 0~3 选择单路 DAC; 4 选择全部 DAC
 */
void ad5761_pull_down_ss(char cs);

/** @brief 初始化模块: SPI、片选引脚、复位、写控制寄存器、输出中点 */
void ad5761_init();

/** @brief SPI 外设初始化 */
void ad5761_SPI_init();

/**
 * @brief 向指定 DAC 写入 24-bit 命令帧
 * @param reg_addr_cmd 命令/寄存器地址 (DB[19:16])
 * @param reg_data     16-bit 数据 (DB[15:0])
 * @param cs           0~3 选择单路 DAC; 4 选择全部 DAC
 */
void ad5761_write(uint8_t reg_addr_cmd, uint16_t reg_data, char cs);

#endif /* AD5761_H */