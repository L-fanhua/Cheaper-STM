/**
 * @file AD5761.cpp
 * @brief AD5761R/AD5721R 16/12-bit 单通道 DAC 驱动实现 (ESP-IDF 风格, 简单 C 接口)
 * @date 2026-07-27
 */

#include "AD5761.h"

/* 创建 SPI 设备句柄 */
spi_device_handle_t vspi_handle;

void ad5761_pull_up_ss(char cs) {
    /* 拉高片选, 结束 SPI 传输 */
    switch (cs) {
        case 0:
            gpio_set_level((gpio_num_t)csPin0, 1);
            break;

        case 1:
            gpio_set_level((gpio_num_t)csPin1, 1);
            break;

        case 2:
            gpio_set_level((gpio_num_t)csPin2, 1);
            break;

        case 3:
            gpio_set_level((gpio_num_t)csPin3, 1);
            break;

        case 4:
            gpio_set_level((gpio_num_t)csPin0, 1);
            gpio_set_level((gpio_num_t)csPin1, 1);
            gpio_set_level((gpio_num_t)csPin2, 1);
            gpio_set_level((gpio_num_t)csPin3, 1);
            break;
    }
}

void ad5761_pull_down_ss(char cs) {
    /* 拉低片选, 开始 SPI 传输 */
    switch (cs) {
        case 0:
            gpio_set_level((gpio_num_t)csPin0, 0);
            break;

        case 1:
            gpio_set_level((gpio_num_t)csPin1, 0);
            break;

        case 2:
            gpio_set_level((gpio_num_t)csPin2, 0);
            break;

        case 3:
            gpio_set_level((gpio_num_t)csPin3, 0);
            break;

        case 4:
            gpio_set_level((gpio_num_t)csPin0, 0);
            gpio_set_level((gpio_num_t)csPin1, 0);
            gpio_set_level((gpio_num_t)csPin2, 0);
            gpio_set_level((gpio_num_t)csPin3, 0);
            break;
    }
}

/* 初始化模块 */
void ad5761_init() {
    ad5761_SPI_init();
    gpio_set_direction((gpio_num_t)csPin0, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)csPin1, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)csPin2, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)csPin3, GPIO_MODE_OUTPUT);

    ad5761_write(CMD_SW_FULL_RESET, 0, 0);
    ad5761_write(CMD_WR_CTRL_REG, CONTROL_REG_VAL, 0);
    ad5761_write(CMD_SW_FULL_RESET, 0, 1);
    ad5761_write(CMD_WR_CTRL_REG, CONTROL_REG_VAL, 1);
    ad5761_write(CMD_SW_FULL_RESET, 0, 2);
    ad5761_write(CMD_WR_CTRL_REG, CONTROL_REG_VAL, 2);
    ad5761_write(CMD_SW_FULL_RESET, 0, 3);
    ad5761_write(CMD_WR_CTRL_REG, CONTROL_REG_VAL, 3);

    ad5761_write(CMD_WR_UPDATE_DAC_REG, 32768, 0);
    ad5761_write(CMD_WR_UPDATE_DAC_REG, 32768, 1);
    ad5761_write(CMD_WR_UPDATE_DAC_REG, 32768, 2);
    ad5761_write(CMD_WR_UPDATE_DAC_REG, 32768, 3);

    /* 全部待机 */
    ad5761_pull_up_ss(4);
}

/* SPI 外设初始化 */
void ad5761_SPI_init() {
    /* 创建参数结构 (字段顺序须与 spi_bus_config_t 声明顺序一致) */
    spi_bus_config_t buscfg = {
        .mosi_io_num = AD5761_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = AD5761_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    /* 初始化 SPI3 */
    spi_bus_initialize(AD5761_USED_SPI, &buscfg, SPI_DMA_CH_AUTO);

    /* 设备接口配置 (字段顺序须与 spi_device_interface_config_t 声明顺序一致) */
    spi_device_interface_config_t devcfg0 = {
        .mode = 2,                      /* SPI mode 2 */
        .clock_speed_hz = AD5761_CLOCK_RATE,
        .spics_io_num = -1,             /* 软件控制片选 */
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    /* 添加 SPI 设备, 返回句柄 */
    spi_bus_add_device(AD5761_USED_SPI, &devcfg0, &vspi_handle);
}

void ad5761_write(uint8_t reg_addr_cmd, uint16_t reg_data, char cs) {
    /* 拉低选中片选 */
    ad5761_pull_down_ss(cs);

    /* 准备 SPI 数据 */
    uint8_t tx_data[3];

    tx_data[0] = reg_addr_cmd;
    tx_data[1] = (reg_data & 0xFF00) >> 8;
    tx_data[2] = (reg_data & 0x00FF) >> 0;

    /* 事务结构 (字段顺序须与 spi_transaction_t 声明顺序一致) */
    spi_transaction_t t = {
        .length = 3 * 8,
        .tx_buffer = tx_data,
    };

    /* 传输 SPI 数据 */
    spi_device_polling_transmit(vspi_handle, &t);

    /* 拉高选中片选 */
    ad5761_pull_up_ss(cs);
}