/**
 * @file version.h
 * @brief 固件版本号 (每次修改后递增)
 *
 * 规范:
 *   主版本.次版本.修订号
 *   主版本: 协议不兼容变更
 *   次版本: 新增功能/命令 (向下兼容)
 *   修订号: Bug 修复/小幅调整
 *
 * 上位机可通过 GET_FW_VERSION (0x61) 查询此字符串。
 */

#ifndef VERSION_H
#define VERSION_H

/* 当前固件版本 (GET_FW_VERSION 返回此 ASCII 字符串) */
#define FW_VERSION_STRING  "STM_V1.3.0"

/* 协议版本 */
#define PROTO_VERSION_MAJOR  1
#define PROTO_VERSION_MINOR  1

#endif /* VERSION_H */