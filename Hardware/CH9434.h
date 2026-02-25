#ifndef __CH9434_H
#define __CH9434_H

#include "stm32f10x.h"

/* CH9434 SPI 命令码 */
#define CH9434_CMD_WRITE      0x98  // 写寄存器
#define CH9434_CMD_READ       0x99  // 读寄存器

/* CAN 控制器寄存器 */
#define CH9434_REG_CANCTRL    0x0F  // CAN控制寄存器
#define CH9434_REG_CANSTAT    0x0E  // CAN状态寄存器
#define CH9434_REG_CANINTE    0x2B  // CAN中断使能
#define CH9434_REG_CANINTF    0x2C  // CAN中断标志

/* CAN配置寄存器 */
#define CH9434_REG_CNF1       0x2A  // 配置寄存器1(同步段)
#define CH9434_REG_CNF2       0x29  // 配置寄存器2(传播段+相位段1)
#define CH9434_REG_CNF3       0x28  // 配置寄存器3(相位段2)

/* CAN发送缓冲区 */
#define CH9434_REG_TXB0CTRL   0x30  // 发送缓冲区0控制
#define CH9434_REG_TXB0SIDH   0x31  // 发送缓冲区0标准ID高字节
#define CH9434_REG_TXB0SIDL   0x32  // 发送缓冲区0标准ID低字节
#define CH9434_REG_TXB0EID8   0x33  // 发送缓冲区0扩展ID高字节
#define CH9434_REG_TXB0EID0   0x34  // 发送缓冲区0扩展ID低字节
#define CH9434_REG_TXB0DLC    0x35  // 发送缓冲区0数据长度
#define CH9434_REG_TXB0D0     0x36  // 发送缓冲区0数据0

/* CAN接收缓冲区 */
#define CH9434_REG_RXB0CTRL   0x60  // 接收缓冲区0控制
#define CH9434_REG_RXB0SIDH   0x61  // 接收缓冲区0标准ID高字节
#define CH9434_REG_RXB0SIDL   0x62  // 接收缓冲区0标准ID低字节
#define CH9434_REG_RXB0DLC    0x65  // 接收缓冲区0数据长度
#define CH9434_REG_RXB0D0     0x66  // 接收缓冲区0数据0

/* CAN控制寄存器位定义 */
#define CANCTRL_REQOP_NORMAL  0x00  // 正常模式
#define CANCTRL_REQOP_SLEEP   0x20  // 睡眠模式
#define CANCTRL_REQOP_LOOPBACK 0x40 // 回环模式
#define CANCTRL_REQOP_LISTENONLY 0x60 // 只听模式
#define CANCTRL_REQOP_CONFIG  0x80  // 配置模式

/* CAN中断位 */
#define CANINTF_RX0IF         0x01  // 接收缓冲区0中断
#define CANINTF_TX0IF         0x04  // 发送缓冲区0中断

/* CAN状态位 */
#define CANSTAT_OPMOD_MASK    0xE0  // 操作模式掩码

/* 系统参数 */
#define CH9434_CLOCK_FREQ     8000000UL  // 8MHz晶振 

/* CAN数据结构 */
typedef struct {
    uint32_t id;        // CAN ID
    uint8_t  dlc;       // 数据长度(0-8)
    uint8_t  data[8];   // 数据
    uint8_t  extended;  // 扩展帧标志(0=标准帧, 1=扩展帧)
} CAN_Message;

/* 函数声明 */
void CH9434_Init_CAN(uint32_t bitrate);
void CH9434_WriteReg(uint8_t addr, uint8_t val);
uint8_t CH9434_ReadReg(uint8_t addr);
uint8_t CH9434_SendCANMessage(CAN_Message *msg);
uint8_t CH9434_ReceiveCANMessage(CAN_Message *msg);
uint8_t CH9434_Available(void);
void CH9434_SetMode(uint8_t mode);

#endif
