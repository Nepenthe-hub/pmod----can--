#ifndef __CH9434_H
#define __CH9434_H

#include "stm32f10x.h"

/* ========================================================= */
/* 1. CH9434D SPI 接口控制寄存器及指令定义                       */
/* ========================================================= */
#define CH9434_SPI_WRITE_BIT    0x80  // SPI 写操作时将地址最高位置 1

/* CH9434D 接口控制寄存器 (8位操作) */
#define CH9434_REG_IO_SEL_FUN   0x45  // 引脚功能复用使能寄存器
#define CH9434_REG_CAN          0x46  // CAN 寄存器读写入口地址

/* 默认引脚控制编号定义 (用于 IO_SEL_FUN_CFG) */
#define CH9434_PIN_UART1        1
#define CH9434_PIN_UART2        2
#define CH9434_PIN_UART3        3
#define CH9434_PIN_UART4        4
#define CH9434_PIN_HSE          5     // 外部高速晶振 XI XO
#define CH9434_PIN_CAN          6     // CAN_TX 和 CAN_RX 引脚

/* ========================================================= */
/* 2. CH9434D CAN 内部控制器寄存器 (通过 32 位读写访问)      */
/* ========================================================= */
/* 控制和状态寄存器 */
#define CAN_CTLR        0x00  // CAN 主控制寄存器
#define CAN_STATR       0x01  // CAN 主状态寄存器
#define CAN_TSTATR      0x02  // CAN 发送状态寄存器
#define CAN_RFIFO0      0x03  // CAN 接收 FIFO0 控制和状态寄存器
#define CAN_RFIFO1      0x04  // CAN 接收 FIFO1 控制和状态寄存器
#define CAN_INTENR      0x05  // CAN 中断使能寄存器
#define CAN_ERRSR       0x06  // CAN 错误状态寄存器
#define CAN_BTIMR       0x07  // CAN 位时序寄存器

/* CAN 发送邮箱 0 寄存器 */
#define CAN_TXMIR0      0x0B  // CAN 发送邮箱0 标识符寄存器
#define CAN_TXMDTR0     0x0C  // CAN 发送邮箱0 数据长度和时间戳寄存器
#define CAN_TXMDLR0     0x0D  // CAN 发送邮箱0 低字节数据寄存器
#define CAN_TXMDHR0     0x0E  // CAN 发送邮箱0 高字节数据寄存器

/* CAN 接收 FIFO0 读取寄存器 */
#define CAN_RXMIR0      0x17  // CAN 接收 FIFO0 邮箱标识符寄存器
#define CAN_RXMDTR0     0x18  // CAN 接收 FIFO0 邮箱数据长度和时间戳
#define CAN_RXMDLR0     0x19  // CAN 接收 FIFO0 邮箱低字节数据
#define CAN_RXMDHR0     0x1A  // CAN 接收 FIFO0 邮箱高字节数据

/* 过滤器寄存器 */
#define CAN_FCTLR       0x1F  // CAN 过滤器主控制寄存器
#define CAN_FMCFGR      0x20  // CAN 过滤器模式寄存器
#define CAN_FSCFGR      0x21  // CAN 过滤器位宽寄存器
#define CAN_FAFIFOR     0x22  // CAN 过滤器 FIFO 关联寄存器
#define CAN_FWR         0x23  // CAN 过滤器激活寄存器
#define CAN_F0R1        0x24  // CAN 过滤器组0 寄存器1
#define CAN_F0R2        0x25  // CAN 过滤器组0 寄存器2

/* ========================================================= */
/* 3. CAN 控制器关键位定义                                   */
/* ========================================================= */
/* CAN_CTLR (0x00) 寄存器位 */
#define CAN_CTLR_RST    (1UL << 15) // 控制器复位请求
#define CAN_CTLR_NART   (1UL << 4)  // 禁止发送自动重传
#define CAN_CTLR_SLEEP  (1UL << 1)  // 睡眠模式请求位
#define CAN_CTLR_INRQ   (1UL << 0)  // 初始化模式请求位

/* CAN_STATR (0x01) 寄存器位 */
#define CAN_STATR_SLAK  (1UL << 1)  // 睡眠模式指示位
#define CAN_STATR_INAK  (1UL << 0)  // 初始化模式指示位

/* CAN_TSTATR (0x02) 寄存器位 */
#define CAN_TSTATR_TME0 (1UL << 26) // 邮箱0 空标志位
#define CAN_TSTATR_TXOK0 (1UL << 1) // 邮箱0 发送成功标志位
#define CAN_TSTATR_RQCP0 (1UL << 0) // 邮箱0 请求完成标志位

/* ========================================================= */
/* 4. CAN 消息数据结构                                       */
/* ========================================================= */
typedef struct {
    uint32_t id;        // CAN ID (标准帧11位 或 扩展帧29位)
    uint8_t  dlc;       // 数据长度 (0-8)
    uint8_t  data[8];   // CAN 帧数据
    uint8_t  extended;  // 扩展帧标志 (0 = 标准帧, 1 = 扩展帧)
    uint8_t  rtr;       // 远程帧标志 (0 = 数据帧, 1 = 远程帧)
} CAN_Message;

/* ========================================================= */
/* 5. 函数声明                                               */
/* ========================================================= */
void     CH9434_Write8(uint8_t addr, uint8_t val);
uint32_t CH9434_AccessCANReg32(uint8_t can_reg_addr, uint32_t write_val, uint8_t is_write);
void     CH9434_EnableDefaultPin(uint8_t pin_addr);
void     CH9434_Init_CAN(uint32_t baudrate);
uint8_t  CH9434_SendCANMessage(CAN_Message *msg);
uint8_t  CH9434_ReceiveCANMessage(CAN_Message *msg);
uint8_t  CH9434_GetMode(void);
uint8_t  CH9434_ReadReg(uint8_t can_reg_addr);

#endif /* __CH9434_H */

