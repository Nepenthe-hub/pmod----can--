#include "CH9434.h"
#include "SPI.h"

/**
 * @brief  CH9434D 通过 SPI 写 8 位寄存器
 */
void CH9434_Write8(uint8_t addr, uint8_t val)
{
    SPI_CS_LOW();
    BSP_SPI_SwapByte(addr | CH9434_SPI_WRITE_BIT);
    for(volatile int d=0; d<30; d++);  // >2us @72MHz
    BSP_SPI_SwapByte(val);
    SPI_CS_HIGH();
    for(volatile int d=0; d<30; d++);  // >2us @72MHz
}

/**
 * @brief  CH9434D 读/写 32位 CAN 寄存器
 * @param  can_reg_addr: CAN 内部寄存器地址
 * @param  write_val: 要写入的 32 位值
 * @param  is_write: 1 为写操作，0 为读操作
 * @retval 读操作时返回 32 位寄存器值；写操作时返回 0
 */
uint32_t CH9434_AccessCANReg32(uint8_t can_reg_addr, uint32_t write_val, uint8_t is_write)
{
    uint32_t read_val = 0;

    SPI_CS_LOW();
    if (is_write) {
        BSP_SPI_SwapByte(CH9434_REG_CAN | CH9434_SPI_WRITE_BIT); // 0xC6
    } else {
        BSP_SPI_SwapByte(CH9434_REG_CAN);                        // 0x46
    }

    for(volatile int d=0; d<30; d++);  // >2us @72MHz

    BSP_SPI_SwapByte(can_reg_addr);

    if (is_write) {
        BSP_SPI_SwapByte((uint8_t)(write_val & 0xFF));
        BSP_SPI_SwapByte((uint8_t)((write_val >> 8) & 0xFF));
        BSP_SPI_SwapByte((uint8_t)((write_val >> 16) & 0xFF));
        BSP_SPI_SwapByte((uint8_t)((write_val >> 24) & 0xFF));
    } else {
        read_val  = (uint32_t)BSP_SPI_SwapByte(0xFF);
        read_val |= (uint32_t)BSP_SPI_SwapByte(0xFF) << 8;
        read_val |= (uint32_t)BSP_SPI_SwapByte(0xFF) << 16;
        read_val |= (uint32_t)BSP_SPI_SwapByte(0xFF) << 24;
    }

    SPI_CS_HIGH();
    for(volatile int d=0; d<20; d++);

    return read_val;
}

/**
 * @brief  启用 CH9434D 的默认引脚功能
 * @param  pin_addr: 默认引脚地址 (例如 CAN_TX_RX 为 6)
 */
void CH9434_EnableDefaultPin(uint8_t pin_addr)
{
    uint8_t status = 0;

    /* 发送开启命令 */
    SPI_CS_LOW();
    BSP_SPI_SwapByte(CH9434_REG_IO_SEL_FUN | CH9434_SPI_WRITE_BIT); // 0xC5
    for(volatile int d=0; d<10; d++);
    BSP_SPI_SwapByte(0x03);      // 子命令：0x03 开启默认引脚使能
    BSP_SPI_SwapByte(pin_addr);  // 默认引脚地址
    BSP_SPI_SwapByte(0x01);      // 1: 使能
    BSP_SPI_SwapByte(0xA5);      // 标志位写入 0xA5
    SPI_CS_HIGH();

    for(volatile int d=0; d<20; d++);

    /* 轮询等待芯片应答 0x5A */
    uint16_t timeout = 1000;
    while(timeout--) {
        SPI_CS_LOW();
        BSP_SPI_SwapByte(CH9434_REG_IO_SEL_FUN); // 读操作 0x45
        for(volatile int d=0; d<10; d++);
        BSP_SPI_SwapByte(0xFF);
        BSP_SPI_SwapByte(0xFF);
        status = BSP_SPI_SwapByte(0xFF);
        SPI_CS_HIGH();

        if(status == 0x5A) break;
        for(volatile int d=0; d<20; d++);
    }
}

/**
 * @brief  读取 CAN 控制器当前工作模式
 * @retval 0x00=正常模式, 0x80=初始化/配置模式, 0x01=睡眠模式
 */
uint8_t CH9434_GetMode(void)
{
    uint32_t statr = CH9434_AccessCANReg32(CAN_STATR, 0, 0);

    if (statr & CAN_STATR_SLAK)
        return 0x01;  // 睡眠模式
    if (statr & CAN_STATR_INAK)
        return 0x80;  // 初始化/配置模式
    return 0x00;      // 正常模式
}

/**
 * @brief  读取 CAN 内部寄存器 (返回低 8 位)
 * @param  can_reg_addr: CAN 内部寄存器地址
 * @retval 寄存器值的低 8 位
 */
uint8_t CH9434_ReadReg(uint8_t can_reg_addr)
{
    return (uint8_t)(CH9434_AccessCANReg32(can_reg_addr, 0, 0) & 0xFF);
}

/**
 * @brief  根据波特率计算 CAN_BTIMR 寄存器值
 *         CH9434D 内部 CAN 时钟 = 96MHz
 *         采用 8 个时间量子: 1(SYNC) + 5(TS1) + 2(TS2) = 8tq
 *         采样点 = (1+5)/8 = 75%
 * @param  baudrate: 目标波特率 (如 125000, 250000, 500000, 1000000)
 * @retval BTIMR 寄存器值
 */
static uint32_t CH9434_CalcBTIMR(uint32_t baudrate)
{
    uint32_t brp;
    uint32_t total_tq = 8;  // 1 + 5 + 2

    /* BRP = (96MHz / baudrate / total_tq) - 1 */
    brp = (96000000UL / baudrate / total_tq) - 1;
    if (brp > 1023) brp = 1023;

    /* BTIMR: [25:24]=SJW(0), [22:20]=TS2(1), [19:16]=TS1(4), [9:0]=BRP */
    return (0UL << 24) | (1UL << 20) | (4UL << 16) | brp;
}

/**
 * @brief  初始化 CH9434D 的 CAN 控制器
 * @param  baudrate: CAN 波特率 (如 125000 表示 125kbps)
 */
void CH9434_Init_CAN(uint32_t baudrate)
{
    uint16_t timeout;

    /* 1. 初始化底层 SPI */
    BSP_SPI_Init();
    for(volatile int i=0; i<10000; i++);

    /* 2. 启用 CAN 引脚复用功能 */
    CH9434_EnableDefaultPin(CH9434_PIN_CAN);

    /* 3. 先复位 CAN 控制器 (RST 单独写，复位后芯片进入睡眠模式) */
    CH9434_AccessCANReg32(CAN_CTLR, CAN_CTLR_RST, 1);
    for(volatile int i=0; i<1000; i++);  // 等待复位完成

    /* 4. 退出睡眠，请求进入初始化模式 (INRQ=1, SLEEP=0) */
    CH9434_AccessCANReg32(CAN_CTLR, CAN_CTLR_INRQ, 1);

    /* 等待进入初始化模式 (INAK=1) */
    timeout = 10000;
    while(timeout--) {
        if (CH9434_AccessCANReg32(CAN_STATR, 0, 0) & CAN_STATR_INAK)
            break;
    }

    /* 4. 配置波特率 */
    CH9434_AccessCANReg32(CAN_BTIMR, CH9434_CalcBTIMR(baudrate), 1);

    /* 5. 配置过滤器 —— 接收所有消息 */
    CH9434_AccessCANReg32(CAN_FCTLR, 0x01, 1);           // FINIT=1, 进入过滤器初始化
    CH9434_AccessCANReg32(CAN_FSCFGR, 0x01, 1);          // 过滤器0: 32位宽度
    CH9434_AccessCANReg32(CAN_FMCFGR, 0x00, 1);          // 过滤器0: 掩码模式
    CH9434_AccessCANReg32(CAN_FAFIFOR, 0x00, 1);         // 过滤器0: 关联 FIFO0
    CH9434_AccessCANReg32(CAN_F0R1, 0x00000000, 1);      // ID = 0
    CH9434_AccessCANReg32(CAN_F0R2, 0x00000000, 1);      // Mask = 0 (接收所有)
    CH9434_AccessCANReg32(CAN_FWR, 0x01, 1);             // 激活过滤器0
    CH9434_AccessCANReg32(CAN_FCTLR, 0x00, 1);           // FINIT=0, 退出过滤器初始化

    /* 6. 退出初始化模式，进入正常模式 */
    CH9434_AccessCANReg32(CAN_CTLR, 0x00000000, 1);

    /* 等待进入正常模式 (INAK=0) */
    timeout = 10000;
    while(timeout--) {
        if ((CH9434_AccessCANReg32(CAN_STATR, 0, 0) & CAN_STATR_INAK) == 0)
            break;
    }
}

/**
 * @brief  通过 CH9434D CAN 控制器发送一帧 CAN 消息
 * @param  msg: 指向 CAN_Message 结构体
 * @retval 1=发送成功, 0=超时, 2=邮箱忙, 3=发送错误(无ACK等)
 */
uint8_t CH9434_SendCANMessage(CAN_Message *msg)
{
    uint32_t tstatr;
    uint32_t txmir, txmdtr, txmdlr, txmdhr;
    uint16_t timeout;

    /* 检查发送邮箱 0 是否为空 */
    tstatr = CH9434_AccessCANReg32(CAN_TSTATR, 0, 0);
    if (!(tstatr & CAN_TSTATR_TME0)) {
        return 2;  // 邮箱忙
    }

    /* 构造 TXMIR (邮箱标识符寄存器) */
    if (msg->extended) {
        /* 扩展帧: ID[28:0] 在 [31:3], IDE=1(bit2) */
        txmir = (msg->id << 3) | (1UL << 2);
    } else {
        /* 标准帧: ID[10:0] 在 [31:21], IDE=0 */
        txmir = (msg->id << 21);
    }
    if (msg->rtr) {
        txmir |= (1UL << 1);  // RTR 位
    }

    /* 构造 TXMDTR (数据长度) */
    txmdtr = msg->dlc & 0x0F;

    /* 构造数据寄存器 (小端: 低字节在前) */
    txmdlr = 0;
    txmdhr = 0;
    if (msg->dlc > 0) txmdlr |= (uint32_t)msg->data[0];
    if (msg->dlc > 1) txmdlr |= (uint32_t)msg->data[1] << 8;
    if (msg->dlc > 2) txmdlr |= (uint32_t)msg->data[2] << 16;
    if (msg->dlc > 3) txmdlr |= (uint32_t)msg->data[3] << 24;
    if (msg->dlc > 4) txmdhr |= (uint32_t)msg->data[4];
    if (msg->dlc > 5) txmdhr |= (uint32_t)msg->data[5] << 8;
    if (msg->dlc > 6) txmdhr |= (uint32_t)msg->data[6] << 16;
    if (msg->dlc > 7) txmdhr |= (uint32_t)msg->data[7] << 24;

    /* 先写数据，再写标识符触发发送 */
    CH9434_AccessCANReg32(CAN_TXMDLR0, txmdlr, 1);
    CH9434_AccessCANReg32(CAN_TXMDHR0, txmdhr, 1);
    CH9434_AccessCANReg32(CAN_TXMDTR0, txmdtr, 1);

    /* 写 TXMIR 并置 TXRQ(bit0)=1 请求发送 */
    txmir |= 0x01;
    CH9434_AccessCANReg32(CAN_TXMIR0, txmir, 1);

    /* 等待发送完成 */
    timeout = 5000;
    while(timeout--) {
        tstatr = CH9434_AccessCANReg32(CAN_TSTATR, 0, 0);
        if (tstatr & CAN_TSTATR_RQCP0) {
            /* 清除 RQCP0 */
            CH9434_AccessCANReg32(CAN_TSTATR, CAN_TSTATR_RQCP0, 1);
            if (tstatr & CAN_TSTATR_TXOK0) {
                return 1;  // 发送成功
            } else {
                return 3;  // 发送错误 (无 ACK 等)
            }
        }
    }

    return 0;  // 超时
}

/**
 * @brief  接收 CH9434D CAN 控制器的一帧消息
 * @param  msg: 用于存储接收到的消息的结构体指针
 * @retval 1=接收成功, 0=FIFO为空(无消息)
 */
uint8_t CH9434_ReceiveCANMessage(CAN_Message *msg)
{
    uint32_t rfifo0;
    uint32_t rxmir, rxmdtr, rxmdlr, rxmdhr;

    /* 1. 检查 FIFO0 中是否有挂号的报文 (FMP0 低8位) */
    rfifo0 = CH9434_AccessCANReg32(CAN_RFIFO0, 0, 0);
    if ((rfifo0 & 0xFF) == 0) {
        return 0;  // FIFO 为空
    }

    /* 2. 读取接收邮箱数据 */
    rxmir  = CH9434_AccessCANReg32(CAN_RXMIR0,  0, 0);
    rxmdtr = CH9434_AccessCANReg32(CAN_RXMDTR0, 0, 0);
    rxmdlr = CH9434_AccessCANReg32(CAN_RXMDLR0, 0, 0);
    rxmdhr = CH9434_AccessCANReg32(CAN_RXMDHR0, 0, 0);

    /* 3. 解析 ID 和帧类型 */
    msg->extended = (rxmir & (1UL << 2)) ? 1 : 0;  // IDE 位
    msg->rtr      = (rxmir & (1UL << 1)) ? 1 : 0;  // RTR 位

    if (msg->extended) {
        msg->id = rxmir >> 3;   // 扩展帧: 29位ID
    } else {
        msg->id = rxmir >> 21;  // 标准帧: 11位ID
    }

    /* 4. 解析数据长度 */
    msg->dlc = rxmdtr & 0x0F;

    /* 5. 解析数据 (小端) */
    if (msg->dlc > 0) msg->data[0] = (uint8_t)(rxmdlr & 0xFF);
    if (msg->dlc > 1) msg->data[1] = (uint8_t)((rxmdlr >> 8)  & 0xFF);
    if (msg->dlc > 2) msg->data[2] = (uint8_t)((rxmdlr >> 16) & 0xFF);
    if (msg->dlc > 3) msg->data[3] = (uint8_t)((rxmdlr >> 24) & 0xFF);
    if (msg->dlc > 4) msg->data[4] = (uint8_t)(rxmdhr & 0xFF);
    if (msg->dlc > 5) msg->data[5] = (uint8_t)((rxmdhr >> 8)  & 0xFF);
    if (msg->dlc > 6) msg->data[6] = (uint8_t)((rxmdhr >> 16) & 0xFF);
    if (msg->dlc > 7) msg->data[7] = (uint8_t)((rxmdhr >> 24) & 0xFF);

    /* 6. 释放 FIFO0 当前邮箱 (置位 RFOM0 bit18) */
    CH9434_AccessCANReg32(CAN_RFIFO0, (1UL << 18), 1);

    return 1;  // 接收成功
}

