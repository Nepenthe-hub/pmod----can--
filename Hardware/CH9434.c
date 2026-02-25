#include "CH9434.h"
#include "SPI.h"  // 引用底层SPI驱动

/**
  * @brief  向 CH9434 写入寄存器
  * @param  addr: 寄存器地址
  * @param  val:  写入的值
  */
void CH9434_WriteReg(uint8_t addr, uint8_t val)
{
    SPI_CS_LOW();
    BSP_SPI_SwapByte(CH9434_CMD_WRITE); // 命令
    BSP_SPI_SwapByte(addr);             // 地址
    BSP_SPI_SwapByte(val);              // 数据
    SPI_CS_HIGH();
}

/**
  * @brief  从 CH9434 读取寄存器
  * @param  addr: 寄存器地址
  * @retval 读取到的值
  */
uint8_t CH9434_ReadReg(uint8_t addr)
{
    uint8_t val;
    SPI_CS_LOW();
    BSP_SPI_SwapByte(CH9434_CMD_READ);  // 命令
    BSP_SPI_SwapByte(addr);             // 地址
    val = BSP_SPI_SwapByte(0xFF);       // 发送Dummy Byte以产生时钟读取数据
    SPI_CS_HIGH();
    return val;
}

/**
  * @brief  设置CH9434工作模式
  * @param  mode: 工作模式
  */
void CH9434_SetMode(uint8_t mode)
{
    CH9434_WriteReg(CH9434_REG_CANCTRL, mode);
    
    // 等待模式切换完成
    uint8_t timeout = 100;
    while(((CH9434_ReadReg(CH9434_REG_CANSTAT) & CANSTAT_OPMOD_MASK) != mode) && timeout--)
    {
        for(volatile int i=0; i<1000; i++); // 简单延时
    }
}

/**
  * @brief  初始化 CH9434 为 CAN 模式
  * @param  bitrate: CAN波特率 (例如 250000 为 250kbps)
  */
void CH9434_Init_CAN(uint32_t bitrate)
{
    /* 1. 初始化底层SPI硬件 */
    BSP_SPI_Init();
    
    /* 简单延时等待芯片上电稳定 */
    for(volatile int i=0; i<10000; i++);

    /* 2. 进入配置模式 */
    CH9434_SetMode(CANCTRL_REQOP_CONFIG);

    /* 3. 配置CAN位时序 */
    /* 假设8MHz晶振，配置常用波特率 */
    if(bitrate == 1000000) {        // 1Mbps
        CH9434_WriteReg(CH9434_REG_CNF1, 0x00);  // SJW=1, BRP=1
        CH9434_WriteReg(CH9434_REG_CNF2, 0x80);  // BTLMODE=1, PHSEG1=1
        CH9434_WriteReg(CH9434_REG_CNF3, 0x00);  // PHSEG2=1
    }
    else if(bitrate == 500000) {    // 500kbps
        CH9434_WriteReg(CH9434_REG_CNF1, 0x00);  // SJW=1, BRP=1
        CH9434_WriteReg(CH9434_REG_CNF2, 0x91);  // BTLMODE=1, PHSEG1=2
        CH9434_WriteReg(CH9434_REG_CNF3, 0x01);  // PHSEG2=2
    }
    else if(bitrate == 250000) {    // 250kbps
        CH9434_WriteReg(CH9434_REG_CNF1, 0x01);  // SJW=1, BRP=2
        CH9434_WriteReg(CH9434_REG_CNF2, 0x91);  // BTLMODE=1, PHSEG1=2
        CH9434_WriteReg(CH9434_REG_CNF3, 0x01);  // PHSEG2=2
    }
    else if(bitrate == 125000) {    // 125kbps (与STM32硬件CAN匹配)
        CH9434_WriteReg(CH9434_REG_CNF1, 0x03);  // SJW=1, BRP=4
        CH9434_WriteReg(CH9434_REG_CNF2, 0x91);  // BTLMODE=1, PHSEG1=2
        CH9434_WriteReg(CH9434_REG_CNF3, 0x01);  // PHSEG2=2
    }
    else {                          // 默认125kbps
        CH9434_WriteReg(CH9434_REG_CNF1, 0x03);  // SJW=1, BRP=4
        CH9434_WriteReg(CH9434_REG_CNF2, 0x91);  // BTLMODE=1, PHSEG1=2
        CH9434_WriteReg(CH9434_REG_CNF3, 0x01);  // PHSEG2=2
    }

    /* 4. 配置接收过滤器 (接收所有消息) */
    // 接收缓冲区0配置为接收所有标准帧
    CH9434_WriteReg(CH9434_REG_RXB0CTRL, 0x60);  // 接收所有有效消息

    /* 5. 清除中断标志 */
    CH9434_WriteReg(CH9434_REG_CANINTF, 0x00);

    /* 6. 使能接收中断 */
    CH9434_WriteReg(CH9434_REG_CANINTE, CANINTF_RX0IF);

    /* 7. 进入正常模式 */
    CH9434_SetMode(CANCTRL_REQOP_NORMAL);
}

/**
  * @brief  检查是否有CAN消息可读
  * @retval 1: 有消息, 0: 无消息
  */
uint8_t CH9434_Available(void)
{
    uint8_t intf = CH9434_ReadReg(CH9434_REG_CANINTF);
    return (intf & CANINTF_RX0IF) ? 1 : 0;
}

/**
  * @brief  发送CAN消息
  * @param  msg: 指向CAN消息结构的指针
  * @retval 1: 发送成功, 0: 发送失败/超时, 2: 缓冲区忙, 3: 发送错误
  */
uint8_t CH9434_SendCANMessage(CAN_Message *msg)
{
    uint8_t i;
    uint16_t timeout;
    
    /* 检查发送缓冲区是否空闲 */
    uint8_t ctrl = CH9434_ReadReg(CH9434_REG_TXB0CTRL);
    if(ctrl & 0x08) {  // TXREQ位为1表示正在发送
        return 2;  // 发送失败，缓冲区忙
    }
    
    /* 设置ID */
    if(msg->extended) {
        // 扩展帧 (29位ID)
        CH9434_WriteReg(CH9434_REG_TXB0SIDH, (uint8_t)(msg->id >> 21));
        CH9434_WriteReg(CH9434_REG_TXB0SIDL, (uint8_t)(msg->id >> 13) | 0x08);  // 设置EXIDE位
        CH9434_WriteReg(CH9434_REG_TXB0EID8, (uint8_t)(msg->id >> 8));
        CH9434_WriteReg(CH9434_REG_TXB0EID0, (uint8_t)msg->id);
    } else {
        // 标准帧 (11位ID)
        CH9434_WriteReg(CH9434_REG_TXB0SIDH, (uint8_t)(msg->id >> 3));
        CH9434_WriteReg(CH9434_REG_TXB0SIDL, (uint8_t)(msg->id << 5));
        CH9434_WriteReg(CH9434_REG_TXB0EID8, 0);
        CH9434_WriteReg(CH9434_REG_TXB0EID0, 0);
    }
    
    /* 设置数据长度 */
    CH9434_WriteReg(CH9434_REG_TXB0DLC, msg->dlc & 0x0F);
    
    /* 写入数据 */
    for(i = 0; i < msg->dlc && i < 8; i++) {
        CH9434_WriteReg(CH9434_REG_TXB0D0 + i, msg->data[i]);
    }
    
    /* 请求发送 */
    CH9434_WriteReg(CH9434_REG_TXB0CTRL, 0x08);  // 设置TXREQ位
    
    /* 等待发送完成 */
    timeout = 10000;
    while(timeout--) {
        ctrl = CH9434_ReadReg(CH9434_REG_TXB0CTRL);
        if(!(ctrl & 0x08)) {  // TXREQ被清除，发送完成
            /* 检查是否有发送错误 */
            if(ctrl & 0x10) {  // TXERR位
                return 3;  // 发送错误（可能是无ACK）
            }
            return 1;  // 发送成功
        }
    }
    
    return 0;  // 超时
}

/**
  * @brief  接收CAN消息
  * @param  msg: 指向CAN消息结构的指针，用于存储接收到的消息
  * @retval 1: 接收成功, 0: 无消息
  */
uint8_t CH9434_ReceiveCANMessage(CAN_Message *msg)
{
    uint8_t i;
    
    /* 检查是否有消息 */
    if(!CH9434_Available()) {
        return 0;
    }
    
    /* 读取ID */
    uint8_t sidh = CH9434_ReadReg(CH9434_REG_RXB0SIDH);
    uint8_t sidl = CH9434_ReadReg(CH9434_REG_RXB0SIDL);
    
    if(sidl & 0x08) {  // 扩展帧
        msg->extended = 1;
        uint8_t eid8 = CH9434_ReadReg(CH9434_REG_RXB0SIDH + 2);  // EID8
        uint8_t eid0 = CH9434_ReadReg(CH9434_REG_RXB0SIDH + 3);  // EID0
        msg->id = ((uint32_t)sidh << 21) | ((uint32_t)(sidl & 0xE0) << 13) | 
                  ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
    } else {  // 标准帧
        msg->extended = 0;
        msg->id = ((uint32_t)sidh << 3) | (sidl >> 5);
    }
    
    /* 读取数据长度 */
    msg->dlc = CH9434_ReadReg(CH9434_REG_RXB0DLC) & 0x0F;
    
    /* 读取数据 */
    for(i = 0; i < msg->dlc && i < 8; i++) {
        msg->data[i] = CH9434_ReadReg(CH9434_REG_RXB0D0 + i);
    }
    
    /* 清除接收中断标志 */
    CH9434_WriteReg(CH9434_REG_CANINTF, 0x00);
    
    return 1;  // 接收成功
}
