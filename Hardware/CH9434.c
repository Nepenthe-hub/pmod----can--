#include "CH9434.h"
#include "SPI.h"

static uint8_t g_ch9434_can_init_ok = 0;

static uint32_t CH9434_ReadCANReg(uint8_t reg)
{
    return CH9434_AccessCANReg32(reg, 0, 0);
}

static void CH9434_WriteCANReg(uint8_t reg, uint32_t value)
{
    CH9434_AccessCANReg32(reg, value, 1);
}

static uint8_t CH9434_WaitRegBits(uint8_t reg, uint32_t mask, uint32_t expected, uint32_t timeout)
{
    while (timeout--) {
        if ((CH9434_ReadCANReg(reg) & mask) == expected) {
            return 1;
        }
    }
    return 0;
}

static uint32_t CH9434_CalcBTIMRFromBaud(uint32_t baudrate)
{
    uint32_t tq_total = 8;
    uint32_t prescaler;

    if (baudrate == 0) {
        baudrate = 125000;
    }

    prescaler = 96000000UL / (baudrate * tq_total);
    if (prescaler == 0) {
        prescaler = 1;
    }
    if (prescaler > 1024) {
        prescaler = 1024;
    }

    return ((uint32_t)CH9434_CAN_MODE_NORMAL << 30) |
           (0UL << 24) |
           (1UL << 20) |
           (4UL << 16) |
           (prescaler - 1);
}

/**
 * @brief  CH9434D 通过 SPI 写 8 位寄存器
 */
void CH9434_Write8(uint8_t addr, uint8_t val)
{
    SPI_CS_LOW();
    BSP_SPI_SwapByte(addr | CH9434_SPI_WRITE_BIT);
    for (volatile int d = 0; d < 30; d++);
    BSP_SPI_SwapByte(val);
    SPI_CS_HIGH();
    for (volatile int d = 0; d < 30; d++);
}

/**
 * @brief  CH9434D 读/写 32位 CAN 寄存器
 */
uint32_t CH9434_AccessCANReg32(uint8_t can_reg_addr, uint32_t write_val, uint8_t is_write)
{
    uint32_t read_val = 0;

    SPI_CS_LOW();
    if (is_write) {
        BSP_SPI_SwapByte(CH9434_REG_CAN | CH9434_SPI_WRITE_BIT);
    } else {
        BSP_SPI_SwapByte(CH9434_REG_CAN);
    }

    for (volatile int d = 0; d < 30; d++);

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
    for (volatile int d = 0; d < 20; d++);

    return read_val;
}

/**
 * @brief  启用 CH9434D 默认功能引脚
 */
void CH9434_EnableDefaultPin(uint8_t pin_addr)
{
    uint8_t status = 0;
    uint16_t timeout;

    SPI_CS_LOW();
    BSP_SPI_SwapByte(CH9434_REG_IO_SEL_FUN | CH9434_SPI_WRITE_BIT);
    for (volatile int d = 0; d < 10; d++);
    BSP_SPI_SwapByte(CH9434_IO_DEF_W_EN);
    BSP_SPI_SwapByte(pin_addr);
    BSP_SPI_SwapByte(0x01);
    BSP_SPI_SwapByte(CH9434_IO_CMD_ACT);
    SPI_CS_HIGH();

    for (volatile int d = 0; d < 20; d++);

    timeout = 2000;
    while (timeout--) {
        SPI_CS_LOW();
        BSP_SPI_SwapByte(CH9434_REG_IO_SEL_FUN);
        for (volatile int d = 0; d < 10; d++);
        BSP_SPI_SwapByte(0xFF);
        BSP_SPI_SwapByte(0xFF);
        status = BSP_SPI_SwapByte(0xFF);
        SPI_CS_HIGH();

        if (status == CH9434_IO_CMD_COMP) {
            break;
        }
        for (volatile int d = 0; d < 20; d++);
    }
}

void CH9434_CAN_StructInit(CH9434_CAN_InitTypeDef *init)
{
    if (init == 0) {
        return;
    }

    init->ttcm = 0;
    init->abom = 1;
    init->awum = 0;
    init->nart = 0;
    init->rflm = 0;
    init->txfp = 0;
    init->mode = CH9434_CAN_MODE_NORMAL;
    init->sjw = 0;
    init->bs1 = 4;
    init->bs2 = 1;
    init->prescaler = 96;
}

uint8_t CH9434_CAN_Init(const CH9434_CAN_InitTypeDef *init)
{
    CH9434_CAN_InitTypeDef local;
    uint32_t ctlr;
    uint32_t btimr;

    if (init == 0) {
        CH9434_CAN_StructInit(&local);
        init = &local;
    }

    BSP_SPI_Init();
    for (volatile int i = 0; i < 10000; i++);

    CH9434_EnableDefaultPin(CH9434_PIN_CAN);

    CH9434_WriteCANReg(CAN_CTLR, CAN_CTLR_RST);
    CH9434_WaitRegBits(CAN_CTLR, CAN_CTLR_RST, 0, 100000);

    ctlr = CH9434_ReadCANReg(CAN_CTLR);
    ctlr &= ~CAN_CTLR_SLEEP;
    CH9434_WriteCANReg(CAN_CTLR, ctlr);

    ctlr = CH9434_ReadCANReg(CAN_CTLR);
    ctlr |= CAN_CTLR_INRQ;
    CH9434_WriteCANReg(CAN_CTLR, ctlr);

    if (!CH9434_WaitRegBits(CAN_STATR, CAN_STATR_INAK, CAN_STATR_INAK, 100000)) {
        g_ch9434_can_init_ok = 0;
        return 0;
    }

    ctlr = CH9434_ReadCANReg(CAN_CTLR);
    ctlr &= ~(CAN_CTLR_TTCM | CAN_CTLR_ABOM | CAN_CTLR_AWUM | CAN_CTLR_NART | CAN_CTLR_RFLM | CAN_CTLR_TXFP);

    if (init->ttcm) ctlr |= CAN_CTLR_TTCM;
    if (init->abom) ctlr |= CAN_CTLR_ABOM;
    if (init->awum) ctlr |= CAN_CTLR_AWUM;
    if (init->nart) ctlr |= CAN_CTLR_NART;
    if (init->rflm) ctlr |= CAN_CTLR_RFLM;
    if (init->txfp) ctlr |= CAN_CTLR_TXFP;

    CH9434_WriteCANReg(CAN_CTLR, ctlr);

    btimr = ((uint32_t)(init->mode & 0x03) << 30) |
            ((uint32_t)(init->sjw & 0x03) << 24) |
            ((uint32_t)(init->bs2 & 0x07) << 20) |
            ((uint32_t)(init->bs1 & 0x0F) << 16) |
            ((uint32_t)((init->prescaler > 0) ? (init->prescaler - 1) : 0) & 0x3FF);

    CH9434_WriteCANReg(CAN_BTIMR, btimr);

    ctlr = CH9434_ReadCANReg(CAN_CTLR);
    ctlr &= ~CAN_CTLR_INRQ;
    CH9434_WriteCANReg(CAN_CTLR, ctlr);

    if (!CH9434_WaitRegBits(CAN_STATR, CAN_STATR_INAK, 0, 100000)) {
        g_ch9434_can_init_ok = 0;
        return 0;
    }

    g_ch9434_can_init_ok = 1;
    return 1;
}

void CH9434_CAN_FilterAcceptAll(void)
{
    uint32_t reg;

    reg = CH9434_ReadCANReg(CAN_FCTLR);
    reg |= CAN_FCTLR_FINIT;
    CH9434_WriteCANReg(CAN_FCTLR, reg);

    reg = CH9434_ReadCANReg(CAN_FWR);
    reg &= ~(1UL << 0);
    CH9434_WriteCANReg(CAN_FWR, reg);

    reg = CH9434_ReadCANReg(CAN_FSCFGR);
    reg |= (1UL << 0);
    CH9434_WriteCANReg(CAN_FSCFGR, reg);

    reg = CH9434_ReadCANReg(CAN_FMCFGR);
    reg &= ~(1UL << 0);
    CH9434_WriteCANReg(CAN_FMCFGR, reg);

    reg = CH9434_ReadCANReg(CAN_FAFIFOR);
    reg &= ~(1UL << 0);
    CH9434_WriteCANReg(CAN_FAFIFOR, reg);

    CH9434_WriteCANReg(CAN_F0R1, 0x00000000);
    CH9434_WriteCANReg(CAN_F0R2, 0x00000000);

    reg = CH9434_ReadCANReg(CAN_FWR);
    reg |= (1UL << 0);
    CH9434_WriteCANReg(CAN_FWR, reg);

    reg = CH9434_ReadCANReg(CAN_FCTLR);
    reg &= ~CAN_FCTLR_FINIT;
    CH9434_WriteCANReg(CAN_FCTLR, reg);
}

uint8_t CH9434_CAN_Transmit(const CAN_Message *msg)
{
    uint32_t tsr;
    uint32_t mir;
    uint32_t mdtr;
    uint32_t mdlr;
    uint32_t mdhr;
    uint8_t mailbox;
    uint8_t dlc;
    uint16_t timeout;
    uint32_t rqcp_bit[3] = {CAN_TSTATR_RQCP0, CAN_TSTATR_RQCP1, CAN_TSTATR_RQCP2};
    uint32_t txok_bit[3] = {CAN_TSTATR_TXOK0, CAN_TSTATR_TXOK1, CAN_TSTATR_TXOK2};

    if ((msg == 0) || (!g_ch9434_can_init_ok)) {
        return 0;
    }

    tsr = CH9434_ReadCANReg(CAN_TSTATR);

    if (tsr & CAN_TSTATR_TME0) {
        mailbox = 0;
    } else if (tsr & CAN_TSTATR_TME1) {
        mailbox = 1;
    } else if (tsr & CAN_TSTATR_TME2) {
        mailbox = 2;
    } else {
        return 2;
    }

    dlc = msg->dlc;
    if (dlc > 8) {
        dlc = 8;
    }

    mir = 0;
    if (msg->extended) {
        mir |= ((msg->id & 0x1FFFFFFFUL) << 3);
        mir |= CAN_TXMIRx_IDE;
    } else {
        mir |= ((msg->id & 0x7FFUL) << 21);
    }
    if (msg->rtr) {
        mir |= CAN_TXMIRx_RTR;
    }

    mdtr = (uint32_t)(dlc & 0x0F);

    mdlr = ((uint32_t)msg->data[0]) |
           ((uint32_t)msg->data[1] << 8) |
           ((uint32_t)msg->data[2] << 16) |
           ((uint32_t)msg->data[3] << 24);

    mdhr = ((uint32_t)msg->data[4]) |
           ((uint32_t)msg->data[5] << 8) |
           ((uint32_t)msg->data[6] << 16) |
           ((uint32_t)msg->data[7] << 24);

    CH9434_WriteCANReg((uint8_t)(CAN_TXMIR0 + mailbox * 4), mir);
    CH9434_WriteCANReg((uint8_t)(CAN_TXMDTR0 + mailbox * 4), mdtr);
    CH9434_WriteCANReg((uint8_t)(CAN_TXMDLR0 + mailbox * 4), mdlr);
    CH9434_WriteCANReg((uint8_t)(CAN_TXMDHR0 + mailbox * 4), mdhr);

    CH9434_WriteCANReg((uint8_t)(CAN_TXMIR0 + mailbox * 4), mir | CAN_TXMIRx_TXRQ);

    timeout = 30000;
    while (timeout--) {
        tsr = CH9434_ReadCANReg(CAN_TSTATR);
        if (tsr & rqcp_bit[mailbox]) {
            CH9434_WriteCANReg(CAN_TSTATR, rqcp_bit[mailbox]);
            if (tsr & txok_bit[mailbox]) {
                return 1;
            }
            return 3;
        }
    }

    return 0;
}

uint8_t CH9434_CAN_TransmitStatus(uint8_t mailbox)
{
    uint32_t tsr = CH9434_ReadCANReg(CAN_TSTATR);

    switch (mailbox) {
    case 0:
        if ((tsr & (CAN_TSTATR_RQCP0 | CAN_TSTATR_TXOK0 | CAN_TSTATR_TME0)) == (CAN_TSTATR_RQCP0 | CAN_TSTATR_TXOK0 | CAN_TSTATR_TME0)) {
            return CH9434_CAN_TX_OK;
        }
        if (tsr & CAN_TSTATR_RQCP0) {
            return CH9434_CAN_TX_FAILED;
        }
        break;

    case 1:
        if ((tsr & (CAN_TSTATR_RQCP1 | CAN_TSTATR_TXOK1 | CAN_TSTATR_TME1)) == (CAN_TSTATR_RQCP1 | CAN_TSTATR_TXOK1 | CAN_TSTATR_TME1)) {
            return CH9434_CAN_TX_OK;
        }
        if (tsr & CAN_TSTATR_RQCP1) {
            return CH9434_CAN_TX_FAILED;
        }
        break;

    case 2:
        if ((tsr & (CAN_TSTATR_RQCP2 | CAN_TSTATR_TXOK2 | CAN_TSTATR_TME2)) == (CAN_TSTATR_RQCP2 | CAN_TSTATR_TXOK2 | CAN_TSTATR_TME2)) {
            return CH9434_CAN_TX_OK;
        }
        if (tsr & CAN_TSTATR_RQCP2) {
            return CH9434_CAN_TX_FAILED;
        }
        break;

    default:
        break;
    }

    return CH9434_CAN_TX_PENDING;
}

uint8_t CH9434_CAN_MessagePending(uint8_t fifo)
{
    if (fifo > 1) {
        fifo = 0;
    }
    return (uint8_t)(CH9434_ReadCANReg((uint8_t)(CAN_RFIFO0 + fifo)) & CAN_RFIFOx_FMPx);
}

void CH9434_CAN_FIFORelease(uint8_t fifo)
{
    uint32_t rfifo;

    if (fifo > 1) {
        fifo = 0;
    }

    rfifo = CH9434_ReadCANReg((uint8_t)(CAN_RFIFO0 + fifo));
    rfifo |= CAN_RFIFOx_RFOMx;
    CH9434_WriteCANReg((uint8_t)(CAN_RFIFO0 + fifo), rfifo);
}

uint8_t CH9434_CAN_Receive(CAN_Message *msg, uint8_t fifo)
{
    uint32_t rxmir;
    uint32_t rxmdtr;
    uint32_t rxmdlr;
    uint32_t rxmdhr;

    if ((msg == 0) || (!g_ch9434_can_init_ok)) {
        return 0;
    }

    if (fifo > 1) {
        fifo = 0;
    }

    if (CH9434_CAN_MessagePending(fifo) == 0) {
        return 0;
    }

    rxmir  = CH9434_ReadCANReg((uint8_t)(CAN_RXMIR0 + fifo * 4));
    rxmdtr = CH9434_ReadCANReg((uint8_t)(CAN_RXMDTR0 + fifo * 4));
    rxmdlr = CH9434_ReadCANReg((uint8_t)(CAN_RXMDLR0 + fifo * 4));
    rxmdhr = CH9434_ReadCANReg((uint8_t)(CAN_RXMDHR0 + fifo * 4));

    msg->extended = (rxmir & CAN_TXMIRx_IDE) ? 1 : 0;
    msg->rtr = (rxmir & CAN_TXMIRx_RTR) ? 1 : 0;

    if (msg->extended) {
        msg->id = (rxmir >> 3) & 0x1FFFFFFF;
    } else {
        msg->id = (rxmir >> 21) & 0x7FF;
    }

    msg->dlc = (uint8_t)(rxmdtr & 0x0F);
    if (msg->dlc > 8) {
        msg->dlc = 8;
    }

    msg->data[0] = (uint8_t)(rxmdlr & 0xFF);
    msg->data[1] = (uint8_t)((rxmdlr >> 8) & 0xFF);
    msg->data[2] = (uint8_t)((rxmdlr >> 16) & 0xFF);
    msg->data[3] = (uint8_t)((rxmdlr >> 24) & 0xFF);
    msg->data[4] = (uint8_t)(rxmdhr & 0xFF);
    msg->data[5] = (uint8_t)((rxmdhr >> 8) & 0xFF);
    msg->data[6] = (uint8_t)((rxmdhr >> 16) & 0xFF);
    msg->data[7] = (uint8_t)((rxmdhr >> 24) & 0xFF);

    CH9434_CAN_FIFORelease(fifo);
    return 1;
}

uint8_t CH9434_CAN_IsInitialized(void)
{
    return g_ch9434_can_init_ok;
}

uint32_t CH9434_CAN_GetErrorReg(void)
{
    return CH9434_ReadCANReg(CAN_ERRSR);
}

uint32_t CH9434_CAN_GetTxStatusReg(void)
{
    return CH9434_ReadCANReg(CAN_TSTATR);
}

/**
 * @brief  读取 CAN 控制器当前工作模式
 */
uint8_t CH9434_GetMode(void)
{
    uint32_t statr = CH9434_ReadCANReg(CAN_STATR);

    if (statr & CAN_STATR_SLAK) {
        return 0x01;
    }
    if (statr & CAN_STATR_INAK) {
        return 0x80;
    }
    return 0x00;
}

/**
 * @brief  读取 CAN 内部寄存器低 8 位
 */
uint8_t CH9434_ReadReg(uint8_t can_reg_addr)
{
    return (uint8_t)(CH9434_ReadCANReg(can_reg_addr) & 0xFF);
}

/**
 * @brief  兼容旧接口：按波特率初始化 CH9434 CAN
 */
void CH9434_Init_CAN(uint32_t baudrate)
{
    CH9434_CAN_InitTypeDef init;

    CH9434_CAN_StructInit(&init);

    if (baudrate != 0) {
        uint32_t btimr = CH9434_CalcBTIMRFromBaud(baudrate);
        init.mode = (uint8_t)((btimr >> 30) & 0x03);
        init.sjw = (uint8_t)((btimr >> 24) & 0x03);
        init.bs2 = (uint8_t)((btimr >> 20) & 0x07);
        init.bs1 = (uint8_t)((btimr >> 16) & 0x0F);
        init.prescaler = (uint16_t)((btimr & 0x3FF) + 1);
    }

    if (CH9434_CAN_Init(&init)) {
        CH9434_CAN_FilterAcceptAll();
    }
}

/**
 * @brief  兼容旧接口：发送一帧消息
 * @retval 1=成功, 0=超时/未初始化, 2=邮箱忙, 3=发送错误
 */
uint8_t CH9434_SendCANMessage(CAN_Message *msg)
{
    return CH9434_CAN_Transmit(msg);
}

/**
 * @brief  兼容旧接口：从 FIFO0 接收一帧消息
 */
uint8_t CH9434_ReceiveCANMessage(CAN_Message *msg)
{
    return CH9434_CAN_Receive(msg, 0);
}
