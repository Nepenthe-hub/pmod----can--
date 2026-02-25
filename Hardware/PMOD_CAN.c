#include "PMOD_CAN.h"
#include "CH9434.h"

/**
  * @brief  初始化PMOD CAN发送模块
  */
void PMOD_CAN_Init(void)
{
    /* 初始化CH9434为CAN模式，使用125kbps波特率与STM32 CAN匹配 */
    CH9434_Init_CAN(125000);
}

/**
  * @brief  通过PMOD发送CAN消息
  * @param  TxMessage: 指向要发送的CAN消息（STM32格式）
  * @retval 发送状态: 1成功, 0超时, 2缓冲区忙, 3发送错误
  */
uint8_t PMOD_CAN_Transmit(CanTxMsg *TxMessage)
{
    CAN_Message msg;
    uint8_t i;
    
    /* 转换消息格式：STM32 CanTxMsg -> CH9434 CAN_Message */
    if(TxMessage->IDE == CAN_Id_Extended) {
        msg.id = TxMessage->ExtId;
        msg.extended = 1;
    } else {
        msg.id = TxMessage->StdId;
        msg.extended = 0;
    }
    
    /* 处理远程帧/数据帧 */
    if(TxMessage->RTR == CAN_RTR_Remote) {
        msg.rtr = 1;
        msg.dlc = TxMessage->DLC;
    } else {
        msg.rtr = 0;
        msg.dlc = TxMessage->DLC;
        for(i = 0; i < msg.dlc && i < 8; i++) {
            msg.data[i] = TxMessage->Data[i];
        }
    }
    
    /* 发送消息并返回状态 */
    return CH9434_SendCANMessage(&msg);
}

/**
  * @brief  获取CH9434D当前工作模式
  * @retval 0x00=正常模式, 0x80=配置模式, 0x01=睡眠模式
  */
uint8_t PMOD_CAN_GetMode(void)
{
    return CH9434_GetMode();
}

/**
  * @brief  通过 PMOD 读取 CAN 接收消息
  * @param  RxMessage: 指向要存储接收到的CAN消息的结构体（STM32格式）
  * @retval 1: 成功接收到一帧消息, 0: 暂无新消息
  */
uint8_t PMOD_CAN_Receive(CanRxMsg *RxMessage)
{
    CAN_Message msg;
    uint8_t i;
    
    /* 1. 尝试从 CH9434D 底层读取消息 */
    if (CH9434_ReceiveCANMessage(&msg) == 0) {
        return 0; // FIFO 为空，没有收到消息
    }
    
    /* 2. 转换消息格式：CH9434 CAN_Message -> STM32 CanRxMsg */
    if (msg.extended) {
        RxMessage->IDE = CAN_Id_Extended;
        RxMessage->ExtId = msg.id;
        RxMessage->StdId = 0; // 清零以防干扰
    } else {
        RxMessage->IDE = CAN_Id_Standard;
        RxMessage->StdId = msg.id;
        RxMessage->ExtId = 0;
    }
    
    /* 3. 处理远程帧/数据帧 */
    if (msg.rtr) {
        RxMessage->RTR = CAN_RTR_Remote;
    } else {
        RxMessage->RTR = CAN_RTR_Data;
    }
    
    /* 4. 拷贝数据长度和内容 */
    RxMessage->DLC = msg.dlc;
    for(i = 0; i < msg.dlc && i < 8; i++) {
        RxMessage->Data[i] = msg.data[i];
    }
    
    /* STM32 StdPeriph 结构体里还有个 FMI (过滤器匹配序号)，CH9434只有过滤器0，暂填0 */
    RxMessage->FMI = 0;
    
    return 1; // 接收成功
}
