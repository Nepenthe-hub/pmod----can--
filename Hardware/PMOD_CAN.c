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
