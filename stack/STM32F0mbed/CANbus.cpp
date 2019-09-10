#include "CANbus.h"
#include "stm32f0xx_hal_can.h"

CANbus::CANbus(PinName rd, PinName td) :
    mbed::CAN(rd, td)
{
}


void CANbus::clearSendingMessages()
{
    lock();
    if(!(__HAL_CAN_TRANSMIT_STATUS(&_can.CanHandle, CAN_TXMAILBOX_0)))
        __HAL_CAN_CANCEL_TRANSMIT(&_can.CanHandle, CAN_TXMAILBOX_0);
    if(!(__HAL_CAN_TRANSMIT_STATUS(&_can.CanHandle, CAN_TXMAILBOX_1)))
        __HAL_CAN_CANCEL_TRANSMIT(&_can.CanHandle, CAN_TXMAILBOX_1);
    if(!(__HAL_CAN_TRANSMIT_STATUS(&_can.CanHandle, CAN_TXMAILBOX_2)))
        __HAL_CAN_CANCEL_TRANSMIT(&_can.CanHandle, CAN_TXMAILBOX_2);
    unlock();
}

bool CANbus::rxOverrunFlagSet()
{
    bool fifo0 = (bool) __HAL_CAN_GET_FLAG(&_can.CanHandle, CAN_FLAG_FOV0);
    bool fifo1 = (bool) __HAL_CAN_GET_FLAG(&_can.CanHandle, CAN_FLAG_FOV1);
    return (fifo0 || fifo1 ? true : false);
}