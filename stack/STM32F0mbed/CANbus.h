#ifndef CO_CANBUS_H
#define CO_CANBUS_H

#include "CAN.h"

class CANbus : public mbed::CAN {

public:
    CANbus(PinName rd, PinName td);

    void clearSendingMessages();
    bool rxOverrunFlagSet();

};

#endif //CO_CANBUS_H