#ifndef CO_CANBUS_H
#define CO_CANBUS_H

#include "CAN.h"

class CANbus : public mbed::CAN {

public:
    CANbus(PinName rd, PinName td);

    void clearSendingMessages();
    bool rxOverrunFlagSet();

    int read_Nonblocking(mbed::CANMessage &msg, int handle = 0);
    int write_Nonblocking(mbed::CANMessage &msg);

};

#endif //CO_CANBUS_H