/*
 * CANopen LSS Slave protocol.
 *
 * @file        CO_Daisychain.c
 * @ingroup     CO_Daisychain
 * @author      Martin Wagner
 * @copyright   2018 Neuberger Gebäudeautomation GmbH
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free and open source software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Following clarification and special exception to the GNU General Public
 * License is included to the distribution terms of CANopenNode:
 *
 * Linking this library statically or dynamically with other modules is
 * making a combined work based on this library. Thus, the terms and
 * conditions of the GNU General Public License cover the whole combination.
 *
 * As a special exception, the copyright holders of this library give
 * you permission to link this library with independent modules to
 * produce an executable, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting
 * executable under terms of your choice, provided that you also meet,
 * for each linked independent module, the terms and conditions of the
 * license of that module. An independent module is a module which is
 * not derived from or based on this library. If you modify this
 * library, you may extend this exception to your version of the
 * library, but you are not obliged to do so. If you do not wish
 * to do so, delete this exception statement from your version.
 */

#include "CANopen.h"
#include "CO_Daisychain.h"

#if CO_DAISY_PRODUCER == 1

CO_ReturnError_t CO_DaisyProducer_init(
        CO_DaisyProducer_t  *DaisyProducer,
        CO_CANmodule_t      *CANdevTx,
        uint16_t             CANdevTxIdx,
        uint32_t             CANidDaisychain)
{
    /* verify arguments */
    if (DaisyProducer==NULL || CANdevTx==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* configure daisychain producer message transmission */
    DaisyProducer->CANdevTx = CANdevTx;
    DaisyProducer->TXbuff = CO_CANtxBufferInit(
            CANdevTx,             /* CAN device */
            CANdevTxIdx,          /* index of specific buffer inside CAN module */
            CANidDaisychain,      /* CAN identifier */
            0,                    /* rtr */
            2,                    /* number of data bytes */
            0);                   /* synchronous message flag bit */

    return CO_ERROR_NO;
}

CO_ReturnError_t CO_DaisyProducer_sendEvent(
        CO_DaisyProducer_t  *DaisyProducer,
        uint8_t              shiftCount,
        uint8_t              nodeID)
{
    if (DaisyProducer==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    DaisyProducer->TXbuff->data[0] = shiftCount;
    DaisyProducer->TXbuff->data[1] = nodeID;

    return CO_CANsend(DaisyProducer->CANdevTx, DaisyProducer->TXbuff);
}

#endif

#if CO_DAISY_CONSUMER == 1

/*
 * Read received message from CAN module.
 *
 * Function will be called (by CAN receive interrupt) every time, when CAN
 * message with correct identifier will be received. For more information and
 * description of parameters see file CO_driver.h.
 */
static void CO_DaisyConsumer_receive(void *object, const CO_CANrxMsg_t *msg)
{
    CO_DaisyConsumer_t *DaisyConsumer;

    DaisyConsumer = (CO_DaisyConsumer_t*)object; /* this is the correct pointer type of the first argument */

    /* verify message length and message overflow (previous message was not processed yet) */
    if(msg->DLC==2 && !IS_CANrxNew(DaisyConsumer->CANrxNew)) {
        /* copy data and set 'new message' flag */
        DaisyConsumer->CANrxData[0] = msg->data[0];
        DaisyConsumer->CANrxData[1] = msg->data[1];

        SET_CANrxNew(DaisyConsumer->CANrxNew);

        /* Optional signal to RTOS, which can resume task, which handles SDO client. */
        if(DaisyConsumer->pFunctSignal != NULL) {
            DaisyConsumer->pFunctSignal(DaisyConsumer->functSignalObject);
        }
    }
}

/*
 * Check Daisychain consumer timeout.
 *
 * Generally, we do not really care if the message has been received before
 * or after the timeout expired. Only if no message has been received we have
 * to check for timeouts
 */
static CO_DaisyConsumer_return_t CO_DaisyConsumer_check_timeout(
        CO_DaisyConsumer_t     *DaisyConsumer,
        uint16_t                timeDifference_ms)
{
  CO_DaisyConsumer_return_t ret = CO_DaisyConsumer_WAIT;

    DaisyConsumer->timeoutTimer += timeDifference_ms;
    if (DaisyConsumer->timeoutTimer >= DaisyConsumer->timeout) {
        DaisyConsumer->timeoutTimer = 0;
        ret = CO_DaisyConsumer_TIMEOUT;
    }

    return ret;
}

CO_ReturnError_t CO_DaisyConsumer_init(
        CO_DaisyConsumer_t  *DaisyConsumer,
        uint16_t             timeout_ms,
        CO_CANmodule_t      *CANdevRx,
        uint16_t             CANdevRxIdx,
        uint32_t             CANidDaisychain)
{
    /* verify arguments */
    if (DaisyConsumer==NULL || CANdevRx==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    DaisyConsumer->timeout = timeout_ms;
    DaisyConsumer->timeoutTimer = 0;
    CLEAR_CANrxNew(DaisyConsumer->CANrxNew);
    CO_memset(DaisyConsumer->CANrxData, 0, sizeof(DaisyConsumer->CANrxData));
    DaisyConsumer->pFunctSignal = NULL;
    DaisyConsumer->functSignalObject = NULL;

    /* configure daisychain consumer message reception */
    CO_CANrxBufferInit(
            CANdevRx,             /* CAN device */
            CANdevRxIdx,          /* rx buffer index */
            CANidDaisychain,      /* CAN identifier */
            0x7FF,                /* mask */
            0,                    /* rtr */
            (void*)DaisyConsumer, /* object passed to receive function */
            CO_DaisyConsumer_receive); /* this function will process received message */

    return CO_ERROR_NO;
}

void CO_DaisyConsumer_initCallback(
        CO_LSSmaster_t      *DaisyConsumer,
        void                *object,
        void               (*pFunctSignal)(void *object))
{
    if(DaisyConsumer != NULL){
        DaisyConsumer->functSignalObject = object;
        DaisyConsumer->pFunctSignal = pFunctSignal;
    }
}

CO_DaisyConsumer_return_t CO_DaisyConsumer_waitEvent(
        CO_DaisyConsumer_t  *DaisyConsumer,
        uint16_t             timeDifference_ms,
        uint8_t             *shiftCount,
        uint8_t             *nodeId)
{
    CO_DaisyConsumer_return_t ret;

    if (DaisyConsumer==NULL){
        return CO_DaisyConsumer_TIMEOUT;
    }

    if (IS_CANrxNew(DaisyConsumer->CANrxNew)) {
        if (shiftCount != NULL) {
            *shiftCount = DaisyConsumer->CANrxData[0];
        }
        if (nodeId != NULL) {
            *nodeId = DaisyConsumer->CANrxData[1];
        }
        CLEAR_CANrxNew(DaisyConsumer->CANrxNew);

        ret = CO_DaisyConsumer_OK;
    }
    else {
        ret = CO_DaisyConsumer_check_timeout(DaisyConsumer, timeDifference_ms);
    }

    return ret;
}

#endif
