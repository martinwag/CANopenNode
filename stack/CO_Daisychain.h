/**
 * CANopen LSS Master/Slave protocol.
 *
 * @file        CO_Daisychain.h
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


#ifndef CO_LSSslave_H
#define CO_LSSslave_H

#include "CO_Daisychain.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup CO_Daisychain CANopen Daisychain extension
 * @ingroup CO_CANopen
 * @{
 *
 * CANopen Daisychain extension
 *
 * This extension gives CANopen the functionallity of detecting the
 * physical structure of a network by using an additional wire (sequential
 * daisy chain, https://en.wikipedia.org/wiki/Daisy_chain_(electrical_engineering)).
 *
 * The daisychain event contains the active node ID and the current shift
 * counter. It should be triggered by the Daisy Chain shift input.
 * This message uses the same COB ID on all nodes. The contained node ID
 * indicates the transmitter. It is in the user's responsibility to only
 * trigger the shift event on one node at a time.
 * The used COB ID (0x6DF) is not in the pre-defined connection set and is
 * also not reserved.
 *
 * The following functions are supported
 * - Daisychain event generation
 * - Daisychain event detection
 *
 * For this, the following CAN message is used:
 * COB ID | Byte0          | Byte1
 * 0x6DF  | Event Counter  | own Node DI
 */

#if CO_DAISY_PRODUCER == 1
/**
 * @defgroup CO_Daisychain_producer Daisychain Producer
 * @ingroup CO_Daisychain
 * @{
 */

/**
 * Daisychain Producer object.
 */
typedef struct{
    CO_CANmodule_t  *CANdevTx;         /**< From #CO_DaisyProducer_init() */
    CO_CANtx_t      *TXbuff;           /**< CAN transmit buffer */
}CO_DaisyProducer_t;

/**
 * Initialize Daisychain object.
 *
 * Function must be called in the communication reset section.
 *
 * @param DaisyProducer This object will be initialized.
 * @param CANdevTx CAN device for LSS master transmission.
 * @param CANdevTxIdx Index of transmit buffer in the above CAN device.
 * @param CANidDaisychain COB ID for transmission.
 * @return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_DaisyProducer_init(
        CO_DaisyProducer_t  *DaisyProducer,
        CO_CANmodule_t      *CANdevTx,
        uint16_t             CANdevTxIdx,
        uint32_t             CANidDaisychain);

/**
 * Produce Daisychain Event
 *
 * @param DaisyProducer This object.
 * @param shiftCount event counter, should be incremented at every function call
 * @param nodeID own node id
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_TX_OVERFLOW
 */
CO_ReturnError_t CO_DaisyProducer_sendEvent(
        CO_DaisyProducer_t  *DaisyProducer,
        uint8_t              shiftCount,
        uint8_t              nodeID);

/** @} */

#endif //CO_DAISY_PRODUCER

#if CO_DAISY_CONSUMER == 1
/**
 * @defgroup CO_Daisychain_consumer Daisychain Consumer
 * @ingroup CO_Daisychain
 * @{
 */

/**
 * Return values of Daisychain Consumer functions.
 */
typedef enum {
    CO_DaisyConsumer_WAIT                = 1,    /**< No response arrived from producer yet */
    CO_DaisyConsumer_OK                  = 0,    /**< Success, end of communication */
    CO_DaisyConsumer_TIMEOUT             = -1,   /**< No reply received */
} CO_DaisyConsumer_return_t;

/**
 * Daisychain Consumer object.
 */
typedef struct{
  uint16_t         timeout;          /**< Daisychain response timeout in ms */

  uint16_t         timeoutTimer;     /**< Timeout timer for daisychain communication */

  volatile void   *CANrxNew;         /**< Indication if new daisychain message is received from CAN bus. It needs to be cleared when received message is completely processed. */
  uint8_t          CANrxData[2];     /**< 2 data bytes of the received message */

  void           (*pFunctSignal)(void *object); /**< From CO_DaisyConsumer_initCallback() or NULL */
  void            *functSignalObject;/**< Pointer to object */
}CO_DaisyConsumer_t;

/**
 * Default timeout for daisychain consumer in ms.
 */
#define CO_DaisyConsumer_DEFAULT_TIMEOUT 100 /* ms */

/**
 * Initialize Daisychain object.
 *
 * Function must be called in the communication reset section.
 *
 * @param DaisyConsumer This object will be initialized.
 * @param timeout_ms consumer timeout in ms
 * @param CANdevRx CAN device for event reception.
 * @param CANdevRxIdx Index of receive buffer in the above CAN device.
 * @param CANidLssSlave COB ID for reception.
 * @return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_DaisyConsumer_init(
        CO_DaisyConsumer_t  *DaisyConsumer,
        uint16_t             timeout_ms,
        CO_CANmodule_t      *CANdevRx,
        uint16_t             CANdevRxIdx,
        uint32_t             CANidDaisychain);

/**
 * Initialize daisychain consumer callback function.
 *
 * Function initializes optional callback function, which is called after new
 * message is received from the CAN bus. Function may wake up external task,
 * which processes mainline CANopen functions.
 *
 * @param DaisyConsumer This object.
 * @param object Pointer to object, which will be passed to pFunctSignal(). Can be NULL
 * @param pFunctSignal Pointer to the callback function. Not called if NULL.
 */
void CO_DaisyConsumer_initCallback(
        CO_LSSmaster_t      *DaisyConsumer,
        void                *object,
        void               (*pFunctSignal)(void *object));

/**
 * Produce Daisychain Event
 *
 * @param DaisyConsumer This object.
 * @param timeDifference_ms Time difference from previous function call in
 * [milliseconds]. Zero when request is started.
 * @param [out] shiftCount event counter
 * @param [otu] nodeId node id of the event producer
 * @return #CO_DaisyConsumer_return_t
 */
CO_DaisyConsumer_return_t CO_DaisyConsumer_waitEvent(
        CO_DaisyConsumer_t  *DaisyConsumer,
        uint16_t             timeDifference_ms,
        uint8_t             *shiftCount,
        uint8_t             *nodeId);

/** @} */

#endif //CO_DAISY_CONSUMER

#ifdef __cplusplus
}
#endif /*__cplusplus*/

/** @} */
#endif
