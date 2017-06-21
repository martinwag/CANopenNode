/**
 * CANopen LSS Master/Slave protocol.
 *
 * @file        CO_LSSslave.h
 * @ingroup     CO_LSS
 * @author      Martin Wagner
 * @copyright   2017 Neuberger Gebäudeautomation GmbH
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

#ifdef __cplusplus
extern "C" {
#endif

#include "CO_LSS.h"

/**
 * @addtogroup CO_LSS
 * @{
 */

/** @todo #CO_CANsend() is used inside Rx callback, user must ensure that this
 * function can be used inside interrupt if Rx is done inside interrupt.
 */

/**
 * LSS slave object.
 */
typedef struct{
    CO_LSS_address_t        lssAddress;       /**< From #CO_LSSslave_init */
    CO_LSS_state_t          lssState;         /**< #CO_LSS_state_t */
    CO_LSS_address_t        lssSelect;        /**< Received LSS Address */

    uint16_t                pendingBitRate;   /**< Bit rate value that is temporarily configured in volatile memory */
    uint8_t                 pendingNodeID;    /**< Node ID that is temporarily configured in volatile memory */
    uint8_t                 activeNodeID;     /**< Node ID used at the CAN interface */

    bool_t                (*pFunctLSScheckBitRate)(void *object, uint16_t bitRate); /**< From CO_LSSslave_initCheckBitRateCallback() or NULL */
    void                   *functLSScheckBitRateObject; /** Pointer to object */
    void                  (*pFunctLSSactivateBitRate)(void *object, uint16_t delay); /**< From CO_LSSslave_initActivateBitRateCallback() or NULL. Delay is in ms */
    void                   *functLSSactivateBitRateObject; /** Pointer to object */
    bool_t                (*pFunctLSScfgStore)(void *object, uint8_t id, uint16_t bitRate); /**< From CO_LSSslave_initCfgStoreCallback() or NULL */
    void                   *functLSScfgStore; /** Pointer to object */

    CO_CANmodule_t         *CANdevTx;         /**< From #CO_LSSslave_init() */
    CO_CANtx_t             *TXbuff;           /**< CAN transmit buffer */
}CO_LSSslave_t;

/**
 * Initialize LSS object.
 *
 * Function must be called in the communication reset section.
 *
 * Depending on the startup type, pending bit rate and node ID have to be
 * supplied differently. After #CO_NMT_RESET_NODE or at power up they should
 * be restored from persitent bit rate and node id. After #CO_NMT_RESET_COMMUNICATION
 * they have to be supplied from the application and are generally the values
 * that have been last returned by #CO_LSSslave_process() before resetting.
 *
 * @param LSSslave This object will be initialized.
 * @param lssAddress LSS address are values from object 0x1018 - Identity
 * @param pendingBitRate Bit rate the CAN interface is supposed to have after init
 * @param pendingNodeID stored node id from nvm or 0xFF - invalid
 * @param CANdevRx CAN device for LSS slave reception.
 * @param CANdevRxIdx Index of receive buffer in the above CAN device.
 * @param CANidLssMaster COB ID for reception
 * @param CANdevTx CAN device for LSS slave transmission.
 * @param CANdevTxIdx Index of transmit buffer in the above CAN device.
 * @param CANidLssSlave COB ID for transmission
 * @return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_LSSslave_init(
        CO_LSSslave_t          *LSSslave,
        CO_LSS_address_t        lssAddress,
        uint16_t                pendingBitRate,
        uint8_t                 pendingNodeID,
        CO_CANmodule_t         *CANdevRx,
        uint16_t                CANdevRxIdx,
        uint32_t                CANidLssMaster,
        CO_CANmodule_t         *CANdevTx,
        uint16_t                CANdevTxIdx,
        uint32_t                CANidLssSlave);

/**
 * Process LSS communication
 *
 * - sets currently active node ID and bit rate so master can read it
 * - get current pending node ID and bit rate
 *
 * @param LSSslave This object.
 * @param activeBitRate Currently active bit rate
 * @param activeNodeId Currently active node ID
 * @param pendingBitRate [out] Requested bit rate
 * @param pendingNodeId [out] Requested node id
 */
void CO_LSSslave_process(
        CO_LSSslave_t          *LSSslave,
        uint16_t                activeBitRate,
        uint8_t                 activeNodeId,
        uint16_t               *pendingBitRate,
        uint8_t                *pendingNodeId);

/**
 * Initialize verify bit rate callback
 *
 * Function initializes callback function, which is called when "config bit
 * timing parameters" is used. The callback function needs to check if the new bit
 * rate is supported by the CANopen device. Callback returns "true" if supported.
 * When no callback is set, LSS server will no-ack the request.
 *
 * @remark Depending on the CAN driver implementation, this function is called
 * inside an ISR
 *
 * @param LSSslave This object.
 * @param object Pointer to object, which will be passed to #pFunctLSScheckBitRate. Can be NULL
 * @param pFunctLSScheckBitRate Pointer to the callback function. Not called if NULL.
 */
void CO_LSSslave_initCheckBitRateCallback(
        CO_LSSslave_t          *LSSslave,
        void                   *object,
        bool_t                (*pFunctLSScheckBitRate)(void *object, uint16_t bitRate));

/**
 * Initialize activate bit rate callback
 *
 * Function initializes callback function, which is called when "activate bit
 * timing parameters" is used. The callback function gives the user an event to
 * allow setting a timer or do calculations based on the exact time the request
 * arrived.
 * According to DSP 305 6.4.4, the delay has to be applied once before and once after
 * switching bit rates. During this time, a device musn't send any messages.
 *
 * @remark Depending on the CAN driver implementation, this function is called
 * inside an ISR
 *
 * @param LSSslave This object.
 * @param object Pointer to object, which will be passed to #pFunctLSSchangeBitRate. Can be NULL
 * @param pFunctLSSchangeBitRate Pointer to the callback function. Not called if NULL.
 */
void CO_LSSslave_initActivateBitRateCallback(
        CO_LSSslave_t          *LSSslave,
        void                   *object,
        void                  (*pFunctLSSactivateBitRate)(void *object, uint16_t delay));

/**
 * Store configuration callback
 *
 * Function initializes callback function, which is called when "store configuration" is used.
 * The callback function gives the user an event to store the corresponding node id and bit rate
 * to NVM. Those values have to be supplied to the init function as "persistent values"
 * after reset. If callback returns "true", success is send to the LSS master. When no
 * callback is set, LSS server will no-ack the request.
 *
 * @remark Depending on the CAN driver implementation, this function is called
 * inside an ISR
 *
 * @param LSSslave This object.
 * @param object Pointer to object, which will be passed to #pFunctLSSchangeBitRate. Can be NULL
 * @param pFunctLSScfgStore Pointer to the callback function. Not called if NULL.
 */
void CO_LSSslave_initCfgStoreCallback(
        CO_LSSslave_t          *LSSslave,
        void                   *object,
        bool_t                (*pFunctLSScfgStore)(void *object, uint8_t id, uint16_t bitRate));




#ifdef __cplusplus
}
#endif /*__cplusplus*/

/** @} */
#endif
