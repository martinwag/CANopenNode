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

/**
 * LSS slave object.
 */
typedef struct{
    CO_LSS_address_t        lssAddress;       /**< From #CO_LSSslave_init */
    CO_LSS_state_t          lssState;         /**< #CO_LSS_state_t */
    CO_LSS_address_t        lssSelect;        /**< Received LSS Address */

    CO_LSS_bitTimingTable_t activeBitRate;    /**< Bit rate, used at the CAN interface */
    CO_LSS_bitTimingTable_t pendingBitRate;   /**< Bit rate value that is temporarily configured in volatile memory */
    CO_LSS_bitTimingTable_t persistentBitRate;/**< Bit rate value that is stored to LSS slaves NVM */
    uint8_t                 activeNodeID;     /**< Node ID used at the CAN interface */
    uint8_t                 pendingNodeID;    /**< Node ID that is temporarily configured in volatile memory */
    uint8_t                 persistentNodeID; /**< Node ID that is stored to LSS slaves NVM */

    CO_CANmodule_t         *CANdevTx;         /**< From #CO_LSSslave_init() */
    CO_CANtx_t             *TXbuff;           /**< CAN transmit buffer */
}CO_LSSslave_t;

/**
 * Initialize LSS object.
 *
 * Function must be called in the communication reset section. todo?
 *
 * @param LSSslave
 * @param lssAddress
 * @param activeBitRate
 * @param persistentBitRate
 * @param activeNodeId
 * @param persistentNodeID
 * @param CANdevRx
 * @param CANdevRxIdx
 * @param CANidLssMaster
 * @param CANdevTx
 * @param CANdevTxIdx
 * @param CANidLssSlave
 * @return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT. todo
 */
CO_ReturnError_t CO_LSSslave_init(
        CO_LSSslave_t          *LSSslave,
        CO_LSS_address_t        lssAddress,
        uint8_t                 activeBitRate,
        uint8_t                 persistentBitRate,
        uint8_t                 activeNodeId,
        uint8_t                 persistentNodeID,
        CO_CANmodule_t         *CANdevRx,
        uint16_t                CANdevRxIdx,
        uint32_t                CANidLssMaster,
        CO_CANmodule_t         *CANdevTx,
        uint16_t                CANdevTxIdx,
        uint32_t                CANidLssSlave);

/**
 * Process LSS communication
 *
 * @return #CO_NMT_reset_cmd_t
 */
CO_NMT_reset_cmd_t CO_LSSslave_process(
        CO_LSSslave_t          *LSSslave,
        const uint8_t          *activeBitRate,
        const uint8_t          *activeNodeId);

/**
 * Initialize LSS persistent value changed callback
 *
 * Function initializes callback function, which is called after LSS persistent
 * values (address or bit rate) are changed. If LSS persistance is required, the
 * application has to provide non-volatile storage for those values.
 *
 * @param LSSslave This object.
 * @param pFunctLSS Pointer to the callback function. Not called if NULL.
 */
void CO_LSSslave_initPersistanceCallback(
        CO_LSSslave_t          *LSSslave,
        void                  (*pFunctLSS)(uint8_t persistentBitRate, uint8_t persistentNodeID));



#ifdef __cplusplus
}
#endif /*__cplusplus*/

/** @} */
#endif
