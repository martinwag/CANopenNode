/**
 * CANopen LSS Master/Slave protocol.
 *
 * @file        CO_LSSmaster.h
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


#ifndef CO_LSSmaster_H
#define CO_LSSmaster_H

#ifdef __cplusplus
extern "C" {
#endif

#if CO_NO_LSS_CLIENT == 1

#include "CO_LSS.h"

/**
 * @addtogroup CO_LSS
 * @defgroup CO_LSSmaster LSS Master
 * @ingroup CO_LSS
 * @{
 */

/**
 * Return values of LSS master functions.
 */
typedef enum {
    CO_LSSmaster_FASTSCAN_FINISHED  = 2,    /**< No more unconfigured slaves found */
    CO_LSSmaster_WAITING            = 1,    /**< No response arrived from server yet */
    CO_LSSmaster_OK                 = 0,    /**< Success, end of communication */
    CO_LSSmaster_WRONG_ARGUMENTS    = -1,   /**< Invalid argument */
    CO_LSSmaster_FASTSCAN_NO_ACK    = -2,   /**< No slaves found with given arguments */
    CO_LSSmaster_NO_ACK             = -3,   /**< Client rejected request */
    CO_LSSmaster_TIMEOUT            = -4    /**< No reply received */
} CO_LSSmaster_return_t;


/**
 * LSS master object.
 */
typedef struct{
    uint16_t              timeout;          /**< LSS response timeout in ms */

    uint8_t               lssState;         /**< #CO_LSS_state_t of the currently active slave */
    CO_LSS_address_t      lssAddress;       /**< #CO_LSS_address_t of the currently active slave */

}CO_LSSmaster_t;


/**
 * Initialize LSS object.
 *
 * Function must be called in the communication reset section. todo?
 * 
 * @return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT. todo
 */
CO_ReturnError_t CO_LSSmaster_init(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeout_ms,
        CO_CANmodule_t         *CANdevRx,
        uint16_t                CANdevRxIdx,
        uint32_t                CANidLssMaster,
        CO_CANmodule_t         *CANdevTx,
        uint16_t                CANdevTxIdx,
        uint32_t                CANidLssSlave);

/**
 * Change LSS master timeout
 *
 * On LSS, a "negative ack" is signaled by the slave not answering. Because of
 * that, a low timeout value can significantly increase initialization speed in
 * some cases (e.g. fastscan). However, as soon as there is activity on the bus,
 * LSS messages can be delayed because of their high COB (see #CO_Default_CAN_ID_t).
 *
 * @remark Be aware that a "late response" will seriously mess up LSS, so this
 * value must be selected "as high as necessary and as low as possible". CiA does
 * neither specify nor recommend a value.
 *
 * @param LSSmaster This object.
 * @param timeout
 */
void CO_LSSmaster_changeTimeout(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeout_ms);


/**
 * Request LSS switch state select
 *
 * This function can select a specific or all slaves.
 *
 * Function must be called cyclically until it returns <=0. Function is
 * non-blocking.
 *
 * @remark Only one selection can be active at any time.
 *
 * @param LSSmaster This object.
 * @param timeDifference_ms Time difference from previous function call in
 * [milliseconds]. Zero when request is started.
 * @param lssAddress LSS target address. If NULL, all slaves are selected
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_switchStateSelect(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeDifference_ms,
        CO_LSS_address_t       *lssAddress);


/**
 * Request LSS switch state deselect
 *
 * This function deselects all slaves, so it doesn't matter if a specific
 * device is selected.
 *
 * @param LSSmaster This object.
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_switchStateDeselect(
        CO_LSSmaster_t         *LSSmaster);


/**
 * Request LSS configure Bit Timing
 *
 * The new bit rate is set as new pending value.
 *
 * This function needs one specific node to be selected.
 *
 * Function must be called cyclically until it returns <=0. Function is
 * non-blocking.
 *
 * @param LSSmaster This object.
 * @param timeDifference_ms Time difference from previous function call in
 * [milliseconds]. Zero when request is started.
 * @param bit new bit rate
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_configureBitTiming(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeDifference_ms,
        uint16_t                bit);


/**
 * Request LSS configure node ID
 *
 * The new node id is set as new pending node ID.
 *
 * This function needs one specific node to be selected.
 *
 * Function must be called cyclically until it returns <=0. Function is
 * non-blocking.
 *
 * @param LSSmaster This object.
 * @param timeDifference_ms Time difference from previous function call in
 * [milliseconds]. Zero when request is started.
 * @param nodeId new node ID. Special value #CO_LSS_NODE_ID_ASSIGNMENT can be
 * used to invalidate node ID.
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_configureNodeId(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeDifference_ms,
        uint8_t                 nodeId);


/**
 * Request LSS store configuration
 *
 * The current "pending" values for bit rate and node ID in LSS slave are
 * stored to NVM.
 *
 * This function needs one specific node to be selected.
 *
 * Function must be called cyclically until it returns <=0. Function is
 * non-blocking.
 *
 * @param LSSmaster This object.
 * @param timeDifference_ms Time difference from previous function call in
 * [milliseconds]. Zero when request is started.
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_configureStore(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeDifference_ms);


/**
 * Request LSS activate bit timing
 *
 * The current "pending" bit rate in LSS slave is applied
 * Be aware that changing the bit rate is a critical step for the network. A
 * failure will render the network unusable!
 *
 * This function needs all slaves to be selected.
 *
 * @param LSSmaster This object.
 * @param switchDelay_ms delay that is applied by the slave once before and
 * once after switching in [milliseconds].
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_ActivateBit(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                switchDelay_ms);


/**
 * Request LSS inquire LSS address
 *
 * The LSS address value is read from the slave. This is useful when the slave
 * was selected by fastscan.
 *
 * This function needs one specific node to be selected.
 *
 * Function must be called cyclically until it returns <=0. Function is
 * non-blocking.
 *
 * @param LSSmaster This object.
 * @param timeDifference_ms Time difference from previous function call in
 * [milliseconds]. Zero when request is started.
 * @param lssAddress [out] read result when function returns successfully
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_InquireLssAddress(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeDifference_ms,
        CO_LSS_address_t       *lssAddress);


/**
 * Request LSS inquire node ID
 *
 * The node ID value is read from the slave.
 *
 * This function needs one specific node to be selected.
 *
 * Function must be called cyclically until it returns <=0. Function is
 * non-blocking.
 *
 * @param LSSmaster This object.
 * @param timeDifference_ms Time difference from previous function call in
 * [milliseconds]. Zero when request is started.
 * @param nodeId [out] read result when function returns successfully
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_InquireNodeId(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeDifference_ms,
        uint8_t                *nodeId);


/**
 * Request LSS identify fastscan
 *
 * This initiates searching for a node by the means of LSS fastscan mechanism.
 * When this function is finished
 * - a (more or less) arbitrary slave is selected
 * - no slave is selected because no slave matched the given criteria
 * - no slave is selected because all slaves are configured
 *
 * This function needs no node to be selected. //todo geht das "all configured" per "is selected??
 *
 * Function must be called cyclically until it returns <=0. Function is
 * non-blocking.
 *
 * @param LSSmaster This object.
 * @param timeDifference_ms Time difference from previous function call in
 * [milliseconds]. Zero when request is started.
 * @param lssAddressScanStart This is the LSS address at which the scan will
 * begin. It is usually the value that the last scan returned or "0" on first
 * scan.
 * @param lssAddressScanMatch This can be used to partly override scanning. If
 * a value != 0 is set on a element this value will be used without scanning.
 * This can be useful to e.g. scan for devices of a specific manufacturer.
 * @param lssAddressFound This contains the LSS address of the selected slave
 * when fastscan successfully selected a node.
 * @return todo
 */
CO_LSSmaster_return_t CO_LSSmaster_IdentifyFastscan(
        CO_LSSmaster_t         *LSSmaster,
        uint16_t                timeDifference_ms,
        CO_LSS_address_t       *lssAddressScanStart,
        CO_LSS_address_t       *lssAddressScanMatch,
        CO_LSS_address_t       *lssAddressFound);



#else /* CO_NO_LSS_CLIENT == 1 */

/**
 * @addtogroup CO_LSS
 * @{
 * If you need documetation for LSS master usage, add "CO_NO_LSS_CLIENT=1" to doxygen
 * "PREDEFINED" variable.
 *
 */

#endif /* CO_NO_LSS_CLIENT == 1 */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

/** @} */
#endif
