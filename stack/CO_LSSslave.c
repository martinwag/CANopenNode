/*
 * CANopen LSS Slave protocol.
 *
 * @file        CO_LSSslave.c
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


#include "CO_driver.h"
#include "CO_LSSslave.h"
#include "CO_SDO.h"

/*
 * Handle service "switch state global"
 */
static void CO_LSSslave_serviceSwitchStateGlobal(
    CO_LSSslave_t *LSSslave,
    CO_LSS_cs_t service,
    uint8_t mode)
{
    switch (mode) {
        case 0:
            LSSslave->lssState = CO_LSS_STATE_WAITING;
            LSSslave->lssSelect.productCode = 0;
            LSSslave->lssSelect.revisionNumber = 0;
            LSSslave->lssSelect.serialNumber = 0;
            LSSslave->lssSelect.vendorID = 0;
            break;
        case 1:
            LSSslave->lssState = CO_LSS_STATE_CONFIGURATION;
            break;
        default:
            break;
    }
}

/*
 * Handle service "switch state selective"
 */
static void CO_LSSslave_serviceSwitchStateSelective(
    CO_LSSslave_t *LSSslave,
    CO_LSS_cs_t service,
    uint32_t value)
{
    if(LSSslave->lssState != CO_LSS_STATE_WAITING) {
        return;
    }

    switch (service) {
        case CO_LSS_SWITCH_STATE_SEL_VENDOR:
            LSSslave->lssSelect.vendorID = value;
            break;
        case CO_LSS_SWITCH_STATE_SEL_PRODUCT:
            LSSslave->lssSelect.productCode = value;
            break;
        case CO_LSS_SWITCH_STATE_SEL_REV:
            LSSslave->lssSelect.revisionNumber = value;
            break;
        case CO_LSS_SWITCH_STATE_SEL_SERIAL:
            LSSslave->lssSelect.serialNumber = value;

            if (LSSslave->lssAddress == LSSslave->lssSelect) {
                LSSslave->lssState = CO_LSS_STATE_CONFIGURATION;
                //todo lss response
            }
            break;
        default:
            break;
    }
}

/*
 * Handle service "configure"
 *
 * values inside message have different meaning, depending on the selected
 * configuration type
 */
static void CO_LSSslave_serviceConfig(
    CO_LSSslave_t *LSSslave,
    CO_LSS_cs_t service,
    const CO_CANrxMsg_t *msg)
{
    if(LSSslave->lssState != CO_LSS_STATE_WAITING) {
        return;
    }

    switch (service) {
        case CO_LSS_CFG_NODE_ID:
            uint8_t nid = *msg->data[1];

            if (CO_LSS_nodeIdValid(nid)) {
              LSSslave->pendingNodeID = nid;
              //todo lss response success
            } else {
              //todo lss response nid out of range
            }
            break;
        case CO_LSS_CFG_BIT_TIMING:
            uint8_t tableSelector = *msg->data[1];
            uint8_t tableIndex = *msg->data[2];

            if (tableSelector != 0) {
              /* we currently only support CiA301 bit timing table */
              //todo lss response ts out of range
              return;
            }

            if (CO_LSS_bitTimingValid(tableIndex)) {
                LSSslave->pendingBitRate = tableIndex;
            }
            break;
        case CO_LSS_CFG_ACTIVATE_BIT_TIMING:
            /* this needs all slaves on the network to be in this state. However,
             * it's in the responsibility of the master to ensure that. */
            uint16_t switchDelay = CO_getUint16(msg->data[1]);
            //todo switch bit rate
            break;
        case CO_LSS_CFG_STORE:
            //todo initiate writing persistent memory
            break;
        default:
            break;
    }
}

/*
 * Handle service "inquire"
 */
static void CO_LSSslave_serviceInquire(
    CO_LSSslave_t *LSSslave,
    CO_LSS_cs_t service)
{
    if(LSSslave->lssState != CO_LSS_STATE_WAITING) {
        return;
    }

    switch (service) {
        case CO_LSS_INQUIRE_VENDOR:
            //todo generate response
            break;
        case CO_LSS_INQUIRE_PRODUCT:
            //todo generate response
            break;
        case CO_LSS_INQUIRE_REV:
            //todo generate response
            break;
        case CO_LSS_INQUIRE_SERIAL:
            //todo generate response
            break;
        case CO_LSS_INQUIRE_NODE_ID:
            //todo generate response
            break;
        default:
            break;
    }
}

/*
 * Handle service "identify"
 */
static void CO_LSSslave_serviceIdentFastscan(
    CO_LSSslave_t *LSSslave,
    CO_LSS_cs_t service,
    uint32_t idNumber,
    uint8_t bitCheck,
    uint8_t lssSub,
    uint8_t lssNext)
{
    if(LSSslave->lssState == CO_LSS_STATE_WAITING) {
        //todo do fastscan
    }
}

/*
 * Read received message from CAN module.
 *
 * Function will be called (by CAN receive interrupt) every time, when CAN
 * message with correct identifier will be received. For more information and
 * description of parameters see file CO_driver.h.
 */
static void CO_LSSslave_receive(void *object, const CO_CANrxMsg_t *msg)
{
    CO_LSSslave_t *LSSslave;
    CO_LSS_cs_t cs;

    LSSslave = (CO_LSSslave_t*)object;   /* this is the correct pointer type of the first argument */

    if(msg->DLC == 8){
        cs = *msg->data[0];

        if (CO_LSS_cs_serviceIsSwitchStateGlobal(cs)) {
            uint8_t mode = *msg->data[1];
            CO_LSSslave_serviceSwitchStateGlobal(LSSslave, cs, mode);
        }
        else if (CO_LSS_cs_serviceIsSwitchStateSelective(cs)) {
            uint32_t value = CO_getUint32(msg->data[1]);
            CO_LSSslave_serviceSwitchStateSelective(LSSslave, cs, value);
        }
        else if (CO_LSS_cs_serviceIsConfig(cs)) {
            CO_LSSslave_serviceConfig(LSSslave, cs, msg);
        }
        else if (CO_LSS_cs_serviceIsInquire(cs)) {
            CO_LSSslave_serviceInquire(LSSslave, cs);
        }
        else if (CO_LSS_cs_serviceIsIdentFastscan(cs)) {
            /* we only support fastscan */
            uint32_t idNumber = CO_getUint32(msg->data[1]);
            uint8_t bitCheck = *msg->data[5];
            uint8_t lssSub = *msg->data[6];
            uint8_t lssNext = *msg->data[7];
            CO_LSSslave_serviceIdentFastscan(LSSslave, cs, idNumber, bitCheck, lssSub, lssNext);
        } else {
            /* Unsupported commands are dropped */
        }
    }
}


/******************************************************************************/
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
        uint32_t                CANidLssSlave)
{
    /* verify arguments */
    if (LSSslave==NULL || CANdevRx==NULL || CANdevTx==NULL ||
        !CO_LSS_nodeIdValid(persistentNodeID) || !CO_LSS_nodeIdValid(activeNodeId) ||
        !CO_LSS_bitTimingValid(persistentBitRate) || !CO_LSS_bitTimingValid(activeBitRate)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    LSSslave->lssAddress = lssAddress;
    LSSslave->lssState = CO_LSS_STATE_WAITING;
    LSSslave->lssSelect.productCode = 0;
    LSSslave->lssSelect.revisionNumber = 0;
    LSSslave->lssSelect.serialNumber = 0;
    LSSslave->lssSelect.vendorID = 0;

    LSSslave->activeBitRate = activeBitRate;
    LSSslave->persistentBitRate = persistentBitRate;
    LSSslave->activeNodeID = activeNodeId;
    LSSslave->persistentNodeID = persistentNodeID;

    /* configure LSS CAN Master message reception */
    CO_CANrxBufferInit(
            CANdevRx,             /* CAN device */
            CANdevRxIdx,          /* rx buffer index */
            CANidLssMaster,       /* CAN identifier */
            0x7FF,                /* mask */
            0,                    /* rtr */
            (void*)LSSslave,      /* object passed to receive function */
            CO_LSSslave_receive); /* this function will process received message */

    /* configure LSS CAN Slave response message transmission */
    LSSslave->CANdevTx = CANdevTx;
    LSSslave->TXbuff = CO_CANtxBufferInit(
            CANdevTx,             /* CAN device */
            CANdevTxIdx,          /* index of specific buffer inside CAN module */
            CANidLssSlave,        /* CAN identifier */
            0,                    /* rtr */
            8,                    /* number of data bytes */
            0);                   /* synchronous message flag bit */

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_LSSslave_initPersistanceCallback(
        CO_LSSslave_t          *LSSslave,
        void                  (*pFunctLSS)(uint8_t persistentBitRate, uint8_t persistentNodeID))
{
//    if(NMT != NULL){
//        NMT->pFunctNMT = pFunctNMT;
//        if(NMT->pFunctNMT != NULL){
//            NMT->pFunctNMT(NMT->operatingState);
//        }
//    }
}


/******************************************************************************/
CO_NMT_reset_cmd_t CO_LSSslave_process(
        CO_LSSslave_t          *LSSslave,
        const uint8_t          *activeBitRate,
        const uint8_t          *activeNodeId)
{

  return CO_RESET_NOT;
}

