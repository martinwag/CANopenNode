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
#include "CO_SDO.h"
#include "CO_Emergency.h"
#include "CO_NMT_Heartbeat.h"
#include "CO_LSSslave.h"

/*
 * Helper function - Check if two LSS addresses are equal
 */
static bool CO_LSSslave_addressEqual(CO_LSS_address_t *a1, CO_LSS_address_t *a2)
{
  if (a1->productCode == a2->productCode &&
      a1->revisionNumber == a2->revisionNumber &&
      a1->serialNumber == a2->serialNumber &&
      a1->vendorID == a2->vendorID) {
      return true;
  }
  return false;
}

/*
 * Helper function - Handle service "switch state global"
 */
static void CO_LSSslave_serviceSwitchStateGlobal(
    CO_LSSslave_t *LSSslave,
    CO_LSS_cs_t service,
    uint8_t mode)
{
    switch (mode) {
        case CO_LSS_STATE_WAITING:
            LSSslave->lssState = CO_LSS_STATE_WAITING;
            CO_memset((uint8_t*)&LSSslave->lssSelect, 0, sizeof(LSSslave->lssSelect));
            break;
        case CO_LSS_STATE_CONFIGURATION:
            LSSslave->lssState = CO_LSS_STATE_CONFIGURATION;
            break;
        default:
            break;
    }
}

/*
 * Helper function - Handle service "switch state selective"
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

            if (CO_LSSslave_addressEqual(&LSSslave->lssAddress, &LSSslave->lssSelect)) {
                LSSslave->lssState = CO_LSS_STATE_CONFIGURATION;

                /* send confirmation */
                LSSslave->TXbuff->data[0] = CO_LSS_SWITCH_STATE_SEL;
                CO_memset(&LSSslave->TXbuff->data[1], 0, 7);
                CO_CANsend(LSSslave->CANdevTx, LSSslave->TXbuff);
            }
            break;
        default:
            break;
    }
}

/*
 * Helper function - Handle service "configure"
 *
 * values inside message have different meaning, depending on the selected
 * configuration type
 */
static void CO_LSSslave_serviceConfig(
    CO_LSSslave_t *LSSslave,
    CO_LSS_cs_t service,
    const CO_CANrxMsg_t *msg)
{
    uint8_t nid;
    uint8_t tableSelector;
    uint8_t tableIndex;
    uint8_t errorCode;

    if(LSSslave->lssState != CO_LSS_STATE_CONFIGURATION) {
        return;
    }

    switch (service) {
        case CO_LSS_CFG_NODE_ID:
            nid = msg->data[1];

            if (CO_LSS_nodeIdValid(nid)) {
                LSSslave->pendingNodeID = nid;
                errorCode = CO_LSS_CFG_NODE_ID_OK;
            }
            else {
                errorCode = CO_LSS_CFG_NODE_ID_OUT_OF_RANGE;
            }

            /* send confirmation */
            LSSslave->TXbuff->data[0] = CO_LSS_CFG_NODE_ID;
            LSSslave->TXbuff->data[1] = errorCode;
            /* we do not use spec-error, always 0 */
            CO_memset(&LSSslave->TXbuff->data[2], 0, 6);
            CO_CANsend(LSSslave->CANdevTx, LSSslave->TXbuff);
            break;
        case CO_LSS_CFG_BIT_TIMING:
            if (LSSslave->pFunctLSScheckBitRate == NULL) {
                /* setting bit timing is not supported. Drop request */
                break;
            }

            tableSelector = msg->data[1];
            tableIndex = msg->data[2];

            if (tableSelector==0 && CO_LSS_bitTimingValid(tableIndex)) {
                uint16_t bit = CO_LSS_bitTimingTableLookup[tableIndex];
                bool_t bit_rate_supported  = LSSslave->pFunctLSScheckBitRate(
                    LSSslave->functLSScheckBitRateObject, bit);

                if (bit_rate_supported) {
                    LSSslave->pendingBitRate = bit;
                    errorCode = CO_LSS_CFG_BIT_TIMING_OK;
                }
                else {
                    errorCode = CO_LSS_CFG_BIT_TIMING_OUT_OF_RANGE;
                }
            }
            else {
                /* we currently only support CiA301 bit timing table */
                errorCode = CO_LSS_CFG_BIT_TIMING_OUT_OF_RANGE;
            }

            /* send confirmation */
            LSSslave->TXbuff->data[0] = CO_LSS_CFG_BIT_TIMING;
            LSSslave->TXbuff->data[1] = errorCode;
            /* we do not use spec-error, always 0 */
            CO_memset(&LSSslave->TXbuff->data[2], 0, 6);
            CO_CANsend(LSSslave->CANdevTx, LSSslave->TXbuff);
            break;
        case CO_LSS_CFG_ACTIVATE_BIT_TIMING:
            if (LSSslave->pFunctLSScheckBitRate == NULL) {
                /* setting bit timing is not supported. Drop request */
                break;
            }

            /* notify application */
            if (LSSslave->pFunctLSSactivateBitRate != NULL) {
              uint16_t delay = CO_getUint16(&msg->data[1]);
              LSSslave->pFunctLSSactivateBitRate(
                  LSSslave->functLSSactivateBitRateObject, delay);
            }
            break;
        case CO_LSS_CFG_STORE:
            LSSslave->TXbuff->data[0] = CO_LSS_CFG_STORE;
            LSSslave->TXbuff->data[1] = CO_LSS_CFG_STORE_OK;

            if (LSSslave->pFunctLSScfgStore == NULL) {
                /* setting bit timing is not supported. Reply error*/
                LSSslave->TXbuff->data[1] = CO_LSS_CFG_STORE_NOT_SUPPORTED;
            }
            else {
                bool_t result;
                /* Store "pending" to "persistent" */
                result = LSSslave->pFunctLSScfgStore(LSSslave->functLSScfgStore,
                    LSSslave->pendingNodeID, LSSslave->pendingBitRate);
                if (result != true) {
                    LSSslave->TXbuff->data[1] = CO_LSS_CFG_STORE_FAILED;
                }
            }
            /* send confirmation */
            /* we do not use spec-error, always 0 */
            CO_memset(&LSSslave->TXbuff->data[2], 0, 6);
            CO_CANsend(LSSslave->CANdevTx, LSSslave->TXbuff);
            break;
        default:
            break;
    }
}

/*
 * Helper function - Handle service "inquire"
 */
static void CO_LSSslave_serviceInquire(
    CO_LSSslave_t *LSSslave,
    CO_LSS_cs_t service)
{
    uint32_t value;

    if(LSSslave->lssState != CO_LSS_STATE_CONFIGURATION) {
        return;
    }

    switch (service) {
        case CO_LSS_INQUIRE_VENDOR:
            value = LSSslave->lssAddress.vendorID;
            break;
        case CO_LSS_INQUIRE_PRODUCT:
            value = LSSslave->lssAddress.productCode;
            break;
        case CO_LSS_INQUIRE_REV:
            value = LSSslave->lssAddress.revisionNumber;
            break;
        case CO_LSS_INQUIRE_SERIAL:
            value = LSSslave->lssAddress.serialNumber;
            break;
        case CO_LSS_INQUIRE_NODE_ID:
            value = (uint32_t)LSSslave->activeNodeID;
            break;
        default:
            return;
    }
    /* send response */
    LSSslave->TXbuff->data[0] = service;
    CO_setUint32(&LSSslave->TXbuff->data[1], value);
    CO_memset(&LSSslave->TXbuff->data[5], 0, 4);
    CO_CANsend(LSSslave->CANdevTx, LSSslave->TXbuff);
}

/*
 * Helper function - Handle service "identify"
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
        cs = msg->data[0];

        if (CO_LSS_cs_serviceIsSwitchStateGlobal(cs)) {
            uint8_t mode = msg->data[1];
            CO_LSSslave_serviceSwitchStateGlobal(LSSslave, cs, mode);
        }
        else if (CO_LSS_cs_serviceIsSwitchStateSelective(cs)) {
            uint32_t value = CO_getUint32(&msg->data[1]);
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
            uint32_t idNumber = CO_getUint32(&msg->data[1]);
            uint8_t bitCheck = msg->data[5];
            uint8_t lssSub = msg->data[6];
            uint8_t lssNext = msg->data[7];
            CO_LSSslave_serviceIdentFastscan(LSSslave, cs, idNumber, bitCheck, lssSub, lssNext);
        }
        else {
            /* No Ack -> Unsupported commands are dropped */
        }
    }
}


/******************************************************************************/
CO_ReturnError_t CO_LSSslave_init(
        CO_LSSslave_t          *LSSslave,
        CO_LSS_address_t        lssAddress,
        uint16_t                persistentBitRate,
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
        !CO_LSS_nodeIdValid(persistentNodeID)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CO_memcpy((uint8_t*)&LSSslave->lssAddress, (uint8_t*)&lssAddress, sizeof(LSSslave->lssAddress));
    LSSslave->lssState = CO_LSS_STATE_WAITING;
    CO_memset((uint8_t*)&LSSslave->lssSelect, 0, sizeof(LSSslave->lssSelect));

    LSSslave->pendingBitRate = persistentBitRate;
    LSSslave->pendingNodeID = persistentNodeID;
    LSSslave->activeNodeID = 0;
    LSSslave->pFunctLSScheckBitRate = NULL;
    LSSslave->functLSScheckBitRateObject = NULL;
    LSSslave->pFunctLSSactivateBitRate = NULL;
    LSSslave->functLSSactivateBitRateObject = NULL;
    LSSslave->pFunctLSScfgStore = NULL;
    LSSslave->functLSScfgStore = NULL;

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
void CO_LSSslave_initCheckBitRateCallback(
        CO_LSSslave_t          *LSSslave,
        void                   *object,
        bool_t                (*pFunctLSScheckBitRate)(void *object, uint16_t bitRate))
{
    if(LSSslave != NULL){
        LSSslave->pFunctLSScheckBitRate = pFunctLSScheckBitRate;
        LSSslave->functLSScheckBitRateObject = object;
    }
}


/******************************************************************************/
void CO_LSSslave_initActivateBitRateCallback(
        CO_LSSslave_t          *LSSslave,
        void                   *object,
        void                  (*pFunctLSSactivateBitRate)(void *object, uint16_t delay))
{
    if(LSSslave != NULL){
        LSSslave->pFunctLSSactivateBitRate = pFunctLSSactivateBitRate;
        LSSslave->functLSSactivateBitRateObject = object;
    }
}


/******************************************************************************/
void CO_LSSslave_initCfgStoreCallback(
        CO_LSSslave_t          *LSSslave,
        void                   *object,
        bool_t                (*pFunctLSScfgStore)(void *object, uint8_t id, uint16_t bitRate))
{
  if(LSSslave != NULL){
      LSSslave->pFunctLSScfgStore = pFunctLSScfgStore;
      LSSslave->functLSScfgStore = object;
  }
}


/******************************************************************************/
CO_LSSslave_cmd_t CO_LSSslave_process(
        CO_LSSslave_t          *LSSslave,
        uint16_t                activeBitRate,
        uint8_t                 activeNodeId,
        uint16_t               *pendingBitRate,
        uint8_t                *pendingNodeId)
{
    CO_LSSslave_cmd_t cmd = CO_LSS_SLAVE_CMD_NOT;

    LSSslave->activeNodeID = activeNodeId;
    *pendingBitRate = LSSslave->pendingBitRate;
    *pendingNodeId = LSSslave->pendingNodeID;

    if (activeNodeId==CO_LSS_NODE_ID_ASSIGNMENT &&
        LSSslave->pendingNodeID>=0x01 && LSSslave->pendingNodeID<=0x7f) {
        /* Normally, node id is applied by NMT master requesting comm reset. This
         * is not possible in this case as our NMT server is still in NMT reset
         * communication sub state. According to DSP 305 8.3.1, after setting a
         * valid node ID, we have to continue NMT initialization. */
        cmd = CO_LSS_SLAVE_CONTINUE_NMT_INIT;
    }

    /* Changing Bit Rate is done by callback functions */

    return cmd;
}

