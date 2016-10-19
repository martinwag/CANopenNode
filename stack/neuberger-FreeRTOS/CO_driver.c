/*
 * CAN module object for neuberger. + FreeRTOS.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster, Martin Wagner
 * @copyright   2004 - 2015 Janez Paternoster, 2016 Neuberger Gebaeudeautomation GmbH
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

#include <string.h>

#include "CO_driver.h"
#include "CO_Emergency.h"

#include "can.h"
#include "driver_defs.h"

/******************************************************************************/
void CO_CANsetConfigurationMode(int32_t CANbaseAddress)
{
  /* Put CAN module in configuration mode */
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
  /* Put CAN module in normal mode */
  if (CANmodule != NULL) {
    CANmodule->CANnormal = true;
  }
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule,
    int32_t CANbaseAddress, CO_CANrx_t rxArray[], uint16_t rxSize,
    CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate)
{
  uint16_t i;
  uint32_t tmp;
  can_state_t state;

  /* verify arguments */
  if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  /* Configure object variables */
  CANmodule->CANbaseAddress = CANbaseAddress;
  CANmodule->rxArray = rxArray;
  CANmodule->rxSize = rxSize;
  CANmodule->txArray = txArray;
  CANmodule->txSize = txSize;
  CANmodule->CANnormal = false;
  //todo CANmodule->useCANrxFilters = (rxSize <= 32U) ? true : false;/* microcontroller dependent */
  CANmodule->firstCANtxMessage = true;
  CANmodule->CANtxCount = 0U;
  CANmodule->errOld = 0U;
  CANmodule->em = NULL;

  for (i = 0U; i < rxSize; i++) {
    rxArray[i].ident = 0U;
    rxArray[i].pFunct = NULL;
  }
  for (i = 0U; i < txSize; i++) {
    txArray[i].bufferFull = false;
  }

  /* First time only configuration */
  if (CANmodule->driver == NULL) {

    /* Configure CAN module */
    CANmodule->driver = can_create(16, 16); //todo queusize woanders festlegen
    if (CANmodule->driver == NULL) {
      return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    state = can_init(CANmodule->driver, DRIVER_HW_TEMPLATE, CAN_MODULE_A);
    if (state != CAN_OK) {
      return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    tmp = 0;
    (void) can_ioctl(CANmodule->driver, CAN_SET_ERROR, &tmp); //todo kein Error Reporting? geht das irgendwie mit dem CO_driver?
    /* CANopenNode assumes tx non-block */ //todo wenn wir das auch so wollen, m"ussen wir uns um das rausschreiben der Objekte k"ummern wenn Queue voll war
    tmp = 0;
    (void) can_ioctl(CANmodule->driver, CAN_SET_TX_MODE, &tmp);
  }

  /* Configure CAN module hardware filters */
  if (CANmodule->useCANrxFilters) {
    /* CAN module filters are used, they will be configured with */
    /* CO_CANrxBufferInit() functions, called by separate CANopen */
    /* init functions. */
    /* Configure all masks so, that received message must match filter */
  } else {
    /* CAN module filters are not used, all messages with standard 11-bit */
    /* identifier will be received */
    /* Configure mask 0 so, that all messages with standard identifier are accepted */
  }

  return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
  /* turn off the module */
//todo brauchen wir das im Modul??
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index,
    uint16_t ident, uint16_t mask, bool_t rtr, void *object,
    void (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
  if ((CANmodule != NULL) && (object != NULL) && (pFunct != NULL)
      && (index < CANmodule->rxSize)) {
    /* buffer, which will be configured */
    CO_CANrx_t *buffer = &CANmodule->rxArray[index];

    /* Configure object variables */
    buffer->object = object;
    buffer->pFunct = pFunct;

    /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
    buffer->ident = ident & CAN_SFF_MASK;
    if (rtr) {
      buffer->ident |= CAN_RTR_FLAG;
    }
    buffer->mask = (mask & CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;

    /* Set CAN hardware module filter and mask. */
    if (CANmodule->useCANrxFilters) {
      //todo
    }
  } else {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  return CO_ERROR_NO;
}

/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index,
    uint16_t ident, bool_t rtr, uint8_t noOfBytes, bool_t syncFlag)
{
  CO_CANtx_t *buffer = NULL;

  if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
    /* get specific buffer */
    buffer = &CANmodule->txArray[index];

    /* CAN identifier, bit aligned with CAN module registers */
    buffer->ident = ident & CAN_SFF_MASK;
    if (rtr) {
      buffer->ident |= CAN_RTR_FLAG;
    }
    buffer->DLC = noOfBytes;
    buffer->syncFlag = syncFlag;
  }

  return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
  can_state_t state;

  if ((CANmodule != NULL) && (buffer != NULL)) {
    state = can_write(CANmodule->driver, (struct can_frame*) buffer);
    if (state != CAN_OK) {
      //todo log?
      //CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, n);
      return CO_ERROR_TX_OVERFLOW;
    }
  } else {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
  /* We do not support "pending" messages. A message is either already enqueued
   * for transmission inside the driver or dropped */
}

/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule)
{
//    uint16_t rxErrors, txErrors, overflow;
//    CO_EM_t* em = (CO_EM_t*)CANmodule->em;
//    uint32_t err;
//
//    /* get error counters from module. Id possible, function may use different way to
//     * determine errors. */
//    rxErrors = CANmodule->txSize;
//    txErrors = CANmodule->txSize;
//    overflow = CANmodule->txSize;
//
//    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;
//
//    if(CANmodule->errOld != err){
//        CANmodule->errOld = err;
//
//        if(txErrors >= 256U){                               /* bus off */
//            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
//        }
//        else{                                               /* not bus off */
//            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);
//
//            if((rxErrors >= 96U) || (txErrors >= 96U)){     /* bus warning */
//                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
//            }
//
//            if(rxErrors >= 128U){                           /* RX bus passive */
//                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
//            }
//            else{
//                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
//            }
//
//            if(txErrors >= 128U){                           /* TX bus passive */
//                if(!CANmodule->firstCANtxMessage){
//                    CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
//                }
//            }
//            else{
//                bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
//                if(isError){
//                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
//                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
//                }
//            }
//
//            if((rxErrors < 96U) && (txErrors < 96U)){       /* no error */
//                CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
//            }
//        }
//
//        if(overflow != 0U){                                 /* CAN RX bus overflow */
//            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
//        }
//    }
}

/******************************************************************************/
void CO_CANrxWait(CO_CANmodule_t *CANmodule)
{
  struct can_frame frame;
  can_state_t state;
  uint32_t i;
  uint32_t rx_id;
  CO_CANrx_t *buffer = NULL;
  bool_t matched = false;

  if (CANmodule == NULL) {
    return;
  }

  /* Wait for message */
  state = can_read(CANmodule->driver, &frame);
  if (state != CAN_OK) {
    //todo fehler behandeln?
    return;
  }

  if ((frame.can_dlc & CAN_EFF_MASK) != 0) {
    /* Drop extended Id Msg */
    return;
  }

  if ((frame.can_id & CAN_ERR_FLAG) != 0) {
    //todo was mit einem Errorframe machen?
    return;
  }

  rx_id = frame.can_id & CAN_SFF_MASK;
  if (CANmodule->useCANrxFilters) {
    /* CAN module filters are used. Message with known 11-bit identifier has */
    /* been received */
//    i = 0; /* get index of the received message here. Or something similar */
//    if (i < CANmodule->rxSize) {
//      buffer = &CANmodule->rxArray[i];
//      /* verify also RTR */
//      if (((rx_id ^ buffer->ident) & buffer->mask) == 0U) {
//        msgMatched = true;
//      }
//    } todo
  } else {
    /* CAN module filters are not used, message with any standard 11-bit identifier */
    /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
    buffer = &CANmodule->rxArray[0];
    for (i = CANmodule->rxSize; i > 0U; i--) {
      if (((rx_id ^ buffer->ident) & buffer->mask) == 0U) {
        matched = true;
        break;
      }
      buffer++;
    }

    /* Call specific function, which will process the message */
    if (matched && (buffer->pFunct != NULL)) {
      buffer->pFunct(buffer->object, (CO_CANrxMsg_t*) &frame);
    }
  }
}


