/**
 * CAN module object for neuberger boot loader.
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
    can_flush(&CANmodule->driver.can);
    CANmodule->CANnormal = true;
  }
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule,
    int32_t CANbaseAddress, CO_CANrx_t rxArray[], uint16_t rxSize,
    CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate)
{
  uint16_t i;
  can_state_t state;

  /* verify arguments */
  if ((CANmodule == NULL) || (rxArray == NULL) || (txArray == NULL)) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  /* Configure object variables */
  CANmodule->CANbaseAddress = CANbaseAddress;
  CANmodule->rxArray = rxArray;
  CANmodule->rxSize = rxSize;
  CANmodule->txArray = txArray;
  CANmodule->txSize = txSize;
  CANmodule->CANnormal = false;
  CANmodule->useCANrxFilters = true;
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

  /* Configure CAN module */
  if (CANmodule->driver.initialized == false) {
    state = can_init(&CANmodule->driver.can, MODTYPE_HW_TEMPLATE, CANbaseAddress);
    if (state != CAN_OK) {
      return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    CANmodule->driver.initialized = true;
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
  /* keine weiteren Aktionen */
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index,
    uint16_t ident, uint16_t mask, bool_t rtr, void *object,
    void (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
  struct can_filter filter;
  can_state_t state;

  if ((CANmodule != NULL) && (object != NULL) && (pFunct != NULL)
      && (index < CANmodule->rxSize)) {
    /* buffer, which will be configured */
    CO_CANrx_t *buffer = &CANmodule->rxArray[index];

    /* Configure object variables */
    buffer->object = object;
    buffer->pFunct = pFunct;

    /* CAN identifier and CAN mask, bit aligned with CAN module. */
    buffer->ident = ident & CAN_SFF_MASK;
    if (rtr) {
      buffer->ident = buffer->ident | CAN_RTR_FLAG;
    }
    buffer->mask = (mask & CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;

    /* Set CAN hardware module filter and mask. */
    if (CANmodule->useCANrxFilters) {
      filter.can_id = buffer->ident;
      filter.can_mask = buffer->mask;
      state = can_ioctl(&CANmodule->driver.can, CAN_SET_FILTER, &filter);
      if (state != CAN_OK) {
        /* We don't have enough hardware filters, fall back to software
         * filtering */
        (void)can_ioctl(&CANmodule->driver.can, CAN_SET_FILTER, NULL);
        CANmodule->useCANrxFilters = false;
      }
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
    state = can_write(&CANmodule->driver.can, (struct can_frame*) buffer);
    if (state != CAN_OK) {;
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
  /* No error handling in bootloader */
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxWait(CO_CANmodule_t *CANmodule, uint16_t timeout)
{
  struct can_frame frame;
  can_state_t state;
  uint32_t i;
  uint32_t rx_id;
  CO_CANrx_t *buffer = NULL;
  bool_t matched = false;

  if (CANmodule == NULL) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  /* Wait for message */
  state = can_poll(&CANmodule->driver.can, timeout);
  if (state == CAN_ERR_TIMEOUT) {
    return CO_ERROR_TIMEOUT;
  }

  state = can_read(&CANmodule->driver.can, &frame);
  if (state != CAN_OK) {
    can_flush(&CANmodule->driver.can);
    return CO_ERROR_RX_OVERFLOW;
  }

  if ((frame.can_dlc & CAN_EFF_FLAG) != 0) {
    /* Drop extended Id Msg */
    return CO_ERROR_NO;
  }

  /* The template supports hardware and software filtering modes. However,
   * hardware filtering mode requires to get filter match index from hardware,
   * which is not implemented in our driver (stm32 supports it) */
  rx_id = frame.can_id & CAN_SFF_MASK;
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
  return CO_ERROR_NO;
}


