/**
 * CAN module object for neuberger. + FreeRTOS.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster, Martin Wagner
 * @copyright   2004 - 2015 Janez Paternoster, 2017 - 2020 Neuberger Gebaeudeautomation GmbH
 *
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>

#include "CO_driver.h"
#include "CO_Emergency.h"

#include "drivers/can.h"
#include "drivers/led.h"
#include "drivers/modtype.h"
#include "drivers/can_error.h"
#include "drivers/driver_defs.h"
#include "interface/log.h"

static const char CAN_ERR_MSG[] = "CAN err %d 0x%x";

SemaphoreHandle_t CO_EMCY_mtx = NULL; /* mutex type semaphore */
SemaphoreHandle_t CO_OD_mtx = NULL;   /* mutex type semaphore */
QueueHandle_t CO_ERR_queue = NULL;
static const u8 CO_ERR_queue_size = 5;

/******************************************************************************/
static inline void CO_CANSignalBusPermanentError(void)
{
  (void)led_set(LED_NAME_BUS_RED, LED_STATE_BLINK);
}

/******************************************************************************/
static void CO_CANSignalBusSingleError(void)
{
  led_state_t state = LED_STATE_OFF;

  (void)led_get(LED_NAME_BUS_RED, &state);
  if (state == LED_STATE_OFF) {
    (void)led_set(LED_NAME_BUS_RED, LED_STATE_PULSE);
  }
}

/******************************************************************************/
static inline void CO_CANSignalBusNoError(void)
{
  (void)led_set(LED_NAME_BUS_RED, LED_STATE_OFF);
}

/******************************************************************************/
static inline void CO_CANSignalRxTx(void)
{
  (void)led_set(LED_NAME_BUS_GREEN, LED_STATE_PULSE);
}

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANdriverState)
{
  /* Put CAN module in configuration mode */
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
  /* Put CAN module in normal mode */
  if (CANmodule != NULL) {
    can_flush(CANmodule->driver);
    CANmodule->CANnormal = true;
  }
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule,
    void *CANdriverState, CO_CANrx_t rxArray[], uint16_t rxSize,
    CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate)
{
  uint16_t i;
  uint32_t tmp;
  can_state_t state;
  can_baud_t baud;

  /* verify arguments */
  if ((CANmodule == NULL) || (rxArray == NULL) || (txArray == NULL)) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  switch (CANbitRate) {
    case 10:
      baud = CAN_BAUD_10;
      break;
    case 20:
      baud = CAN_BAUD_20;
      break;
    case 50:
      baud = CAN_BAUD_50;
      break;
    case 100:
      baud = CAN_BAUD_100;
      break;
    case 125:
      baud = CAN_BAUD_125;
      break;
    case 250:
      baud = CAN_BAUD_250;
      break;
    case 500:
      baud = CAN_BAUD_500;
      break;
    case 1000:
      baud = CAN_BAUD_1000;
      break;
    default:
      return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  /* Configure object variables */
  CANmodule->CANbaseAddress = (int32_t)CANdriverState;
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

  for(i=0U; i<rxSize; i++){
      rxArray[i].ident = 0U;
      rxArray[i].mask = 0xFFFFFFFFU;
      rxArray[i].object = NULL;
      rxArray[i].pFunct = NULL;
  }
  for(i=0U; i<txSize; i++){
      txArray[i].bufferFull = false;
  }

  /* First time only configuration */
  if (CO_EMCY_mtx == NULL) {
    CO_EMCY_mtx = xSemaphoreCreateMutex();
    if (CO_EMCY_mtx == NULL) {
      return CO_ERROR_OUT_OF_MEMORY;
    }
  }
  if (CO_OD_mtx == NULL) {
    CO_OD_mtx = xSemaphoreCreateMutex();
    if (CO_OD_mtx == NULL) {
      return CO_ERROR_OUT_OF_MEMORY;
    }
  }
  if (CO_ERR_queue == NULL) {
    CO_ERR_queue = xQueueCreate(CO_ERR_queue_size, sizeof(struct can_frame));
    if (CO_ERR_queue == NULL) {
      return CO_ERROR_OUT_OF_MEMORY;
    }
  }

  (void)led_setup_blink(LED_NAME_BUS_RED, CO_BUS_LED_BLINK, CO_BUS_LED_BLINK);
  (void)led_setup_blink(LED_NAME_BUS_GREEN, CO_BUS_LED_FLASH, 0);
  (void)led_set(LED_NAME_BUS_RED, LED_STATE_OFF);
  (void)led_set(LED_NAME_BUS_GREEN, LED_STATE_OFF);

  if (CANmodule->driver == NULL) {

    /* Configure CAN module */
    CANmodule->driver = can_create(CO_QUEUE_RX, CO_QUEUE_TX);
    if (CANmodule->driver == NULL) {
      return CO_ERROR_OUT_OF_MEMORY;
    }

    state = can_init(CANmodule->driver, MODTYPE_HW_TEMPLATE, CANmodule->CANbaseAddress);
    if (state != CAN_OK) {
      log_printf(LOG_DEBUG, CAN_ERR_MSG, __LINE__, state);
      return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    (void)can_ioctl(CANmodule->driver, CAN_SET_BAUDRATE, &baud);

    /* CANopenNode supports tx non-block by using the bufferFull flag, however
     * we do not take advantage of this. When the queue is full, all following
     * messages are dropped */
    tmp = 0;
    (void)can_ioctl(CANmodule->driver, CAN_SET_TX_MODE, &tmp);
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
  can_deinit(CANmodule->driver);
  can_free(CANmodule->driver);
  CANmodule->driver = NULL;
  xQueueReset(CO_ERR_queue);
  (void)led_set(LED_NAME_BUS_RED, LED_STATE_OFF);
  (void)led_set(LED_NAME_BUS_GREEN, LED_STATE_OFF);
}

/******************************************************************************/
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg)
{
    /* remove socketCAN flags */
    return (uint16_t) (rxMsg->ident & CAN_SFF_MASK);
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
      state = can_ioctl(CANmodule->driver, CAN_SET_FILTER, &filter);
      if (state != CAN_OK) {
        /* We don't have enough hardware filters, fall back to software
         * filtering */
        (void)can_ioctl(CANmodule->driver, CAN_SET_FILTER, NULL);
        CANmodule->useCANrxFilters = false;
        log_printf(LOG_WARNING, "Not enough CAN HW filters. Falling back to SW");
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
  CO_EM_t* em = (CO_EM_t*)CANmodule->em;

  if ((CANmodule != NULL) && (buffer != NULL)) {
    state = can_write(CANmodule->driver, (struct can_frame*) buffer);
    if (state != CAN_OK) {
      log_printf(LOG_DEBUG, CAN_ERR_MSG, __LINE__, state);
      CO_errorReport(em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
      CO_CANSignalBusSingleError();
      return CO_ERROR_TX_OVERFLOW;
    }
  } else {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  /* Tx successfull -> reset OF */
  CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, 0);

  CO_CANSignalRxTx();
  return CO_ERROR_NO;
}

/******************************************************************************/
CO_ReturnError_t CO_CANCheckSend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
  can_queue_info_t queue;

  if (CANmodule == NULL) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }
  can_ioctl(CANmodule->driver, CAN_GET_TX_QUEUE_INFO, &queue);
  if (queue.queue_remaining<=1 ||
      (queue.queue_remaining < (queue.queue_length / 2))) { //always round down
    return CO_ERROR_TX_BUSY;
  }
  return CO_CANsend(CANmodule, buffer);
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
  BaseType_t result;
  struct can_frame frame;
  CO_EM_t* em = (CO_EM_t*)CANmodule->em;

  result = xQueueReceive(CO_ERR_queue, &frame, 0);
  if (result == pdTRUE) {
    switch (frame.can_id & CAN_ERR_MASK) {
      case CAN_ERR_CRTL:
        if ((frame.data[1] & CAN_ERR_CRTL_RX_OVERFLOW) != 0) {
          CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
          CO_CANSignalBusSingleError();

        } else if ((frame.data[1] & CAN_ERR_CRTL_TX_OVERFLOW) != 0) {
          CO_errorReport(em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
          CO_CANSignalBusSingleError();

        } else if ((frame.data[1] & CAN_ERR_CRTL_RX_PASSIVE) != 0) {
          CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, 0);
          CO_CANSignalBusPermanentError();

        } else if ((frame.data[1] & CAN_ERR_CRTL_TX_PASSIVE) != 0) {
          CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, 0);
          CO_CANSignalBusPermanentError();

        } else if ((frame.data[1] & CAN_ERR_CRTL_ACTIVE) != 0) {
          /* active -> Busfehler quittieren */
          CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, 0);
          CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, 0);
          CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, 0);
          CO_CANSignalBusNoError();
        } else {
          /* Everyting else, eg. Warning level */
          CO_CANSignalBusSingleError();
        }
        break;
      case CAN_ERR_BUSOFF:
        /* wird verschickt wenn wir nicht mehr "Bus Off" sind */
        CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, 0);
        CO_CANSignalBusPermanentError();
        break;
    }
  }
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxWait(CO_CANmodule_t *CANmodule, uint16_t timeout)
{
  struct can_frame frame;
  can_state_t state;
  uint32_t i;
  CO_CANrx_t *buffer = NULL;
  CO_EM_t* em = (CO_EM_t*)CANmodule->em;
  bool_t matched = false;

  if (CANmodule == NULL) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  /* Wait for message */
  state = can_poll(CANmodule->driver, timeout);
  if (state == CAN_ERR_TIMEOUT) {
    return CO_ERROR_TIMEOUT;
  } else if (state != CAN_OK) {
    log_printf(LOG_DEBUG, CAN_ERR_MSG, __LINE__, state);
    CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
    CO_CANSignalBusSingleError();
    return CO_ERROR_RX_OVERFLOW;
  }

  state = can_read(CANmodule->driver, &frame);
  if (state != CAN_OK) {
    log_printf(LOG_DEBUG, CAN_ERR_MSG, __LINE__, state);
    CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
    CO_CANSignalBusSingleError();
    return CO_ERROR_RX_OVERFLOW;
  }

  if ((frame.can_id & CAN_ERR_FLAG) != 0) {
    (void)xQueueSend(CO_ERR_queue, &frame, 0);
    log_printf(LOG_DEBUG, CAN_ERR_MSG, __LINE__, frame.can_id);
    return CO_ERROR_NO;
  }

  /* Rx successfull -> reset OF */
  CO_errorReset(em, CO_EM_CAN_RXB_OVERFLOW, 0);

  /* The template supports hardware and software filtering modes. However,
   * hardware filtering mode requires to get filter match index from hardware,
   * which is not implemented in our driver (stm32 supports it) */
  buffer = &CANmodule->rxArray[0];
  for (i = CANmodule->rxSize; i > 0U; i--) {
    if ((((frame.can_id & CAN_EFF_MASK) ^ buffer->ident) & buffer->mask) == 0U) {
      matched = true;
      break;
    }
    buffer++;
  }

  /* Call specific function, which will process the message */
  if (matched && (buffer->pFunct != NULL)) {
    buffer->pFunct(buffer->object, (CO_CANrxMsg_t*) &frame);
    CO_CANSignalRxTx();
  }

  return CO_ERROR_NO;
}


