/**
 * CAN module object for neuberger. + FreeRTOS.
 *
 * @file        CO_freertos_threads.c
 * @ingroup     CO_driver
 * @author      Martin Wagner
 * @copyright   2016 Neuberger Gebaeudeautomation GmbH
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

#include "os/freertos/include/FreeRTOS.h"
#include "os/freertos/include/task.h"

#include "CO_driver.h"
#include "CANopen.h"

/* Mainline thread (threadMain) ***************************************************/
static struct
{
  TickType_t interval_start; /* time value CO_process() was called last time */
  uint16_t interval_next;    /* calculated next timer interval */
  uint16_t interval;         /* max timer interval */
  TaskHandle_t id;           /* ID of the main thread */
} threadMain;

/**
 * This function resumes the main thread after an SDO event happened
 */
static void threadMain_resumeCallback(void)
{
  if (threadMain.id != 0) {
    xTaskAbortDelay(threadMain.id);
  }
}

void threadMain_init(uint16_t interval, TaskHandle_t threadMainID)
{
  threadMain.interval = interval;
  threadMain.interval_next = 1; /* do not block the first time. 0 is not allowed by the OS */
  threadMain.interval_start = xTaskGetTickCount();
  threadMain.id = threadMainID;
  CO_SDO_initCallback(CO->SDO[0], threadMain_resumeCallback);
  CO_EM_initCallback(CO->em, threadMain_resumeCallback);
}

void threadMain_close(void)
{

}

void threadMain_process(CO_NMT_reset_cmd_t *reset)
{
  uint16_t diff;
  uint16_t next;
  TickType_t now;
  TickType_t start;

  start = threadMain.interval_start;
  vTaskDelayUntil(&start, pdMS_TO_TICKS(threadMain.interval_next));
  now = xTaskGetTickCount();
  if (start > now) {
    /* vTaskDelayUntil returns the delay end time in value "start". If this time
     * is in the future, delay has been aborted by callback.
     * Unfortunately, vTaskDelayUntil() doesn't give us a exact time stamp it
     * was abortet, so we have to take a timestamp afterwards. This introduces
     * some inaccuracy. */
    start = now;
  }

  diff = (uint16_t)(start - threadMain.interval_start);

  do {
    next = threadMain.interval;
    *reset = CO_process(CO, diff, &next);
    diff = 0;
  } while ((*reset == CO_RESET_NOT) && (next == 0));

  /* prepare next call */
  threadMain.interval_next = next;
  threadMain.interval_start = start;
}

/* Realtime thread (threadRT) *****************************************************/
static struct {
  int16_t interval;          /* max timer interval */
  TickType_t interval_time;  /* time value CO_process() was called last time */
} threadRT;

void CANrx_threadTmr_init(uint16_t interval)
{
  threadRT.interval = interval;
  threadRT.interval_time = xTaskGetTickCount(); /* Processing is due now */
}

void CANrx_threadTmr_close(void)
{

}

void CANrx_threadTmr_process(void)
{
  int16_t timeout;
  uint32_t us_interval;
  TickType_t now;
  bool_t syncWas;
  CO_ReturnError_t result;

  /* This function waits for either can msg rx or interval timeout. Can driver
   * only takes timeout in ms but not timestamp, so we have to calculate that.
   * Using timeouts introduces some jitter in execution compared to timestamps. */

  /* Calculate delay time for rxWait() */
  now = xTaskGetTickCount();
  timeout = threadRT.interval - (int16_t)(now - threadRT.interval_time);
  if (timeout < 0) {
    timeout = 0;
  }

  result = CO_CANrxWait(CO->CANmodule[0], timeout);
  switch (result) {
    case CO_ERROR_TIMEOUT:
      CO_LOCK_OD();

      if(CO->CANmodule[0]->CANnormal == true) {

        us_interval = threadRT.interval * 1000;

#if CO_NO_SYNC == 1
        /* Process Sync */
        syncWas = CO_process_SYNC(CO, us_interval);
#else
        syncWas = false;
#endif

        /* Read inputs */
        CO_process_RPDO(CO, syncWas);

        /* Write outputs */
        CO_process_TPDO(CO, syncWas, us_interval);
      }

      CO_UNLOCK_OD();

      /* Calculate time of next execution. This ist done by adding interval to
       * now */
      threadRT.interval_time = threadRT.interval_time + threadRT.interval;
      break;
    case CO_ERROR_NO:
    default:
      /* Messages/Errors are processed inside rxWait() */
      break;
  }
}

/**
 * @}
 **/
