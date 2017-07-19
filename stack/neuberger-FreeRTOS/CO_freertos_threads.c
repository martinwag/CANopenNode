/**
 * CAN module object for neuberger. + FreeRTOS.
 *
 * @file        CO_freertos_threads.c
 * @ingroup     CO_driver
 * @author      Martin Wagner
 * @copyright   2016 Neuberger Gebaeudeautomation GmbH
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

#include "FreeRTOS.h"
#include "task.h"

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

        /* Process Sync and read inputs */
        syncWas = CO_process_SYNC_RPDO(CO, us_interval);

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
