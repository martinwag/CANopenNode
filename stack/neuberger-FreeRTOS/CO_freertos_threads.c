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
  TickType_t last_executed;  /* time value CO_process() was called last time */
  uint16_t interval_next;    /* calculated next timer interval */
  uint16_t interval;         /* max timer interval */
} threadMain;

void threadMain_init(uint16_t interval)
{
  threadMain.interval = interval;
  threadMain.interval_next = 1; /* do not block the first time. 0 is not allowed by the OS */
  threadMain.last_executed = xTaskGetTickCount();
}

void threadMain_close(void)
{

}

void threadMain_process(CO_NMT_reset_cmd_t *reset)
{
  uint16_t diff;
  uint16_t next;
  TickType_t now;

  now = threadMain.last_executed;
  vTaskDelayUntil(&now, pdMS_TO_TICKS(threadMain.interval_next));

  diff = (uint16_t)(now - threadMain.last_executed);

  next = threadMain.interval;
  do {
    *reset = CO_process(CO, diff, &next);
  } while ((*reset == CO_RESET_NOT) && (next == 0));

  threadMain.interval_next = next;
  threadMain.last_executed = now;
}

/* Realtime thread (threadRT) *****************************************************/
static struct {
  TickType_t last_executed;  /* time value CO_process() was called last time */
  uint16_t interval_next;    /* calculated next timer interval */
  uint16_t interval;         /* max timer interval */
} threadRT;

void CANrx_threadTmr_init(uint16_t interval)
{
  threadRT.interval = interval;
  threadRT.last_executed = xTaskGetTickCount();
  threadRT.interval_next = threadRT.interval;
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

  /* Calculate delay time for rxWait() */
  now = xTaskGetTickCount();
  timeout = threadRT.interval_next - (now - threadRT.last_executed);
  if (timeout < 0) {
    timeout = 0;
  }

  result = CO_CANrxWait(CO->CANmodule[0], timeout);
  switch (result) {
    case CO_ERROR_NO:
      /* Calculate remaining delay time */
      now = xTaskGetTickCount(); //todo ungenau, geht mit freertos aber scheinbar nicht besser
      threadRT.interval_next = threadRT.interval - (now - threadRT.last_executed);
      threadRT.last_executed = now;

      /* Message has already been processed inside rxWait() */

      break;
    case CO_ERROR_TIMEOUT:
      if(CO->CANmodule[0]->CANnormal == true) {

        us_interval = threadRT.interval * 1000;

        /* Process Sync and read inputs */
        syncWas = CO_process_SYNC_RPDO(CO, us_interval);

        /* Write outputs */
        CO_process_TPDO(CO, syncWas, us_interval);

        /* verify timer overflow */
        if(0) {
    //todo      CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
        }
      }

      /* Calculate time of execution. This ist done by adding interval time to
       * the last execution time */
      //todo wie geht das mit der Ungenauigkeit aus dem Nachricht Empfangen Teil zusammen? Addiert sich der Fehler auf oder wird er kompensiert?
      threadRT.last_executed = threadRT.last_executed + threadRT.interval;
      threadRT.interval_next = threadRT.interval;
      break;
    default:
      //todo error handling
      break;
  }
}

/**
 * @}
 **/
