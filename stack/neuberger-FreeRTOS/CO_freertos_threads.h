/**
 * CAN module object for neuberger. + FreeRTOS.
 *
 * @file        CO_freertos_threads.h
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
#ifndef CO_FREERTOS_THREADS_H_
#define CO_FREERTOS_THREADS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* This driver is loosely based upon the CO socketCAN driver
 * The "threads" inside this driver do not fork threads themselve, but require
 * that two threads are provided by the calling application.
 *
 * Like the CO socketCAN driver implementation, this driver uses the global CO
 * object and has one thread-local struct for variables. */

/**
 * Initialize mainline thread.
 *
 * threadMain is non-realtime thread for CANopenNode processing. It is blocking.
 * It blocks for a maximum of <interval> ms or less if necessary.
 * This thread processes CO_process() function from CANopen.c file.
 *
 * @param interval maximum interval in ms, recommended value: 50 ms
 * @param threadMainID ID of the thread that will run #threadMain_process()
 */
extern void threadMain_init(uint16_t interval, TaskHandle_t threadMainID);

/**
 * Cleanup mainline thread.
 */
extern void threadMain_close(void);

/**
 * Process mainline thread.
 *
 * This function must be called inside an infinite loop. It blocks until either
 * some event happens or a timer runs out.
 *
 * @param [out] reset return value from CO_process() function.
 */
extern void threadMain_process(CO_NMT_reset_cmd_t *reset);


/**
 * Initialize realtime thread.
 *
 * CANrx_threadTmr is realtime thread for CANopenNode processing. It is blocking.
 * It waits for either CAN message receive or <interval> ms timeout.
 * Inside interval it processes CANopen SYNC message, RPDOs(inputs)
 * and TPDOs(outputs).
 *
 * @param interval Interval of periodic timer in ms, recommended value for
 *                 realtime response: 1ms
 */
extern void CANrx_threadTmr_init(uint16_t interval);

/**
 * Cleanup realtime thread.
 */
extern void CANrx_threadTmr_close(void);

/**
 * Process realtime thread.
 *
 * This function must be called inside an infinite loop. It blocks until either
 * some event happens or a timer runs out.
 */
extern void CANrx_threadTmr_process(void);

/**
 * Disable CAN receive thread temporary.
 *
 * Function is called at SYNC message on CAN bus.
 * It disables CAN receive thread until RPDOs are processed.
 */
extern void CANrx_lockCbSync(bool_t syncReceived); //todo brauchen wir das?

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif

/**
* @}
**/
