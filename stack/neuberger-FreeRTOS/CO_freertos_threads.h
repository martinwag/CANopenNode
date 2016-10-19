/**
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_freertos_threads.h
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
#ifndef SRC_CANOPEN_CANOPENNODE_STACK_NEUBERGER_FREERTOS_CO_FREERTOS_THREADS_H_
#define SRC_CANOPEN_CANOPENNODE_STACK_NEUBERGER_FREERTOS_CO_FREERTOS_THREADS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* This driver is loosely based upon the CO socketCAN driver
 * The "threads" inside this driver do not fork threads themselve, but require
 * that two threads are provided by the calling application.
 *
 * Like the CO socketCAN driver implementation, this driver uses the global CO
 * object. */

/**
 * Initialize mainline thread.
 *
 * threadMain is non-realtime thread for CANopenNode processing. It is blocking.
 * It blocks for a maximum of 50 ms or less if necessary.
 * It uses FreeRTOS vTaskDelayUntil for exact timings.
 * This thread processes CO_process() function from CANopen.c file.
 *
 * @param maxTime Pointer to variable, where longest interval will be written
 * [in milliseconds]. If NULL, calculations won't be made. //todo brauchen wir das?
 */
void threadMain_init(uint16_t *maxTime);

/**
 * Cleanup mainline thread.
 */
void threadMain_close(void);

/**
 * Process mainline thread.
 *
 * This function must be called inside an infinite loop. It blocks until either
 * some event happens or a timer runs out.
 *
 * @param reset return value from CO_process() function.
 */
void threadMain_process(int fd, CO_NMT_reset_cmd_t *reset, uint16_t timer1ms);

/**
 * Signal function, which triggers mainline thread.
 *
 * It is used from some CANopenNode objects as callback.
 */
void threadMain_cbSignal(void);

/**
 * Initialize realtime thread.
 *
 * CANrx_threadTmr is realtime thread for CANopenNode processing. It is blocking
 * and is executing on CAN message receive or periodically in <intervalns>
 * intervals. Inside interval is processed CANopen SYNC message, RPDOs(inputs)
 * and TPDOs(outputs). Between inputs and outputs can also be executed some
 * realtime application code.
 * CANrx_threadTmr uses can driver poll functionality
 *
 * @param intervalns Interval of periodic timer in nanoseconds.
 * @param maxTime Pointer to variable, where longest interval will be written
 * [in milliseconds]. If NULL, calculations won't be made. //todo was macht die?
 */
void CANrx_threadTmr_init(int fdEpoll, long intervalns, uint16_t *maxTime);

/**
 * Cleanup realtime thread.
 */
void CANrx_threadTmr_close(void);

/**
 * Process realtime thread.
 *
 * This function must be called inside an infinite loop. It blocks until either
 * some event happens or a timer runs out.
 */
void CANrx_threadTmr_process(void);

/**
 * Disable CAN receive thread temporary.
 *
 * Function is called at SYNC message on CAN bus.
 * It disables CAN receive thread until RPDOs are processed.
 */
void CANrx_lockCbSync(bool_t syncReceived); //todo brauchen wir das?

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* SRC_CANOPEN_CANOPENNODE_STACK_NEUBERGER_FREERTOS_CO_FREERTOS_THREADS_H_ */

/**
* @}
**/
