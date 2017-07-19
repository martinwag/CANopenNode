/**
 * CAN module object for neuberger. + FreeRTOS.
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

#endif /* SRC_CANOPEN_CANOPENNODE_STACK_NEUBERGER_FREERTOS_CO_FREERTOS_THREADS_H_ */

/**
* @}
**/
