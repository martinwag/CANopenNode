/**
 * CAN module object for neuberger. + FreeRTOS.
 *
 * @file        crc.h
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


#ifndef CO_CRC_H
#define CO_CRC_H

/**
 * Our own implementation of the CANopenNode CRC function
 *
 * @param block Pointer to block of data.
 * @param blockLength Length of data in bytes;
 * @param crc Initial value (zero for xmodem). If block is split into
 * multiple segments, previous CRC is used as initial.
 *
 * @return Calculated CRC.
 */
unsigned short crc16_ccitt(
        const unsigned char     block[],
        unsigned int            blockLength,
        unsigned short          crc);


/** @} */
#endif
