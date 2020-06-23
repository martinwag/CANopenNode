/**
 * CAN module object for neuberger. + FreeRTOS.
 *
 * @file        crc.c
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

#include "drivers/checksum.h"

/*
 * This function wraps the BSP CRC function to the CANopenNode signature.
 */
unsigned short crc16_ccitt(
        const unsigned char     block[],
        unsigned int            blockLength,
        unsigned short          crc)
{
  return checksum_calculate_crc16(block, blockLength, crc, CHECKSUM_CRC16_POLYNOM_CCITT16);
}
