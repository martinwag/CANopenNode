/**
* @addtogroup io8000 template
* @{
* @addtogroup application
* @{
* @file canopen_storage.cpp
* @copyright Neuberger Gebäudeautomation GmbH
* @author mwagner
* @brief Ablage der Parameter im Festwertspeicher
*
* @details \b Programm-Name template
* @details Dieses Modul verwaltet die Ablage der CANopen
* Parameter im Festwertspeicher
**/

#include <cstddef>
#include <string.h>
#include <stdlib.h>

#include "canopen_storage.h"
#include "globdef.h"
#include "checksum.h"
#include "log.h"
#include "errors.h"

CO_ReturnError_t Canopen_storage_type::load(
    u16 start, u16 reserved, u16 size, u8 *p_work, u8 *p_to)
{
  u32 crc;
  u32 crc_read;

  if (reserved < (size + sizeof(crc_read))) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  /* Daten lesen. Der CRC ist an das Ende des Datenbereichs angeh"angt  */
  (void)storage.read(start, size + sizeof(crc_read), p_work);

  crc = checksum_calculate_crc32(p_work, size,
                                 CHECKSUM_CRC32_START_0xFFFFFFFF,
                                 CHECKSUM_CRC32_POLYNOM_ISO3309);
  crc_read = *reinterpret_cast<u32*>(p_work + size);
  if (crc != crc_read) {
    return CO_ERROR_CRC;
  }
  /* Daten sind g"ultig, in Ausgabepuffer "ubernehmen */
  (void)memcpy(reinterpret_cast<void*>(p_to),
               reinterpret_cast<void*>(p_work), size);
  return CO_ERROR_NO;
}

CO_ReturnError_t Canopen_storage_type::save(
    u16 start, u16 reserved, u16 size, u8 *p_work, const u8 *p_from)
{
  u32 crc_write;
  u32 crc_read;
  nvmem_state_t state;

  if (reserved < (size + sizeof(crc_write))) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  crc_write = checksum_calculate_crc32(p_from, size,
                                       CHECKSUM_CRC32_START_0xFFFFFFFF,
                                       CHECKSUM_CRC32_POLYNOM_ISO3309);

  /* M"ussen wir einen Schreibvorgang ausl"osen? */
  (void)storage.read(start + size, sizeof(crc_read),
                     reinterpret_cast<u8*>(&crc_read));
  if (crc_read == crc_write) {
    return CO_ERROR_NO;
  }

  /* Wir schreiben immer den gesamten Block. Der CRC ist am Ende des Datenblocks
   * und schaltet die Daten g"ultig. */
  (void)memcpy(reinterpret_cast<void*>(p_work),
               reinterpret_cast<const void*>(p_from), size);
  (void)memcpy(reinterpret_cast<void*>(p_work + size),
               reinterpret_cast<const void*>(&crc_write), sizeof(crc_write));
  state = storage.write(start, size + sizeof(crc_write), p_work);
  if (state != NVMEM_OK) {
    return CO_ERROR_DATA_CORRUPT;
  }
  return CO_ERROR_NO;
}

void Canopen_storage_type::erase(u16 start, u16 size)
{
  u8 tmp[16];
  u16 remaining;

  (void)memset(reinterpret_cast<void*>(tmp), 0xff, sizeof(tmp));

  remaining = size;
  do {
    if (remaining >= sizeof(tmp)) {
      size = sizeof(tmp);
    } else {
      size = remaining;
    }

    (void)storage.write(start, size, tmp);

    remaining = remaining - size;
  } while (remaining > 0);
}

void Canopen_storage::lock(void)
{
  if (this->in_use == NULL) {
    this->in_use = xSemaphoreCreateMutex();
  }
  (void)xSemaphoreTake(this->in_use, portMAX_DELAY);
}

void Canopen_storage::unlock(void)
{
  (void)xSemaphoreGive(this->in_use);
}

CO_ReturnError_t Canopen_storage::load(storage_type_t type)
{
  CO_ReturnError_t result;

  if (this->remaining_size < 0) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  lock();

  /* Daten lesen. Sind diese korrekt, so werden die aktuellen Werte
   * "uberschrieben */
  result = Canopen_storage_type::load(this->start[type], this->reserved_size[type],
                                      this->actual_size[type], this->work,
                                      this->p_ram[type]);

  unlock();

  return result;
}

CO_ReturnError_t Canopen_storage::save(storage_type_t type)
{
  CO_ReturnError_t result;

  if (this->remaining_size < 0) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  lock();

  result =  Canopen_storage_type::save(this->start[type], this->reserved_size[type],
                                       this->actual_size[type], this->work,
                                       this->p_ram[type]);

  unlock();

  return result;
}

void Canopen_storage::restore(storage_type_t type)
{
  if (this->remaining_size < 0) {
    return;
  }

  lock();

  Canopen_storage_type::erase(this->start[type], this->reserved_size[type]);
  /* Der eigentliche Restore wird erst beim n"achsten NMT reset comm/app
   * durchgef"uhrt */

  unlock();
}

/**
* @} @}
**/
