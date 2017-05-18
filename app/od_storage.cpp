/**
* @addtogroup io8000 template
* @{
* @addtogroup application
* @{
* @file od_storage.c
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

#include "od_storage.h"
#include "globdef.h"
#include "checksum.h"

CO_ReturnError_t od_storage::load(void)
{
  u32 crc;

  if (od_max_size < od_actual_size) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  if (defaults.FirstWord != CO_OD_FIRST_LAST_WORD) {
    /* Bootup, Startwerte sichern um Restore zu erm"oglichen */
    (void)memcpy(reinterpret_cast<void*>(&defaults),
                 reinterpret_cast<void*>(&CO_OD_EEPROM),
                 sizeof(defaults));
  }
  /* Startwerte wiederherstellen. Diese werden im weiteren Verlauf ggf.
   * "uberschrieben */
  (void)memcpy(reinterpret_cast<void*>(&CO_OD_EEPROM),
               reinterpret_cast<void*>(&defaults),
               sizeof(CO_OD_EEPROM));

  /* Daten lesen */
  (void)storage.read(storage.canopen_start,
                     sizeof(block), reinterpret_cast<u8*>(&block));

  if (block.fw_id != globals.get_app_checksum()) {
    /* Daten von anderer FW Version oder Erststart */
    return CO_ERROR_NO;
  }

  crc = checksum_calculate_crc32(block.data, od_actual_size,
                                 CHECKSUM_CRC32_START_0xFFFFFFFF,
                                 CHECKSUM_CRC32_POLYNOM_ISO3309);
  if (crc != block.crc) {
    restore();
    return CO_ERROR_CRC;
  }
  (void)memcpy(reinterpret_cast<void*>(&CO_OD_EEPROM),
               reinterpret_cast<void*>(block.data), sizeof(CO_OD_EEPROM));
  return CO_ERROR_NO;
}

CO_ReturnError_t od_storage::save(void)
{
  u32 crc;
  u32 fw_id;
  nvmem_state_t state;

  if (od_max_size < od_actual_size) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  /* Neuen Datenblock generieren */
  block.fw_id = globals.get_app_checksum();
  (void)memcpy(reinterpret_cast<void*>(block.data),
               reinterpret_cast<void*>(&CO_OD_EEPROM), sizeof(block.data));
  block.crc = checksum_calculate_crc32(block.data, sizeof(block.data),
                                       CHECKSUM_CRC32_START_0xFFFFFFFF,
                                       CHECKSUM_CRC32_POLYNOM_ISO3309);

  /* M"ussen wir einen Schreibvorgang ausl"osen? */
  (void)storage.read(storage.canopen_start + 0, sizeof(fw_id),
                     reinterpret_cast<u8*>(&fw_id));
  (void)storage.read(storage.canopen_start + offsetof(struct block, crc),
                     sizeof(crc), reinterpret_cast<u8*>(&crc));
  if ((block.fw_id == fw_id) && (block.crc == crc)) {
    return CO_ERROR_NO;
  }

  /* Wir schreiben immer den gesamten Block. Der CRC ist am Ende des Datenblocks
   * und schaltet die Daten g"ultig. */
  state = storage.write(storage.canopen_start,
                        sizeof(block), reinterpret_cast<u8*>(&block));
  if (state != NVMEM_OK) {
    return CO_ERROR_DATA_CORRUPT;
  }
  return CO_ERROR_NO;
}

void od_storage::restore(void)
{
  const u32 dummy = 0;

  /* Wir "uberschreiben die Kennung, dieses triggert das beim n"achsten Reset
   * die alte Konfig nicht geladen wird. */
  (void)storage.write(storage.canopen_start, sizeof(dummy),
                      reinterpret_cast<const u8*>(&dummy));
}

/**
* @} @}
**/
