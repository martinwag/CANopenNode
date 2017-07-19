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

#include "canopen_storage.h"
#include "globdef.h"
#include "checksum.h"

CO_ReturnError_t Canopen_storage::load_od(void)
{
  u32 crc;

  /* Das sollte vom Compiler wegoptimiert werden da alles Konstanten sind */
  if ((od_reserved_size < sizeof(od)) || (remaining_size < 0)) {
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
  (void)storage.read(od_start, sizeof(od), reinterpret_cast<u8*>(&od));

  if (od.fw_id != globals.get_app_checksum()) {
    /* Daten von anderer FW Version oder Erststart */
    return CO_ERROR_NO;
  }

  crc = checksum_calculate_crc32(od.data, od_actual_size,
                                 CHECKSUM_CRC32_START_0xFFFFFFFF,
                                 CHECKSUM_CRC32_POLYNOM_ISO3309);
  if (crc != od.crc) {
    restore_od();
    return CO_ERROR_CRC;
  }
  (void)memcpy(reinterpret_cast<void*>(&CO_OD_EEPROM),
               reinterpret_cast<void*>(od.data), sizeof(CO_OD_EEPROM));
  return CO_ERROR_NO;
}

CO_ReturnError_t Canopen_storage::save_od(void)
{
  u32 crc;
  u32 fw_id;
  nvmem_state_t state;

  /* Neuen Datenblock generieren */
  od.fw_id = globals.get_app_checksum();
  (void)memcpy(reinterpret_cast<void*>(od.data),
               reinterpret_cast<void*>(&CO_OD_EEPROM), sizeof(od.data));
  od.crc = checksum_calculate_crc32(od.data, sizeof(od.data),
                                    CHECKSUM_CRC32_START_0xFFFFFFFF,
                                    CHECKSUM_CRC32_POLYNOM_ISO3309);

  /* M"ussen wir einen Schreibvorgang ausl"osen? */
  (void)storage.read(od_start + 0, sizeof(fw_id), reinterpret_cast<u8*>(&fw_id));
  (void)storage.read(od_start + offsetof(struct od, crc),
                     sizeof(crc), reinterpret_cast<u8*>(&crc));
  if ((od.fw_id == fw_id) && (od.crc == crc)) {
    return CO_ERROR_NO;
  }

  /* Wir schreiben immer den gesamten Block. Der CRC ist am Ende des Datenblocks
   * und schaltet die Daten g"ultig. */
  state = storage.write(od_start, sizeof(od), reinterpret_cast<u8*>(&od));
  if (state != NVMEM_OK) {
    return CO_ERROR_DATA_CORRUPT;
  }
  return CO_ERROR_NO;
}

void Canopen_storage::restore_od(void)
{
  const u32 dummy = 0;

  /* Wir "uberschreiben die Kennung, dieses triggert das beim n"achsten Reset
   * die alte Konfig nicht geladen wird. */
  (void)storage.write(od_start, sizeof(dummy), reinterpret_cast<const u8*>(&dummy));
}

CO_ReturnError_t Canopen_storage::load_lss(u8 *p_nid, u16 *p_bit)
{
  u32 crc;
  struct lss lss;

  /* Das sollte vom Compiler wegoptimiert werden da alles Konstanten sind */
  if ((lss_reserved_size < sizeof(lss)) || (remaining_size < 0)) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  /* Daten lesen */
  (void)storage.read(lss_start, sizeof(lss), reinterpret_cast<u8*>(&lss));

  crc = checksum_calculate_crc32(lss.data, lss_actual_size,
                                 CHECKSUM_CRC32_START_0xFFFFFFFF,
                                 CHECKSUM_CRC32_POLYNOM_ISO3309);
  if (crc != lss.crc) {
    *p_nid = 0;
    *p_bit = 0;
    return CO_ERROR_CRC;
  }
  *p_nid = lss.data[0];
  *p_bit = lss.data[1] << 8 | lss.data[2];

  return CO_ERROR_NO;
}

CO_ReturnError_t Canopen_storage::save_lss(u8 nid, u16 bit)
{
  u32 crc;
  struct lss lss;
  nvmem_state_t state;

  /* Neuen Datenblock generieren */
  lss.data[0] = nid;
  lss.data[1] = (u8)(bit >> 8);
  lss.data[2] = (u8)(bit);
  lss.crc = checksum_calculate_crc32(lss.data, sizeof(lss.data),
                                     CHECKSUM_CRC32_START_0xFFFFFFFF,
                                     CHECKSUM_CRC32_POLYNOM_ISO3309);

  /* M"ussen wir einen Schreibvorgang ausl"osen? */
  (void)storage.read(lss_start + offsetof(struct lss, crc),
                     sizeof(crc), reinterpret_cast<u8*>(&crc));
  if (lss.crc == crc) {
    return CO_ERROR_NO;
  }

  /* Wir schreiben immer den gesamten Block. Der CRC ist am Ende des Datenblocks
   * und schaltet die Daten g"ultig. */
  state = storage.write(lss_start, sizeof(lss), reinterpret_cast<u8*>(&lss));
  if (state != NVMEM_OK) {
    return CO_ERROR_DATA_CORRUPT;
  }
  return CO_ERROR_NO;
}

CO_ReturnError_t Canopen_storage::load_test(u8 *p_data, u16 *p_size)
{
  u32 crc;
  struct test test;

  if ((test_reserved_size < (sizeof(test) + *p_size)) || (remaining_size < 0)) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  /* Header lesen */
  (void)storage.read(test_start, sizeof(test), reinterpret_cast<u8*>(&test));
  /* "size" ist nicht per CRC gesichert -> Plausibilit"atstest */
  if (test.size > test_reserved_size) {
    return CO_ERROR_CRC;
  }

  /* wir lesen den kleineren der beiden Bereiche */
  if (*p_size > test.size) {
    *p_size = test.size;
  }
  test.size = *p_size;

  (void)storage.read(test_start + sizeof(test.size), test.size, p_data);
  crc = checksum_calculate_crc32(p_data, test.size,
                                 CHECKSUM_CRC32_START_0xFFFFFFFF,
                                 CHECKSUM_CRC32_POLYNOM_ISO3309);
  if (crc != test.crc) {
    *p_size = 0;
    return CO_ERROR_CRC;
  }

  return CO_ERROR_NO;
}

CO_ReturnError_t Canopen_storage::save_test(const u8 *p_data, u16 size)
{
  u32 crc;
  struct test test;
  nvmem_state_t state;

  if ((test_reserved_size < (sizeof(test) + size)) || (remaining_size < 0)) {
    return CO_ERROR_OUT_OF_MEMORY;
  }

  /* Neuen Datenblock generieren */
  test.size = size;
  test.crc = checksum_calculate_crc32(p_data, test.size,
                                      CHECKSUM_CRC32_START_0xFFFFFFFF,
                                      CHECKSUM_CRC32_POLYNOM_ISO3309);

  /* M"ussen wir einen Schreibvorgang ausl"osen? */
  (void)storage.read(test_start + offsetof(struct test, crc),
                     sizeof(crc), reinterpret_cast<u8*>(&crc));
  if (test.crc == crc) {
    return CO_ERROR_NO;
  }

  /* Wir schreiben immer den gesamten Block. Der CRC wird zuletzt geschrieben
   * und schaltet die Daten g"ultig. */
  state = storage.write(test_start + sizeof(test), test.size,
                        reinterpret_cast<u8*>(&test));
  if (state != NVMEM_OK) {
    return CO_ERROR_DATA_CORRUPT;
  }
  state = storage.write(test_start, sizeof(test), reinterpret_cast<u8*>(&test));
  if (state != NVMEM_OK) {
    return CO_ERROR_DATA_CORRUPT;
  }
  return CO_ERROR_NO;
}


/**
* @} @}
**/
