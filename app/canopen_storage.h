/**
* @addtogroup io8000 template
* @{
* @addtogroup application
* @{
* @file canopen_storage.h
* @copyright Neuberger Gebäudeautomation GmbH
* @author mwagner
* @brief Ablage der Parameter im Festwertspeicher
*
* @details \b Programm-Name template
* @details Dieses Modul verwaltet die Ablage der CANopen
* Parameter im Festwertspeicher
**/
#ifndef SRC_CANOPEN_CANOPEN_NVM_H_
#define SRC_CANOPEN_CANOPEN_NVM_H_

#include "CANopen.h"

#include "globdef.h"

/**
 * Ablage der CO Parameter im EEPROM. Die Funktionalit"at k"onnten wir
 * erben, eine zweite Instanz der HW ist aber nicht m"oglich...
 *
 * wir orientieren uns an der Vorlage eeprom.c/h aus dem Stack Treiberbeispiel
 */
class Canopen_storage {
  protected:
    static const u16 max_size = storage.canopen_size;
    static const u16 od_reserved_size = 1024 * 2;
    static const u16 od_actual_size = sizeof(struct sCO_OD_EEPROM);
    static const u16 lss_reserved_size = 64;
    static const u16 lss_actual_size = 3;
    static const u16 test_reserved_size = 1024;
    static const s16 remaining_size =
        max_size - od_reserved_size - lss_reserved_size - test_reserved_size;

    static const u16 od_start = storage.canopen_start;
    static const u16 lss_start = od_start + od_reserved_size;
    static const u16 test_start = lss_start + remaining_size;

    /**
     * Ablage CO_OD_EEPROM
     */
    struct od {
      u32 fw_id;                  // Eindeutige ID der Firmware
      u8 data[od_actual_size];
      u32 crc;                    // CRC "uber <data>
    } od;

    /**
     * Ablage der Default Daten erm"oglicht OD restore
     */
    struct sCO_OD_EEPROM defaults;

    /**
     * Ablage LSS persistent values
     */
    struct lss {
      u8 data[lss_actual_size];
      u32 crc;                    // CRC "uber <data>
    };

    /**
     * Ablage Testsystemdaten
     */
    struct test {
      u16 size;                   // sizeof<data>
      u32 crc;                    // CRC "uber <data>
      /* u8 data[count] */
    };

  public:

    /**
     * Parametersatz laden. Falls das Laden fehlschl"agt, wird die bestehende
     * Konfig nicht ver"andert.
     *
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t load_od(void);

    /**
     * Parametersatz speichern. Ein Schreibvorgang wird nur ausgel"ost wenn
     * sich Daten ge"andert haben.
     *
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t save_od(void);

    /**
     * Parametersatz zur"ucksetzen. Dieses ver"andert die aktuell geladene
     * Konfiguration nicht (siehe CiA 301 Beschreibung Objekt 1011).
     *
     * @return
     */
    void restore_od(void);

    /**
     * LSS persistent values laden
     *
     * @param p_nid [out] Node ID
     * @param p_bit [out] Bitrate
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t load_lss(u8 *p_nid, u16 *p_bit);

    /**
     * LSS persistent values speichern
     *
     * @param nid Node ID
     * @param bit Bitrate
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t save_lss(u8 nid, u16 bit);

    /**
     * Testsystemdaten laden
     *
     * @param p_data [out] Zieldatenbereich
     * @param p_size [in] Anzahl zu lesender Bytes [out] anzahl gelesener Bytes
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t load_test(u8 *p_data, u16 *p_size);

    /**
     * Testsystemdaten speichern
     *
     * @param p_data Quelldatenbereich
     * @param size Anzahl zu schreibender Bytes
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t save_test(const u8 *p_data, u16 size);

};

#endif /* SRC_CANOPEN_CANOPEN_NVM_H_ */

/**
* @} @}
**/
