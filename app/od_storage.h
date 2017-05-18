/**
* @addtogroup io8000 template
* @{
* @addtogroup application
* @{
* @file od_storage.h
* @copyright Neuberger Gebäudeautomation GmbH
* @author mwagner
* @brief Ablage der Parameter im Festwertspeicher
*
* @details \b Programm-Name template
* @details Dieses Modul verwaltet die Ablage der CANopen
* Parameter im Festwertspeicher
**/
#ifndef SRC_CANOPEN_OD_STORAGE_H_
#define SRC_CANOPEN_OD_STORAGE_H_

#include "CANopen.h"

#include "globdef.h"

/**
 * Ablage der CO Parameter im EEPROM. Die Funktionalit"at k"onnten wir
 * erben, eine zweite Instanz der HW ist aber nicht m"oglich...
 *
 * wir orientieren uns an der Vorlage eeprom.c/h aus dem Stack Treiberbeispiel
 */
class od_storage {
  protected:
    static const u16 od_max_size = storage.canopen_size;
    static const u16 od_actual_size = sizeof(struct sCO_OD_EEPROM);

    struct block {
      u32 fw_id;                  // Eindeutige ID der Firmware
      u8 data[od_actual_size];
      u32 crc;                    // CRC "uber <data>
    } block;

    struct sCO_OD_EEPROM defaults;

  public:

    /**
     * Parametersatz laden. Falls das Laden fehlschl"agt, wird die bestehende
     * Konfig nicht ver"andert.
     *
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t load(void);

    /**
     * Parametersatz speichern. Ein Schreibvorgang wird nur ausgel"ost wenn
     * sich Daten ge"andert haben.
     *
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t save(void);

    /**
     * Parametersatz zur"ucksetzen. Dieses ver"andert die aktuell geladene
     * Konfiguration nicht (siehe CiA 301 Beschreibung Objekt 1011).
     *
     * @return
     */
    void restore(void);
};

#endif /* SRC_CANOPEN_OD_STORAGE_H_ */

/**
* @} @}
**/
