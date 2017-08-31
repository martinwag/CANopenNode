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

#include "FreeRTOS.h"
#include "queue.h"

/**
 * Ablage eines CANopen Speicherbereichs
 */
class Canopen_storage_type {
  protected:
    /**
     * Parametersatz laden. Falls das Laden fehlschl"agt, wird der Zieldaten-
     * bereich nicht ver"andert.
     *
     * @param start Startadresse im EEPROM
     * @param reserved F"ur diesen Bereich reservierter Speicher in Bytes
     * @param size L"ange des Nutzdatenbereichs in Bytes
     * @param p_work <reserved> Bytes f"ur tempor"are Daten. Mind. <size> + 4
     * @param p_to <size> Bytes f"ur gelesene Daten
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t load(u16 start, u16 reserved, u16 size, u8 *p_work, u8 *p_to);

    /**
     * Parametersatz speichern. Ein Schreibvorgang wird nur ausgel"ost wenn
     * sich Daten ge"andert haben.
     *
     * @param start Startadresse im EEPROM
     * @param reserved F"ur diesen Bereich reservierter Speicher in Bytes
     * @param size L"ange des Nutzdatenbereichs in Bytes
     * @param p_work <reserved> Bytes f"ur tempor"are Daten. Mind. <size> + 4
     * @param p_from <size> Bytes f"ur gelesene Daten
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t save(u16 start, u16 reserved, u16 size, u8 *p_work, const u8 *p_from);

    /**
     * Speicher l"oschen.
     *
     * @param start Startadresse im EEPROM
     * @param size L"ange des Datenbereichs in Bytes
     */
    void erase(u16 start, u16 size);
};

/**
 * Ablage der CO Speicherbereiche im EEPROM
 *
 * wir orientieren uns an der Vorlage eeprom.c/h aus dem Stack Treiberbeispiel
 */
class Canopen_storage : Canopen_storage_type {
  public:
    typedef enum {
      COMMUNICATION = 0,
      PARAMS,
      RUNTIME,
      SERIAL,
      TEST,
      CALIB,

      TYPE_COUNT
    } storage_type_t;

  protected:
    const u16 max_size = storage.canopen_size;

    /*
     * F"ur die einzelnen Bereiche reservierter Speicher
     */
    const u16 reserved_size[TYPE_COUNT] = {
      /* com */     32,
      /* params */  2048,
      /* runtime */ 128,
      /* serial */  64,
      /* test */    256,
      /* calib */   1024
    };

    /*
     * Von den einzelnen Bereichen tats"achlich belegter Speicher (ohne Verwaltungsinfos)
     */
    const u16 actual_size[TYPE_COUNT] = {
      /* com */     sizeof(struct sCO_OD_COMMUNICATION),
      /* params */  sizeof(struct sCO_OD_EEPROM),
      /* runtime */ sizeof(struct sCO_OD_RUNTIME),
      /* serial */  sizeof(struct sCO_OD_SERIAL),
      /* test */    sizeof(struct sCO_OD_TEST),
      /* calib */   sizeof(struct sCO_OD_CALIBRATION)
    };

    /*
     * Zeiger auf Bereiche im RAM
     */
    u8 *const p_ram[TYPE_COUNT] = {
        /* com */     reinterpret_cast<u8*>(&CO_OD_COMMUNICATION),
        /* params */  reinterpret_cast<u8*>(&CO_OD_EEPROM),
        /* runtime */ reinterpret_cast<u8*>(&CO_OD_RUNTIME),
        /* serial */  reinterpret_cast<u8*>(&CO_OD_SERIAL),
        /* test */    reinterpret_cast<u8*>(&CO_OD_TEST),
        /* calib */   reinterpret_cast<u8*>(&CO_OD_CALIBRATION)
    };

    /*
     * Nach Platzierung aller reservierten Speicherbereiche verbleibender Speicher
     */
    const s32 remaining_size = max_size - reserved_size[COMMUNICATION] -
        reserved_size[PARAMS] - reserved_size[RUNTIME] - reserved_size[SERIAL] -
        reserved_size[TEST] - reserved_size[CALIB];

    /*
     * Anordnung im Speicher. Daten die "uber FW Update hinweg erhalten bleiben
     * m"ussen von vorne beginnend, Rest von hinten beginnend angeordnet sein.
     */
    const u16 serial_start = storage.canopen_start;
    const u16 test_start = serial_start + reserved_size[SERIAL];
    const u16 calib_start = test_start + reserved_size[TEST];
    const u16 runtime_start = calib_start + reserved_size[CALIB];
    const u16 remaining_start = runtime_start + reserved_size[RUNTIME];
    const u16 com_start = remaining_start + remaining_size;
    const u16 params_start = com_start + reserved_size[COMMUNICATION];
    /*
     * Startadressen im Speicherbaustein
     */
    const u16 start[TYPE_COUNT] = {
      /* com */     com_start,
      /* params */  params_start,
      /* runtime */ runtime_start,
      /* serial */  serial_start,
      /* test */    test_start,
      /* calib */   calib_start
    };

    /* Puffer f"ur interne Verarbeitung */
    union work {
      struct sCO_OD_COMMUNICATION com;
      struct sCO_OD_EEPROM params;
      struct sCO_OD_RUNTIME runtime;
      struct sCO_OD_SERIAL serial;
      struct sCO_OD_TEST test;
      struct sCO_OD_CALIBRATION calib;
    };
    u8 work[sizeof(union work) + sizeof(u32)];
    QueueHandle_t in_use = NULL;
    void lock(void);
    void unlock(void);

  public:

    /**
     * Parametersatz laden. Falls das Laden fehlschl"agt, wird die bestehende
     * Konfig nicht ver"andert.
     *
     * @param type Zu bearbeitender Speicherbereich
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t load(storage_type_t type);

    /**
     * Parametersatz speichern. Ein Schreibvorgang wird nur ausgel"ost wenn
     * sich Daten ge"andert haben.
     *
     * @param type Zu bearbeitender Speicherbereich
     * @return CO_ERROR_NO wenn OK
     */
    CO_ReturnError_t save(storage_type_t type);

    /**
     * Parametersatz zur"ucksetzen. Dieses ver"andert die aktuell geladene
     * Konfiguration nicht (siehe CiA 301 Beschreibung Objekt 1011).
     *
     * @param type Zu bearbeitender Speicherbereich
     */
    void restore(storage_type_t type);
};


#endif /* SRC_CANOPEN_CANOPEN_NVM_H_ */

/**
* @} @}
**/
