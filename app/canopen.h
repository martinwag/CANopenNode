/**
* @addtogroup io8000 template
* @{
* @addtogroup application
* @{
* @file canopen.h
* @copyright Neuberger Gebäudeautomation GmbH
* @author mwagner
* @brief CANopenNode
*
* @details \b Programm-Name template
**/
#ifndef SRC_CANOPEN_CANOPEN_H_
#define SRC_CANOPEN_CANOPEN_H_

#include "CANopen.h"

#include "os/freertos/include/FreeRTOS.h"
#include "os/freertos/include/queue.h"
#include "os/freertos_cli/FreeRTOS_CLI.h"

#include "interface/nbtyp.h"
#include "drivers/can.h"

#include "canopen_storage.h"
#include "canopen_errors.h"


/**
 * Die CANopen Klasse
 */
class Canopen: public Canopen_errors {
  private:
    CO_NMT_reset_cmd_t reset;         /*!< Resetanforderung */
    u8 *const p_active_nid = &OD_CANNodeID; /*!< Eigene CANopen Node ID. Zeigt auf Eintrag im OD. */
    u16 active_bit = 1000;            /*!< Standard Bitrate */
    class Canopen_storage storage;    /*!< OD Parameter */
    static const u8 main_interval = 50; /*!< ms, max. Wartezeit auf Events in process() */
    u32 worker_interval;              /*!< CO Thread Intervall */
    static QueueHandle_t nmt_event_queue; /*!< per <nmt_event()> eingetragene Queue */
    bool once;                        /*!< Flag Erststart */
    /* PDO in Anwendung f"ur PSE Stack
     * Einfachste Implementierung, solange nur eine Instanz notwendig ist.
     * Falls mehrere notwendig sein sollten, Ablage z.B. in library-asotab */
    CO_TPDO_t *p_tpdo = nullptr;
    TickType_t tpdo_called = 0;
    void (*p_rpdo)(u16 id, const u8* p_data, u8 count);

    /*1010*/CO_SDO_abortCode_t store_parameters_callback(CO_ODF_arg_t *p_odf_arg);
    /*1011*/CO_SDO_abortCode_t restore_default_parameters_callback(CO_ODF_arg_t *p_odf_arg);
    /*1012*/CO_SDO_abortCode_t cob_id_timestamp_callback(CO_ODF_arg_t *p_odf_arg);
    /*1f51*/CO_SDO_abortCode_t program_control_callback(CO_ODF_arg_t *p_odf_arg);
    /*2108*/CO_SDO_abortCode_t temperature_callback(CO_ODF_arg_t *p_odf_arg);
    /*2109*/CO_SDO_abortCode_t voltage_callback(CO_ODF_arg_t *p_odf_arg);
    /*2110*/CO_SDO_abortCode_t can_runtime_info_callback(CO_ODF_arg_t *p_odf_arg);
    /*2112*/CO_SDO_abortCode_t daisychain_callback(CO_ODF_arg_t *p_odf_arg);
    /*5000*/CO_SDO_abortCode_t serial_number_callback(CO_ODF_arg_t *p_odf_arg);

    /* Init Helper */
    void od_load_start(void);
    void od_set_defaults(void);
    void lss_check(u8 *p_pending_nid);
    CO_ReturnError_t co_init(u8 pending_nid);
    void lss_nid_assignment(u8 *p_pending_nid);
    CO_ReturnError_t co_start(u8 pending_nid, u32 interval);

    /* Diese Callbacks m"ussen Klassenmethoden sein, da der Stack Callback
     * keinen Pointer f"ur die Instanz zur Verf"ugung stellt. Diese sind daher
     * so aufgebaut das keine Info "uber die Instanz notwendig ist. */
    static void nmt_state_callback(CO_NMT_internalState_t state);
    static CO_SDO_abortCode_t generic_write_callback(CO_ODF_arg_t *p_odf_arg);

    void set_callback(u16 obj_dict_id, CO_SDO_abortCode_t (*pODFunc)(CO_ODF_arg_t *ODF_arg));

    void *get_od_pointer(u16 index, u8 subindex, size_t size);

    void daisychain_event_callback(void);
    bool store_lss_config_callback(uint8_t nid, uint16_t bitRate);
    void rpdo_callback(const CO_RPDO_t *rpdo, const CO_CANrxMsg_t *message);

    volatile bool timer_rx_suspend;
    TaskHandle_t timer_rx_handle;
    void timer_rx_thread();

  public:

    /**
     * @defgroup OD Zugriffsfunktionen f"ur Objektverzeichnis
     * @{
     */

    /**
     * Ermöglicht synchronen Zugriff auf mehrere OD Einträge
     */
    void od_lock(void);

    /**
     * Synchronen Zugriff abschließen
     */
    void od_unlock(void);

    /**
     * Zugriff auf Einträge im Objektverzeichnis
     *
     * Das OD muss mit <od_lock()> gesperrt sein
     *
     * @param index OD Index (z.B. aus CO_OD.h)
     * @param subindex OD Subindex (z.B. aus CO_OD.h)
     * @param [out] p_retval Im OD hinterlegter Wert
     */
    void od_get(u16 index, u8 subindex, u8 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, u16 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, u32 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, u64 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, s8 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, s16 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, s32 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, s64 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, f32 *p_retval);
    /** \overload */
    void od_get(u16 index, u8 subindex, const char **pp_visible_string);
    // weitere CO Standardtypen

    /**
     * Ändern von Einträgen im Objektverzeichnis
     *
     * Das OD muss mit <od_lock()> gesperrt sein
     *
     * @param index OD Index (z.B. aus CO_OD.h)
     * @param subindex OD Subindex (z.B. aus CO_OD.h)
     * @param val Zu übernehmender Wert
     */
    void od_set(u16 index, u8 subindex, u8 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, u16 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, u32 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, u64 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, s8 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, s16 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, s32 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, s64 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, f32 val);
    /** \overload */
    void od_set(u16 index, u8 subindex, const char *p_visible_string);
    // weitere CO Standardtypen

    /**
     * Eintragen einer Event Queue
     *
     * Das Event wird bei Schreibzugriff auf den per <index> eingetragenen
     * OD Eintrag ausgel"ost. Jedes Event beinhaltet die Information "uber
     * die Eventquelle. Der zugeh"orige Wert kann mit <od_get()> ausgelesen
     * werden.
     *
     * Zum Eintragen muss das OD mit <od_lock()> gesperrt sein
     *
     * @param index OD Index (z.B. aus CO_OD.h)
     * @param p_event_queue Queue auf der das Event abelegt werden soll. Auf
     * die Queue wird non-blocking geschrieben, d.H. Events werden verworfen
     * wenn kein Platz mehr frei ist!
     */
    void od_event(u16 index, QueueHandle_t event_queue);

    /**
     * Callback Event Nachricht
     */
    typedef struct {
      u16 index;
      u8 subindex;
    } od_event_t;

    /** @}*/

    /**
     * @defgroup PDO Zugriffsfunktionen f"ur manuelles PDO Handling
     * @{
     */

    /**
     * Manuelle TPDO Steuerung aktivieren
     *
     * @param tpdo_com_param_index Eintrag des zugeh"origen TPDO communication parameter
     * @return CO_ERROR_NO wenn erfolgreich
     */
    CO_ReturnError_t tpdo_take_control(u16 tpdo_com_param_index);

    /**
     * Manuelle TPDO Steuerung deaktivieren
     *
     * @param id ID wie in #tpdo_take_control()
     */
    void tpdo_release_control(u16 id);

    /**
     * TPDO versenden
     *
     * @remark Inhibit Time und Event Timer werden ignoriert!
     *
     * @param id ID von #tpdo_take_control()
     * @return CO_ERROR_NO wenn kein Fehler aufgetreten ist
     */
    CO_ReturnError_t tpdo_send(u16 id);

    /**
     * Manuelle RPDO Steuerung aktivieren
     *
     * @remark Auf die in diesen PDO gemappten OD Eintr"age kann nicht mehr
     * per SDO zugegriffen werden!
     *
     * @remark Innerhalb der Callbackfunktion ist das OD bereits per #od_lock()
     * gesichert.
     *
     * @param tpdo_com_param_index Eintrag des zugeh"origen RPDO communication parameter
     * @param p Callback RPDO empfangen
     * @return CO_ERROR_NO wenn kein Fehler aufgetreten ist
     */
    CO_ReturnError_t rpdo_take_control(u16 rpdo_com_param_index,
        void (*p)(u16 id, const u8* p_data, u8 count));

    /**
     * Manuelle RPDO Steuerung deaktivieren
     *
     * @param id ID von #rpdo_take_control()
     */
    void rpdo_release_control(u16 id);

    /** @}*/

    /**
     * @defgroup EMCY Zugriffsfunktionen auf CANopen Emergency Funktionen.
     * @{
     */

    /**
     * Pr"ufen ob Fehler aktiv ist
     *
     * @param error Fehlercode
     * @return true wenn aktiv
     */
    bool error_get(errorcode_t error);

    /**
     * Fehler aktiv setzen
     *
     * Ein bereits aktiver Fehler kann ohne Auswirkung erneut aktiv gesetzt
     * werden.
     *
     * Falls der Fehler auf eine CO Standard Emergency gemapped werden kann,
     * so wird diese eingetragen. Standardm"a"sig wird die Emergency 0xFFxx
     * "DEVICE_SPECIFIC" ausgel"ost.
     *
     * @param error Fehlercode
     * @param detail Dieser Wert wird an die Fehlernachricht angeh"angt. Er
     * wird nicht gespeichert und kann nicht abgefragt werden!
     */
    void error_set(errorcode_t error, u32 detail);

    /**
     * Fehler zur"ucksetzen
     *
     * Ein bereits zur"uckgesetzter Fehler kann ohne Auswirkung erneut zur"uck
     * gesetzt werden.
     *
     * @param error Fehlercode
     * @param detail siehe <error_set()>
     */
    void error_reset(errorcode_t error, u32 detail);

    /** @}*/

    /**
     * @defgroup NMT Zugriffsfunktionen auf Netzwerkmanagement
     * @{
     */

    /**
     * Eintragen einer Event Queue
     *
     * Das Event wird bei "Anderung des NMT Zustands ausgel"ost. Jedes Event
     * beinhaltet die Information "uber den neuen Zustand.
     *
     * @remark Mit der aktuellen Implementierung kann nur ein Event Consumer
     * registriert werden.
     *
     * @param p_event_queue Queue auf der das Event abelegt werden soll. Auf
     * die Queue wird non-blocking geschrieben, d.H. Events werden verworfen
     * wenn kein Platz mehr frei ist!
     */
    void nmt_event(QueueHandle_t event_queue);

    /**
     * M"ogliche NMT Events
     */
    typedef enum {
      INITIALIZING = CO_NMT_INITIALIZING,      //!< Device is initializing */
      PRE_OPERATIONAL = CO_NMT_PRE_OPERATIONAL,//!< Device is in pre-operational state */
      OPERATIONAL = CO_NMT_OPERATIONAL,        //!< Device is in operational state */
      STOPPED = CO_NMT_STOPPED,                //!< Device is stopped */
       //todo NMT Heartbeat Consumer Timeout
    } nmt_event_t;

    /** @}*/

    /**
     * CANopen Stack initialisieren
     *
     * @remark Falls keine Node ID vorgegeben ist, wird diese per LSS bestimmt.
     * Dieser Vorgang wartet bis eine g"ultige Adresse (1..127) gesetzt wurde.
     * Hierf"ur wird eine Watchdog ID ben"otigt um den WDT korrekt zu triggern.
     *
     * @param nid CANopen Node ID. 0 = Node ID per LSS bestimmen.
     * @param interval Abarbeitungsintervall f"ur zeitkritische CANopen Komponenten
     * @return CO_ERROR_NO wenn erfolgreich
     */
    CO_ReturnError_t init(u8 nid, u32 interval);

    /**
     * CANopen Stack deinitialisieren
     */
    void deinit(void);

    /**
     * Nicht zeitkritische CANopen Abarbeitung
     *
     * Blockiert bis zu 50 ms. Abh. von der internen Verarbeitung kann diese
     * Zeit auch k"urzer sein.
     */
    void process();

    /**
     * CANopen Terminalbefehl
     */
    BaseType_t cmd_terminal( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

    /**
     * @defgroup Wrapper f"ur "C" Callbacks
     * @{
     */
    static void timer_rx_thread_wrapper(void *p);
    static void daisychain_event_callback_wrapper(void *p_object);
    static bool_t store_lss_config_callback_wrapper(void *p_object, uint8_t nid, uint16_t bit_rate);
    static void rpdo_callback_wrapper(void *p_object, const CO_RPDO_t *rpdo, const CO_CANrxMsg_t *message);
    static CO_SDO_abortCode_t store_parameters_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    static CO_SDO_abortCode_t restore_default_parameters_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    static CO_SDO_abortCode_t cob_id_timestamp_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    static CO_SDO_abortCode_t program_control_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    static CO_SDO_abortCode_t temperature_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    static CO_SDO_abortCode_t voltage_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    static CO_SDO_abortCode_t can_runtime_info_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    static CO_SDO_abortCode_t daisychain_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    static CO_SDO_abortCode_t serial_number_callback_wrapper(CO_ODF_arg_t *p_odf_arg);
    /** @} */

};

#ifndef MOCK_CANOPEN
  extern class Canopen canopen;
#endif

#endif /* SRC_CANOPEN_CANOPEN_H_ */

/**
* @} @}
**/
