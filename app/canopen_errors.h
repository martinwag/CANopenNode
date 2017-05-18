/**
* @addtogroup io8000 template
* @{
* @addtogroup application
* @{
* @file canopen_errors.h
* @copyright Neuberger Gebäudeautomation GmbH
* @author mwagner
* @brief CANopen weitere Fehlercodes
*
* @details \b Programm-Name template
**/
#ifndef SRC_CANOPEN_CANOPEN_ERRORS_H_
#define SRC_CANOPEN_CANOPEN_ERRORS_H_

#include "CANopen.h"

/**
 * Diese Klasse beinhaltet alle CANopen Modulspezifischen Errorcodes.
 * Sie erweitert die Emergency Liste "CO_EM_errorStatusBits Error status bits"
 * aus CANopenNode.
 * F"ur jeden Fehler wird ein Bit im OD Eintrag "Error Status Bits" reserviert.
 */
class canopen_errors {
  public:

    /**
     * Neuberger modul"ubergreifende Fehlercodes.
     *
     * Falls ein neuer hinzugef"ugt werden soll, so muss dieser auch im Template
     * eingepflegt werden.
     * @todo das ist sch...
     */
    typedef enum {
      START = CO_EM_MANUFACTURER_START,       //!< Erster verf"ugbarer Wert

      /* Standard DSP401 Codes */
      OUT_CUR_HIGH = START,                   //!< Current at outputs too high (overload)
      OUT_SHORTED,                            //!< Short circuit at outputs
      OUT_LOAD_DUMP,                          //!< Load dump at outputs
      IN_VOLT_HI,                             //!< Input voltage too high
      IN_VOLT_LOW,                            //!< Input voltage too low
      INTERN_VOLT_HI,                         //!< Internal voltage too high
      INTERN_VOLT_LO,                         //!< Internal voltage too low
      OUT_VOLT_HIGH,                          //!< Output voltage too high
      OUT_VOLT_LOW,                           //!< Output voltage too low
      /* Weitere Codes */
      INIT_MISMATCH_MODTYPE,                  //!< Firmware nicht lauff"ahig auf dieser HW
      INIT_MISMATCH_HW_REV,                   //!< Firmware passt nicht zur HW Revision
      INIT_INTERNAL,                          //!< anderer Initialisierungsfehler
      HARDWARE,                               //!< Externe Hardware
      INTERNAL,                               //!< Software/Controllerperipherie

      END = ODL_errorStatusBits_stringLength * 8 - 1 //!< Letzter verf"ugbarer Wert
    } errorcode_t;
};



#endif /* SRC_CANOPEN_CANOPEN_ERRORS_H_ */

/**
* @} @}
**/
