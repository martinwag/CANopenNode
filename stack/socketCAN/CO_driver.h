/*
 * CAN module object for Linux SocketCAN.
 *
 * @file        CO_driver.h
 * @author      Janez Paternoster
 * @copyright   2015 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free and open source software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef CO_DRIVER_H
#define CO_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* For documentation see file drvTemplate/CO_driver.h */


#include <stddef.h>         /* for 'NULL' */
#include <stdint.h>         /* for 'int8_t' to 'uint64_t' */
#include <stdbool.h>        /* for 'true', 'false' */
#include <unistd.h>
#include <endian.h>

#ifndef CO_SINGLE_THREAD
#include <pthread.h>
#endif

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>


/* general configuration */
//    #define CO_LOG_CAN_MESSAGES   /* Call external function for each received or transmitted CAN message. */
    #define CO_SDO_BUFFER_SIZE           889    /* Override default SDO buffer size. */


/** Critical sections */
#ifdef CO_SINGLE_THREAD
    #define CO_LOCK_CAN_SEND()
    #define CO_UNLOCK_CAN_SEND()

    #define CO_LOCK_EMCY()
    #define CO_UNLOCK_EMCY()

    #define CO_LOCK_OD()
    #define CO_UNLOCK_OD()

    #define CANrxMemoryBarrier()
#else
    #define CO_LOCK_CAN_SEND()      /* not needed */
    #define CO_UNLOCK_CAN_SEND()

    extern pthread_mutex_t CO_EMCY_mtx;
    #define CO_LOCK_EMCY()          {if(pthread_mutex_lock(&CO_EMCY_mtx) != 0) CO_errExit("Mutex lock CO_EMCY_mtx failed");}
    #define CO_UNLOCK_EMCY()        {if(pthread_mutex_unlock(&CO_EMCY_mtx) != 0) CO_errExit("Mutex unlock CO_EMCY_mtx failed");}

    extern pthread_mutex_t CO_OD_mtx;
    #define CO_LOCK_OD()            {if(pthread_mutex_lock(&CO_OD_mtx) != 0) CO_errExit("Mutex lock CO_OD_mtx failed");}
    #define CO_UNLOCK_OD()          {if(pthread_mutex_unlock(&CO_OD_mtx) != 0) CO_errExit("Mutex unlock CO_OD_mtx failed");}

    #define CANrxMemoryBarrier()    {__sync_synchronize();}
#endif

/** Syncronisation functions */
#define IS_CANrxNew(rxNew) ((long)rxNew)
#define SET_CANrxNew(rxNew) {CANrxMemoryBarrier(); rxNew = (void*)1L;}
#define CLEAR_CANrxNew(rxNew) {CANrxMemoryBarrier(); rxNew = (void*)0L;}


/**
 * @defgroup CO_dataTypes Data types
 * @{
 *
 * According to Misra C
 */
    /* int8_t to uint64_t are defined in stdint.h */
    typedef _Bool                   bool_t;     /**< bool_t */
    typedef float                   float32_t;  /**< float32_t */
    typedef double                  float64_t;  /**< float64_t */
    typedef char                    char_t;     /**< char_t */
    typedef unsigned char           oChar_t;    /**< oChar_t */
    typedef unsigned char           domain_t;   /**< domain_t */
/** @} */

/**
 * Return values of some CANopen functions. If function was executed
 * successfully it returns 0 otherwise it returns <0.
 */
typedef enum{
    CO_ERROR_NO                 = 0,
    CO_ERROR_ILLEGAL_ARGUMENT   = -1,
    CO_ERROR_OUT_OF_MEMORY      = -2,
    CO_ERROR_TIMEOUT            = -3,
    CO_ERROR_ILLEGAL_BAUDRATE   = -4,
    CO_ERROR_RX_OVERFLOW        = -5,
    CO_ERROR_RX_PDO_OVERFLOW    = -6,
    CO_ERROR_RX_MSG_LENGTH      = -7,
    CO_ERROR_RX_PDO_LENGTH      = -8,
    CO_ERROR_TX_OVERFLOW        = -9,
    CO_ERROR_TX_PDO_WINDOW      = -10,
    CO_ERROR_TX_UNCONFIGURED    = -11,
    CO_ERROR_PARAMETERS         = -12,
    CO_ERROR_DATA_CORRUPT       = -13,
    CO_ERROR_CRC                = -14,
    CO_ERROR_SYSCALL            = -15
}CO_ReturnError_t;


/**
 * CAN receive message structure as aligned in socketCAN module.
 */
typedef struct{
    uint32_t        ident;
    uint8_t         DLC;
    uint8_t         data[8] __attribute__((aligned(8)));
}CO_CANrxMsg_t;


/**
 * Received message object
 */
typedef struct{
    uint32_t            ident;
    uint32_t            mask;
    void               *object;
    void              (*pFunct)(void *object, const CO_CANrxMsg_t *message);
}CO_CANrx_t;


/**
 * Transmit message object.
 */
typedef struct{
    uint32_t            ident;          /**< CAN identifier as aligned in CAN module */
    uint8_t             DLC;           /**< Length of CAN message. (DLC may also be part of ident) */
    uint8_t             data[8] __attribute__((aligned(8))); /**< 8 data bytes */
    volatile bool_t     bufferFull;     /**< True if previous message is still in buffer */
    /** Synchronous PDO messages has this flag set. It prevents them to be sent outside the synchronous window */
    volatile bool_t     syncFlag;
}CO_CANtx_t;


/**
 * CAN module object.
 */
typedef struct{
    int32_t             CANbaseAddress;
#ifdef CO_LOG_CAN_MESSAGES
    CO_CANtx_t          txRecord;
#endif
    CO_CANrx_t         *rxArray;
    uint16_t            rxSize;
    CO_CANtx_t         *txArray;
    uint16_t            txSize;
    int                 fd;         /* CAN_RAW socket file descriptor */
    struct can_filter  *filter;     /* array of CAN filters of size rxSize */
    volatile bool_t     CANnormal;
    volatile bool_t     useCANrxFilters;
    uint32_t            CANrxDropCount; /* messages dropped on rx socket queue */
    void               *em;
}CO_CANmodule_t;


/* Endianes */
#ifdef BYTE_ORDER
#if BYTE_ORDER == LITTLE_ENDIAN
    #define CO_LITTLE_ENDIAN
#else
    #define CO_BIG_ENDIAN
#endif
#endif


/* Helper function, must be defined externally. */
static inline void CO_errExit(char* msg) {}; //todo


/**
 * Request CAN configuration (stopped) mode. Not supported in this driver
 */
void CO_CANsetConfigurationMode(int32_t CANbaseAddress);

/**
 * Request CAN normal (opearational) mode and *wait* untill it is set.
 *
 * @param CANmodule This object.
 */
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule);

/**
 * Initialize CAN module object and open socketCAN connection.
 *
 * Function must be called in the communication reset section. CAN module must
 * be in Configuration Mode before.
 *
 * @param CANmodule This object will be initialized.
 * @param CANbaseAddress CAN module base address.
 * @param rxArray Array for handling received CAN messages
 * @param rxSize Size of the above array. Must be equal to number of receiving CAN objects.
 * @param txArray Array for handling transmitting CAN messages
 * @param txSize Size of the above array. Must be equal to number of transmitting CAN objects.
 * @param CANbitRate not supported in this driver. Needs to be set by OS
 *
 * Return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT or
 * CO_ERROR_SYSCALL.
 */
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        int32_t                 CANbaseAddress,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate);


/**
 * Close socketCAN connection. Call at program exit.
 *
 * @param CANmodule CAN module object.
 */
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule);


/**
 * Read CAN identifier from received message
 *
 * @param rxMsg Pointer to received message
 * @return 11-bit CAN standard identifier.
 */
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg);


/**
 * Configure CAN message receive buffer.
 *
 * Function configures specific CAN receive buffer. It sets CAN identifier
 * and connects buffer with specific object. Function must be called for each
 * member in _rxArray_ from CO_CANmodule_t.
 *
 * @param CANmodule This object.
 * @param index Index of the specific buffer in _rxArray_.
 * @param ident 11-bit standard CAN Identifier.
 * @param mask 11-bit mask for identifier. Most usually set to 0x7FF.
 * Received message (rcvMsg) will be accepted if the following
 * condition is true: (((rcvMsgId ^ ident) & mask) == 0).
 * @param rtr If true, 'Remote Transmit Request' messages will be accepted.
 * @param object CANopen object, to which buffer is connected. It will be used as
 * an argument to pFunct. Its type is (void), pFunct will change its
 * type back to the correct object type.
 * @param pFunct Pointer to function, which will be called, if received CAN
 * message matches the identifier. It must be fast function.
 *
 * Return #CO_ReturnError_t: CO_ERROR_NO CO_ERROR_ILLEGAL_ARGUMENT or
 * CO_ERROR_OUT_OF_MEMORY (not enough masks for configuration).
 */
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message));


/**
 * Configure CAN message transmit buffer.
 *
 * Function configures specific CAN transmit buffer. Function must be called for
 * each member in _txArray_ from CO_CANmodule_t.
 *
 * @param CANmodule This object.
 * @param index Index of the specific buffer in _txArray_.
 * @param ident 11-bit standard CAN Identifier.
 * @param rtr If true, 'Remote Transmit Request' messages will be transmitted.
 * @param noOfBytes Length of CAN message in bytes (0 to 8 bytes).
 * @param syncFlag This flag bit is used for synchronous TPDO messages. If it is set,
 * message will not be sent, if curent time is outside synchronous window.
 *
 * @return Pointer to CAN transmit message buffer. 8 bytes data array inside
 * buffer should be written, before CO_CANsend() function is called.
 * Zero is returned in case of wrong arguments.
 */
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag);


/**
 * Send CAN message.
 *
 * @param CANmodule This object.
 * @param buffer Pointer to transmit buffer, returned by CO_CANtxBufferInit().
 * Data bytes must be written in buffer before function call.
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_TX_OVERFLOW or
 * CO_ERROR_TX_PDO_WINDOW (Synchronous TPDO is outside window).
 */
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer);


/**
 * Clear all synchronous TPDOs from CAN module transmit buffers.
 * This function is not supported in this driver.
 */
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule);

/**
 * Verify all errors of CAN module.
 * This function is not supported in this driver. Error checking is done
 * inside <CO_CANrxWait()>.
 */
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule);


/**
 * Functions receives CAN messages. It is blocking.
 *
 * The received messages are evaluated inside this function and the corresponding
 * callback or error handler is called.
 *
 * @param CANmodule This object.
 */
void CO_CANrxWait(CO_CANmodule_t *CANmodule);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif
