/*
 * CAN module object for Linux SocketCAN.
 *
 * @file        CO_driver.c
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


#include "CO_driver.h"
#include "CO_Emergency.h"
#include <string.h> /* for memcpy */
#include <stdlib.h> /* for malloc, free */
#include <stdio.h>  /* for printf */
#include <errno.h>
#include <sys/socket.h>


/******************************************************************************/
#ifndef CO_SINGLE_THREAD
    pthread_mutex_t CO_EMCY_mtx = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t CO_OD_mtx = PTHREAD_MUTEX_INITIALIZER;
#endif


/** Set socketCAN filters *****************************************************/
static CO_ReturnError_t setFilters(CO_CANmodule_t *CANmodule)
{
    int32_t ret;
    CO_ReturnError_t co_ret = CO_ERROR_NO;

    errno = 0;

    if(CANmodule->useCANrxFilters){

        int nFiltersIn;
        int nFiltersOut;
        int i;
        int idZeroCnt = 0;

        nFiltersIn = CANmodule->rxSize;
        nFiltersOut = 0;

        struct can_filter filtersOut[nFiltersIn];
        memset(filtersOut, 0, sizeof(struct can_filter) * nFiltersIn);

        /* Copy filterIn to filtersOut. Accept only first filter with
         * can_id=0, omit others. */
        for(i=0; i<nFiltersIn; i++){
            struct can_filter *fin;

            fin = &CANmodule->filter[i];
            if(fin->can_id == 0){
                idZeroCnt++;
            }
            if(fin->can_id != 0 || idZeroCnt == 1){
                struct can_filter *fout;

                fout = &filtersOut[nFiltersOut++];
                fout->can_id = fin->can_id;
                fout->can_mask = fin->can_mask;
            }
        }

        ret = setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                         filtersOut, sizeof(struct can_filter) * nFiltersOut);
        if(ret < 0)
        {
            printf("Linux Syscall setsockopt(FILTER) failed (%d - %s)\n", errno, strerror(errno));
            co_ret = CO_ERROR_SYSCALL;
        }
    }

    if(!CANmodule->useCANrxFilters || co_ret!=CO_ERROR_NO){
        /* fall back to software filtering */
        co_ret = CO_ERROR_NO;

        /* Use one socketCAN filter, match any CAN address, including extended and rtr. */
        struct can_filter filter;
        filter.can_id = 0;
        filter.can_mask = 0;
        ret = setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                         &filter, sizeof(struct can_filter));
        if(ret < 0)
        {
            printf("Linux Syscall setsockopt(FILTER) failed (%d - %s)\n", errno, strerror(errno));
            co_ret = CO_ERROR_SYSCALL;
        }
    }

    return co_ret;
}


/******************************************************************************/
void CO_CANsetConfigurationMode(int32_t CANbaseAddress)
{
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    /* set CAN filters */
    if(CANmodule == NULL || CANmodule->fd < 0 ||
        setFilters(CANmodule) != CO_ERROR_NO){
        CO_errExit("CO_CANsetNormalMode failed");
        return;
    }
    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        int32_t                 CANbaseAddress,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    int32_t ret;
    int32_t ovfl;
    int32_t bytes;
    uint16_t i;
    socklen_t sLen;
    can_err_mask_t err_mask;
    struct sockaddr_can sockAddr;

    errno = 0;

    /* verify arguments */
    if(CANmodule==NULL || CANbaseAddress==0 || rxArray==NULL || txArray==NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANbaseAddress = CANbaseAddress;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->fd = -1;
    CANmodule->filter = NULL;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = true;
    CANmodule->CANrxDropCount = 0;
    CANmodule->em = NULL;

#ifdef CO_LOG_CAN_MESSAGES
    CANmodule->useCANrxFilters = false;
#endif

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFFFFF;
        rxArray[i].object = NULL;
        rxArray[i].pFunct = NULL;
    }
    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = false;
    }

    if(CANmodule->useCANrxFilters) {
        /* allocate memory for filter array */
        CANmodule->filter = (struct can_filter *) calloc(rxSize, sizeof(struct can_filter));
        if(CANmodule->filter == NULL){
            return CO_ERROR_OUT_OF_MEMORY;
        }
        /* Initalize filter masks:
         * Match filter, standard 11 bit CAN address only, no rtr */
        for(i=0U; i<rxSize; i++){
            CANmodule->filter[i].can_id = 0;
            CANmodule->filter[i].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
        }
    } 
    else {
        CANmodule->filter = NULL;
        CANmodule->useCANrxFilters = false;
    }

    /* Create and bind socket */
    CANmodule->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(CANmodule->fd < 0){
        printf("Linux Syscall socket() failed (%d - %s)\n", errno, strerror(errno));
        return CO_ERROR_SYSCALL;
    }

    /* enable socket rx queue overflow detection */
    ovfl = 1;
    ret = setsockopt(CANmodule->fd, SOL_SOCKET, SO_RXQ_OVFL, &ovfl, sizeof(ovfl));
    if(ret < 0){
        printf("Linux Syscall setsockopt(OVFL) failed (%d - %s)\n",
               errno, strerror(errno));
        CO_CANmodule_disable(CANmodule);
        return CO_ERROR_SYSCALL;
    }

    //todo modify rx buffer size? first one needs root
    //ret = setsockopt(fd, SOL_SOCKET, SO_RCVBUFFORCE, (void *)&bytes, sLen);
    //ret = setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (void *)&bytes, sLen);

    /* print socket rx buffer size in bytes (In my experience, the kernel reserves
     * around 450 bytes for each CAN message) */
    sLen = sizeof(bytes);
    getsockopt(CANmodule->fd, SOL_SOCKET, SO_RCVBUF, (void *)&bytes, &sLen);
    if (sLen == sizeof(bytes)) {
        printf("socketCAN rx buffer size: %d bytes\n", bytes);
    }

    /* bind socket */
    sockAddr.can_family = AF_CAN;
    sockAddr.can_ifindex = CANbaseAddress;
    ret = bind(CANmodule->fd, (struct sockaddr*)&sockAddr, sizeof(sockAddr));
    if(ret < 0){
        printf("Linux Syscall bind() failed (%d - %s)", errno, strerror(errno));
        CO_CANmodule_disable(CANmodule);
        return CO_ERROR_SYSCALL;
    }

    /* set up error frame generation. What actually is available depends on your
     * CAN kernel driver */
#ifdef CO_LOG_CAN_MESSAGES
    err_mask = CAN_ERR_MASK; //enable ALL error frames
#else
    err_mask = CAN_ERR_ACK | CAN_ERR_CRTL | CAN_ERR_LOSTARB | CAN_ERR_BUSOFF |
               CAN_ERR_BUSERROR;
#endif
    ret = setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask,
                     sizeof(err_mask));
    if(ret < 0){
        printf("Linux Syscall setsockopt(ERR) failed (%d - %s)",
               errno, strerror(errno));
        CO_CANmodule_disable(CANmodule);
        return CO_ERROR_SYSCALL;
    }

    /* disable the reception of CAN frames for now. */
    ret = setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    if(ret < 0){
        printf("Linux Syscall setsockopt(FILTER) failed (%d - %s)",
               errno, strerror(errno));
        CO_CANmodule_disable(CANmodule);
        return CO_ERROR_SYSCALL;
    }

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule == NULL) {
      return;
    }

    if (CANmodule->fd >= 0) {
        close(CANmodule->fd);
    }
    CANmodule->fd = -1;

    if (CANmodule->filter != NULL) {
        free(CANmodule->filter);
    }
    CANmodule->filter = NULL;
}


/******************************************************************************/
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg)
{
    return (uint16_t) rxMsg->ident;
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (pFunct!=NULL) &&
       (CANmodule->filter!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->pFunct = pFunct;

        /* Configure CAN identifier and CAN mask, bit aligned with CAN module. */
        buffer->ident = ident & CAN_SFF_MASK;
        if(rtr){
            buffer->ident |= CAN_RTR_FLAG;
        }
        buffer->mask = (mask & CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){
            CANmodule->filter[index].can_id = buffer->ident;
            CANmodule->filter[index].can_mask = buffer->mask;
            if(CANmodule->CANnormal){
                ret = setFilters(CANmodule);
            }
        }
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, bit aligned with CAN module registers */
        buffer->ident = ident & CAN_SFF_MASK;
        if(rtr){
            buffer->ident |= CAN_RTR_FLAG;
        }

        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;
    ssize_t n;
    size_t count = sizeof(struct can_frame);

    if (CANmodule==NULL || CANmodule->fd < 0) {
        return CO_ERROR_PARAMETERS;
    }

    do {
        errno = 0;
        n = write(CANmodule->fd, buffer, count);
        if (errno==EINTR || errno==EAGAIN) {
            /* try again */
            continue;
        } 
        else if (errno == ENOBUFS) {
            /* socketCAN doesn't support blocking write. You can wait here for
             * a few hundred us and then try again */
            break;
        } else if (n <= 0) {
            break;
        }
    } while (errno != 0);

#ifdef CO_LOG_CAN_MESSAGES
    void CO_logMessage(const CanMsg *msg);
    CO_logMessage((const CanMsg*) buffer);
#endif

    if(n != count){
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, n);
        err = CO_ERROR_TX_OVERFLOW;
        printf("Linux Syscall write() failed (%d - %s)",
               errno, strerror(errno));
    }

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    /* Messages can not be cleared, because they are already in kernel */
}


/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule)
{
    /* socketCAN doesn't support microcontroller-like error counters. If an
     * error has occured, a special can message is created by the driver and
     * received by the application like a regular message.
     * Therefore, error counter evaluation is included in rx function.*/

#if 0
    unsigned rxErrors, txErrors;
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;
    uint32_t err;

    canGetErrorCounters(CANmodule->CANbaseAddress, &rxErrors, &txErrors);
    if(txErrors > 0xFFFF) txErrors = 0xFFFF;
    if(rxErrors > 0xFF) rxErrors = 0xFF;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | CANmodule->error;

    if(CANmodule->errOld != err){
        CANmodule->errOld = err;

        if(txErrors >= 256U){                               /* bus off */
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
        }
        else{                                               /* not bus off */
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

            if((rxErrors >= 96U) || (txErrors >= 96U)){     /* bus warning */
                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
            }

            if(rxErrors >= 128U){                           /* RX bus passive */
                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
            }
            else{
                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
            }

            if(txErrors >= 128U){                           /* TX bus passive */
                if(!CANmodule->firstCANtxMessage){
                    CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
                }
            }
            else{
                bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }

            if((rxErrors < 96U) && (txErrors < 96U)){       /* no error */
                bool_t isError = CO_isError(em, CO_EM_CAN_BUS_WARNING);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }
        }

        if(CANmodule->error & 0x02){                       /* CAN RX bus overflow */
            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
        }
    }
#endif
}

/******************************************************************************/
static CO_ReturnError_t CO_CANread(
        CO_CANmodule_t *CANmodule,
        struct can_frame *msg)
{
    int32_t n;
    uint32_t dropped;
    /* recvmsg - like read, but generates statistics about the socket
     * example in berlios candump.c */
    struct iovec iov;
    struct msghdr msghdr;
    char ctrlmsg[CMSG_SPACE(sizeof(dropped))];
    struct cmsghdr *cmsg;

    iov.iov_base = msg;
    iov.iov_len = sizeof(*msg);

    msghdr.msg_name = NULL;
    msghdr.msg_namelen = 0;
    msghdr.msg_iov = &iov;
    msghdr.msg_iovlen = 1;
    msghdr.msg_control = &ctrlmsg;
    msghdr.msg_controllen = sizeof(ctrlmsg);
    msghdr.msg_flags = 0;

    do {
        errno = 0;
        n = recvmsg(CANmodule->fd, &msghdr, 0);
        if (errno==EINTR || errno==EAGAIN) {
            /* try again */
            continue;
        } 
        else if (n <= 0) {
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_RXB_OVERFLOW,
                           CO_EMC_CAN_OVERRUN, n);
            printf("Linux Syscall recvmsg() failed (%d - %s)",
                   errno, strerror(errno));

            return CO_ERROR_SYSCALL;
        }
    } while (errno != 0);

    /* check for rx queue overflow */
    cmsg = CMSG_FIRSTHDR(&msghdr);
    if ((cmsg != NULL) && (cmsg->cmsg_level == SOL_SOCKET) &&
        (cmsg->cmsg_type == SO_RXQ_OVFL))
    {
        dropped = *(uint32_t*)CMSG_DATA(cmsg);
        if (dropped > CANmodule->CANrxDropCount) {
            printf("CAN rx queue dropped %d messages\n",
                   dropped - CANmodule->CANrxDropCount);
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_RXB_OVERFLOW,
                           CO_EMC_COMMUNICATION, 0);
        }
        CANmodule->CANrxDropCount = dropped;
    }

    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANrxWait(CO_CANmodule_t *CANmodule)
{
    CO_ReturnError_t err;
    struct can_frame msg;

    if (CANmodule==NULL || CANmodule->fd<0) {
        errno = EFAULT;
        CO_errExit("CO_CANreceive - CANmodule not configured.");
        return;
    }

    /* Read socket and pre-process message */
    err = CO_CANread(CANmodule, &msg);
    if (err != CO_ERROR_NO) {
        return;
    }

    if(CANmodule->CANnormal){

        if (msg.can_id & CAN_ERR_FLAG) {
            printf("CAN error detected. ID: 0x%0X, Data[0..7]: 0x%0X, 0x%0X, 0x%0X,"
                   "0x%0X, 0x%0X, 0x%0X, 0x%0X, 0x%0X\n", msg.can_id, msg.data[0],
                   msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5],
                   msg.data[6], msg.data[7]);
        } 
        else {
            int i;
            bool_t msgMatched;
            CO_CANrx_t *buffer; /* receive message buffer from CO_CANmodule_t object. */
            CO_CANrxMsg_t *rcvMsg;

            /* CANopenNode can message is compatible to the socketCAN one, except
             * for extension flags */
            msg.can_id &= CAN_EFF_MASK;
            rcvMsg = (CO_CANrxMsg_t *) &msg;

            /* Search rxArray from CANmodule for the matching CAN-ID. */
            buffer = &CANmodule->rxArray[0];
            msgMatched = false;
            for(i = CANmodule->rxSize; i > 0U; i--){
                if(((rcvMsg->ident ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                    break;
                }
                buffer++;
            }

            /* Call specific function, which will process the message */
            if(msgMatched && (buffer->pFunct != NULL)){
                buffer->pFunct(buffer->object, rcvMsg);
            }
        }
#ifdef CO_LOG_CAN_MESSAGES
        void CO_logMessage(const CanMsg *msg);
        CO_logMessage((CanMsg*)&rcvMsg);
#endif
    }
}
