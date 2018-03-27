/*
 * CAN module object for Linux socketCAN.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster, Martin Wagner
 * @copyright   2004 - 2015 Janez Paternoster, 2017 Neuberger Gebaeudeautomation GmbH
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
 *
 * Following clarification and special exception to the GNU General Public
 * License is included to the distribution terms of CANopenNode:
 *
 * Linking this library statically or dynamically with other modules is
 * making a combined work based on this library. Thus, the terms and
 * conditions of the GNU General Public License cover the whole combination.
 *
 * As a special exception, the copyright holders of this library give
 * you permission to link this library with independent modules to
 * produce an executable, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting
 * executable under terms of your choice, provided that you also meet,
 * for each linked independent module, the terms and conditions of the
 * license of that module. An independent module is a module which is
 * not derived from or based on this library. If you modify this
 * library, you may extend this exception to your version of the
 * library, but you are not obliged to do so. If you do not wish
 * to do so, delete this exception statement from your version.
 */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <sys/socket.h>

#include "CO_driver.h"

#if __has_include("CO_Emergency.h")
  #include "CO_Emergency.h"
  #define USE_EMERGENCY_OBJECT
#endif

/** Disable socketCAN rx *****************************************************/
static CO_ReturnError_t disableRx(CO_CANmodule_t *CANmodule)
{
  int ret;

  /* insert a filter that doesn't match any messages */
  ret = setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
  if(ret < 0){
      return CO_ERROR_SYSCALL;
  }
  return CO_ERROR_NO;
}

/** Set up or update socketCAN rx filters *************************************/
static CO_ReturnError_t setRxFilters(CO_CANmodule_t *CANmodule)
{
    int ret;
    int i;
    int count;

    struct can_filter rxFiltersCpy[CANmodule->rxSize];

    count = 0;
    /* remove unused entries ( id == 0 and mask == 0 ) as they would act as
     * "pass all" filter */
    for (i = 0; i < CANmodule->rxSize; i ++) {
        if ((CANmodule->rxFilter[i].can_id != 0) ||
            (CANmodule->rxFilter[i].can_mask != 0)) {

            rxFiltersCpy[count] = CANmodule->rxFilter[i];

            count ++;
        }
    }

    if (count == 0) {
        /* No filter is set, disable RX */
        return disableRx(CANmodule);
    }

    ret = setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER, rxFiltersCpy,
                     sizeof(struct can_filter) * count);
    if(ret < 0){
        return CO_ERROR_SYSCALL;
    }
    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANsetConfigurationMode(int32_t CANbaseAddress)
{
    /* Can't do anything because no object is provided */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    CO_ReturnError_t ret;

    CANmodule->CANnormal = false;

    if(CANmodule != NULL && CANmodule->fd >= 0) {
        ret = setRxFilters(CANmodule);
        if (ret == CO_ERROR_NO) {
            /* Put CAN module in normal mode */
            CANmodule->CANnormal = true;
        }
    }
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

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANbaseAddress = CANbaseAddress;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANnormal = false;
    CANmodule->fd = -1;
    CANmodule->rxFilter = NULL;
    CANmodule->em = NULL; //this is set inside CO_Emergency.c init function!

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].pFunct = NULL;
    }

    /* initialize socketCAN filters
     * CAN module filters will be configured with CO_CANrxBufferInit()
     * functions, called by separate CANopen init functions */
    CANmodule->rxFilter = calloc(rxSize, sizeof(struct can_filter));
    if(CANmodule->rxFilter == NULL){
        return CO_ERROR_OUT_OF_MEMORY;
    }

    /* Create socket */
    CANmodule->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(CANmodule->fd < 0){
        CO_CANmodule_disable(CANmodule);
        return CO_ERROR_SYSCALL;
    }

    /* enable socket rx queue overflow detection */
    ovfl = 1;
    ret = setsockopt(CANmodule->fd, SOL_SOCKET, SO_RXQ_OVFL, &ovfl, sizeof(ovfl));
    if(ret < 0){
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
//    if (sLen == sizeof(bytes)) {
//        printf("socketCAN rx buffer size: %d bytes\n", bytes);
//    } todo

    /* bind socket */
    memset(&sockAddr, 0, sizeof(sockAddr));
    sockAddr.can_family = AF_CAN;
    sockAddr.can_ifindex = CANbaseAddress;
    ret = bind(CANmodule->fd, (struct sockaddr*)&sockAddr, sizeof(sockAddr));
    if(ret < 0){
        CO_CANmodule_disable(CANmodule);
        return CO_ERROR_SYSCALL;
    }

    /* set up error frame generation. What actually is available depends on your
     * CAN kernel driver */
#ifdef DEBUG
    err_mask = CAN_ERR_MASK; //enable ALL error frames
#else
    err_mask = CAN_ERR_ACK | CAN_ERR_CRTL | CAN_ERR_LOSTARB | CAN_ERR_BUSOFF |
               CAN_ERR_BUSERROR;
#endif
    ret = setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask,
                     sizeof(err_mask));
    if(ret < 0){
        CO_CANmodule_disable(CANmodule);
        return CO_ERROR_SYSCALL;
    }

    /* rx is started by calling #CO_CANsetNormalMode() */
    ret = disableRx(CANmodule);

    return ret;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    /* turn off the module */

    if (CANmodule == NULL) {
      return;
    }

    if (CANmodule->fd >= 0) {
        close(CANmodule->fd);
    }
    CANmodule->fd = -1;

    if (CANmodule->rxFilter != NULL) {
        free(CANmodule->rxFilter);
    }
    CANmodule->rxFilter = NULL;
}


/******************************************************************************/
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg)
{
    /* remove socketCAN flags */
    return (uint16_t) (rxMsg->ident & CAN_SFF_MASK);
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint32_t                index,
        uint32_t                ident,
        uint32_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->pFunct = pFunct;

        /* CAN identifier and CAN mask, bit aligned with CAN module */
        buffer->ident = ident & CAN_SFF_MASK;
        if(rtr){
            buffer->ident |= CAN_RTR_FLAG;
        }
        buffer->mask = (mask & CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;

        /* Set CAN hardware module filter and mask. */
        CANmodule->rxFilter[index].can_id = buffer->ident;
        CANmodule->rxFilter[index].can_mask = buffer->mask;
        if(CANmodule->CANnormal){
            ret = setRxFilters(CANmodule);
        }
    }
    else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint32_t                index,
        uint32_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier and rtr */
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
    CO_ReturnError_t err;
    err = CO_CANCheckSend(CANmodule, buffer);
    if (err == CO_ERROR_TX_BUSY) {
        /* send doesn't have "busy" */
#ifdef USE_EMERGENCY_OBJECT
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
#endif
        err = CO_ERROR_TX_OVERFLOW;
    }
    return err;
}


/******************************************************************************/
CO_ReturnError_t CO_CANCheckSend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;
    ssize_t n;
    size_t count;

    if (CANmodule==NULL || CANmodule->fd < 0) {
        return CO_ERROR_PARAMETERS;
    }

    count = sizeof(struct can_frame);
    do {
        errno = 0;
        n = send(CANmodule->fd, buffer, count, MSG_DONTWAIT);
        if (errno==EINTR) {
            /* try again */
            continue;
        }
        else if (errno==EAGAIN) {
            /* socket queue full */
            return CO_ERROR_TX_OVERFLOW;
        }
        else if (errno == ENOBUFS) {
            /* socketCAN doesn't support blocking write. You can wait here for
             * a few hundred us and then try again */
            return CO_ERROR_TX_BUSY;
        } else if (n <= 0) {
            break;
        }
    } while (errno != 0);

    if(n != count){
#ifdef USE_EMERGENCY_OBJECT
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
#endif
        err = CO_ERROR_TX_OVERFLOW;
    }

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    /* Messages are either written to the socket queue or dropped */
}


/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule)
{
  /* socketCAN doesn't support microcontroller-like error counters. If an
   * error has occured, a special can message is created by the driver and
   * received by the application like a regular message.
   * Therefore, error counter evaluation is included in rx function.*/
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
#ifdef USE_EMERGENCY_OBJECT
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_RXB_OVERFLOW,
                           CO_EMC_CAN_OVERRUN, n);
#endif
            return CO_ERROR_SYSCALL;
        }
    } while (errno != 0);

    /* check for rx queue overflow */
    cmsg = CMSG_FIRSTHDR(&msghdr);
    if ((cmsg != NULL) && (cmsg->cmsg_level == SOL_SOCKET) &&
        (cmsg->cmsg_type == SO_RXQ_OVFL))
    {
        dropped = *(uint32_t*)CMSG_DATA(cmsg);
        if (dropped > CANmodule->rxDropCount) {
#ifdef USE_EMERGENCY_OBJECT
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_RXB_OVERFLOW,
                           CO_EMC_COMMUNICATION, 0);
#endif
        }
        CANmodule->rxDropCount = dropped;
    }

    return CO_ERROR_NO;
}

/******************************************************************************/
int32_t CO_CANrxWait(CO_CANmodule_t *CANmodule, CO_CANrxMsg_t *buffer)
{
    int32_t retval;
    CO_ReturnError_t err;
    struct can_frame msg;

    if (CANmodule==NULL || CANmodule->fd<0) {
        return -1;
    }

    /* blocking read */
    err = CO_CANread(CANmodule, &msg);
    if (err != CO_ERROR_NO) {
        return -1;
    }

    retval = -1;
    if(CANmodule->CANnormal){

        if (msg.can_id & CAN_ERR_FLAG) {
            //todo
        } else {
            CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
            uint16_t index;             /* index of received message */
            uint32_t rcvMsgIdent;       /* identifier of the received message */
            CO_CANrx_t *rcvMsgObj = NULL; /* receive message object from CO_CANmodule_t object. */
            bool_t msgMatched = false;

            /* CANopenNode can message is compatible to the socketCAN one, except
             * for extension flags */
            msg.can_id &= CAN_EFF_MASK;
            rcvMsg = (CO_CANrxMsg_t *) &msg;
            rcvMsgIdent = rcvMsg->ident;

            /* Message has been received. Search rxArray from CANmodule for the
             * same CAN-ID. */
            rcvMsgObj = &CANmodule->rxArray[0];
            for(index = CANmodule->rxSize; index > 0U; index--){
                if(((rcvMsgIdent ^ rcvMsgObj->ident) & rcvMsgObj->mask) == 0U){
                    msgMatched = true;
                    break;
                }
                rcvMsgObj++;
            }
            if(msgMatched) {
                /* Call specific function, which will process the message */
                if ((rcvMsgObj != NULL) && (rcvMsgObj->pFunct != NULL)){
                    rcvMsgObj->pFunct(rcvMsgObj->object, rcvMsg);
                }
                /* return message */
                if (buffer != NULL) {
                    memcpy(buffer, rcvMsg, sizeof(*buffer));
                }
                retval = index;
            }
        }
    }
    return retval;
}
