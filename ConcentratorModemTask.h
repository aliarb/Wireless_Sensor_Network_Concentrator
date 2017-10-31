/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TASKS_CONCENTRATORMODEMTASKTASK_H_
#define TASKS_CONCENTRATORMODEMTASKTASK_H_

#include "stdint.h"
#include "string.h"
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

enum ConcentratorModemStatus {
    ConcentratorModemStatus_Off,
    ConcentratorModemStatus_On,
    ConcentratorModemStatus_Busy,
    ConcentratorModemStatus_FailedtoConnect,
    ConcentratorModemStatus_ValidResponse,
    ConcentratorModemStatus_InvaliedResponse,
    ConcentratorModemStatus_Rebooting,
    ConcentratorModemStatus_WaitForNocareer,
};

enum MODEM_RESPONSE_ENUM {
    OK, ERROR, CONNECT, NOCAREER, EndLine, EMPTY,
};

static const char *MODEM_RESPONSE[] = {
    "OK", "ERROR", "CONNECT", "NO CAREER", "\r\n", "",
};
//struct Time{
    //uint8_t year;
    //uint8_t month;
    //uint8_t day;
    //uint8_t hour;
    //uint8_t minutes;
    //uint8_t second;
//};

struct CommandHeader {
    uint8_t sourceAddress;
    uint8_t packetType;
    uint32_t time;
};

union ConcentratorCommand {
    struct CommandHeader  header;
    //struct AdcSensorPacket adcSensorPacket;
    //struct ParkingModeSensorPacket pmSensorPacket;
    //struct DualModeSensorPacket dmSensorPacket;
};

// AT COMMANDs for CAT-M SEQUANS/NIMBELINK //
uint8_t str2dec(uint8_t str);
uint8_t CATM_readto(UART_Handle handle, uint8_t *rxBuf,const char *str,size_t size);
uint8_t CATM_readall(UART_Handle handle, uint8_t *rxBuf);
uint8_t CATM_getIMEI(UART_Handle handle, uint8_t *rxBuf);
struct tm CATM_getTime(UART_Handle handle, uint8_t *rxBuf);
uint8_t CATM_readLine(UART_Handle handle, uint8_t *rxBuf);
uint8_t CATM_sendSMS(UART_Handle handle, uint8_t *rxBuf, uint8_t *txBuf);
uint8_t CATM_sendData(UART_Handle handle, uint8_t *rxBuf, uint8_t *txBuf);
uint8_t CATM_convert2string(uint8_t *txBuf);
uint32_t CATM_cTime();
struct tm CATM_str2Time(uint16_t len,uint8_t *rxBuf);

typedef void (*ConcentratorModem_CommandReceivedCallback)(union ConcentratorCommand* command, int8_t timeout);

/* Create the ConcentratorModemTask and creates all TI-RTOS objects */
void ConcentratorModemTask_init(void);

/* Register the packet received callback */
void ConcentratorModemTask_registerCommandReceivedCallback(ConcentratorModem_CommandReceivedCallback callback);

void SendCommandPostEvent();

#endif /* TASKS_CONCENTRATORMODEMTASKTASK_H_ */
