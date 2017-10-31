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

/***** Includes *****/
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <time.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/hal/Seconds.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Drivers */
#include <ti/drivers/UART.h>
#include <ti/drivers/PIN.h>


#include "ConcentratorRadioTask.h"
#include "ConcentratorTask.h"
#include "RadioProtocol.h"
#include "ConcentratorModemTask.h"


/* Board Header files */
#include "Board.h"



/***** Defines *****/
#define CONCENTRATORMODEM_TASK_STACK_SIZE 768
#define CONCENTRATORMODEM_TASK_PRIORITY   3

#define MODEM_EVENT_ALL                  0xFFFFFFFF
#define MODEM_EVENT_COMMAND_RECEIVED      (uint32_t)(1 << 0)
#define MODEM_EVENT_COMMAND_SEND    (uint32_t)(1 << 1)

#define CONCENTRATORMODEM_MAX_RETRIES 2
//#define NORERADIO_ACK_TIMEOUT_TIME_MS (160)

#define CONCENTRATOR_MODEM_ACTIVITY_LED Board_LED1 //Red LED

/***** Type declarations *****/

/***** Variable declarations *****/
static Task_Params concentratorModemTaskParams;
Task_Struct concentratorModemTask; /* not static so you can see in ROV */
static char concentratorModemTaskStack[CONCENTRATORMODEM_TASK_STACK_SIZE];
Event_Struct modemOperationEvent;  /* not static so you can see in ROV */
static Event_Handle modemOperationEventHandle;

UART_Handle modem;
UART_Params uartParams;
uint8_t modemStatus;

// Time and clock variables
#define TIMEZONE_SEC 4*3600
uint32_t cSec_TI;  // Last updated time in Unix Format
uint32_t cSec;  // Last updated time in Unix Format
uint32_t lastSec=1426692075;  // Last updated time in Unix Format

time_t tUinx;
struct tm t;
static time_t now;
//static ConcentratorModem_CommandReceivedCallback commandReceivedCallback;

//static buffers
char buf_time[80];
char buf_thingspace[250];

/***** Prototypes *****/
static void concentratorModemTaskFunction(UArg arg0, UArg arg1);
//static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
//static void notifyCommandReceived(union ConcentratorCommand* latestRxCommand);
//void Modem_readCallBack(UART_Handle handle, void *ptr, size_t size);
//static void sendAck(uint8_t latestSourceAddress);

/* Pin driver handles */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 is on.
 *   - LEDs Board_LED1 is off.
 */
PIN_Config ledPinTableModem[] = {
    CONCENTRATOR_MODEM_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};


#define MAX_NUM_RX_BYTES    400   // Maximum RX bytes to receive in one go
#define MAX_NUM_TX_BYTES    300   // Maximum TX bytes to send in one go
size_t wantedRxBytes;            // Number of bytes received so far
uint8_t rxBuf[MAX_NUM_RX_BYTES];   // Receive buffer
uint8_t txBuf[MAX_NUM_TX_BYTES];   // Transmit buffer
const char thingSpaceSenssorString[]="POST /dweet/for/SensorNet1?s1=2,0,0&s2=2,0,0&s3=2,0,0&s4=2,0,0&s5=2,0,0&s6=2,0,0&s7=2,0,0&Unix=0 HTTP/1.1\r\n\r\n";

/*
// Callback function
static void Modem_readCallBack(UART_Handle handle, void *rxBuf, size_t size)
{
    // Copy bytes from RX buffer to TX buffer
    uint_t i=0;
    for(i = 0; i < size; i++)
        txBuf[i] = ((uint8_t*)rxBuf)[i];
    // Echo the bytes received back to transmitter
    UART_write(handle, txBuf, size);
    // Start another read, with size the same as it was during first call to
    UART_read(handle, rxBuf, wantedRxBytes);
    Event_post(modemOperationEventHandle, MODEM_EVENT_COMMAND_RECEIVED);
}
*/

void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    CPUdelay(8000*50);
    if (!PIN_getInputValue(pinId)) {
            SendCommandPostEvent();
    }
}

/***** Function definitions *****/
void ConcentratorModemTask_init(void) {
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTableModem);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle) {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */

    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
        System_abort("Error registering button callback function");
    }


    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&modemOperationEvent, &eventParam);
    modemOperationEventHandle = Event_handle(&modemOperationEvent);

    /* Create the concentrator modem protocol task */
    Task_Params_init(&concentratorModemTaskParams);
    concentratorModemTaskParams.stackSize = CONCENTRATORMODEM_TASK_STACK_SIZE;
    concentratorModemTaskParams.priority = CONCENTRATORMODEM_TASK_PRIORITY;
    concentratorModemTaskParams.stack = &concentratorModemTaskStack;
    Task_construct(&concentratorModemTask, concentratorModemTaskFunction, &concentratorModemTaskParams, NULL);
}

//void concentratorModemTask_registerPacketReceivedCallback(ConcentratorModem_CommandReceivedCallback callback) {
    //commandReceivedCallback = callback;
//}

void SendCommandPostEvent()
{
    if((modemStatus!=ConcentratorModemStatus_Busy))
        Event_post(modemOperationEventHandle, MODEM_EVENT_COMMAND_SEND);
    if (modemStatus==ConcentratorModemStatus_WaitForNocareer){
        Task_sleep(1000);
        Event_post(modemOperationEventHandle, MODEM_EVENT_COMMAND_SEND);
    }
}

static void concentratorModemTaskFunction(UArg arg0, UArg arg1)
{
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_FULL;
    //uartParams.readMode = UART_MODE_CALLBACK;
    //uartParams.readCallback = Modem_readCallBack;
    uartParams.readTimeout = 5000;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 921600;
    //uartParams.baudRate = 115200;
    //uartParams.baudRate = 9600;
    modem = UART_open(Board_UART0, &uartParams);

    // GEt the time from LTE network (Should update once a day)

    if (modem == NULL) {
        System_abort("Error opening the UART");
    }
    uint8_t len;
    UART_write(modem, "AT\r\n",4);
    len=CATM_readto(modem,(uint8_t*)&rxBuf[0],"OK",2);
    UART_write(modem, "AT+CFUN=1\r\n\r\n",13);
    System_printf("Turning on the modem...\n");
    System_flush();
    len=CATM_readto(modem,(uint8_t*)&rxBuf[0],"OK",2);
    Task_sleep(3000);
    Task_sleep(3000);
    len=CATM_readto(modem,(uint8_t*)&rxBuf[0],"READY",5);
    len=CATM_getIMEI(modem, (uint8_t*)&rxBuf[0]);
    System_printf("%i-IMEI is Received: %s\n",len,rxBuf);
    System_printf("Synch time with LTE network...\n");
    t = CATM_getTime(modem, (uint8_t*)&rxBuf[0]);
    UART_write(modem, "AT+CFUN=0\r\n\r\n",13);
    len=CATM_readto(modem,(uint8_t*)&rxBuf[0],"OK",2);
    Task_sleep(3000);
    len=CATM_readall(modem,(uint8_t*)&rxBuf[0]);
    System_flush();
    uint8_t modemState = CATM_sendData(modem, (uint8_t*)&rxBuf[0], (uint8_t*)&txBuf[0]);
    //Task_sleep(3000);
    //t = CATM_getTime(modem, (uint8_t*)&rxBuf[0]);

    while (1) {
        uint32_t events = Event_pend(modemOperationEventHandle, 0, MODEM_EVENT_ALL, BIOS_WAIT_FOREVER);
        /* If valid packet received */
        if(events & MODEM_EVENT_COMMAND_SEND) {
            //txBuf="at\r\n";
            //int sent_num;
            //sent_num=UART_write(modem, txBuf, 3);
            //if(sent_num==UART_ERROR)
                //System_printf("ERROR sending UART\n");
            System_printf("Time to push data to the Cloud, Status: %i\n\n",modemStatus);
            System_flush();

            //CATM_convert2string(txBuf, (uint8_t*)&txBuf[0]);
            uint8_t modemState = CATM_sendData(modem, (uint8_t*)&rxBuf[0], (uint8_t*)&txBuf[0]);

            /* Send ack packet */
            //sendAck(latestRxPacket.header.sourceAddress);

            /* Call packet received callback */
            //notifyPacketReceived(&latestRxPacket);

            /* Go back to RX */
            //if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                //System_abort("EasyLink_receiveAsync failed");
            //}

            /* toggle Activity LED */
            PIN_setOutputValue(ledPinHandle, CONCENTRATOR_MODEM_ACTIVITY_LED,
                    !PIN_getOutputValue(CONCENTRATOR_MODEM_ACTIVITY_LED));
        }

        /* If invalid packet received */
        if(events & MODEM_EVENT_COMMAND_RECEIVED) {
                System_printf("Received Some Data form Modem: %s\n",rxBuf);
                System_flush();
            /* Go back to RX */
            //if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                //System_abort("EasyLink_receiveAsync failed");
            //}
        }
    }
}

uint8_t CATM_sendData(UART_Handle uart, uint8_t *rxBuf, uint8_t *txBuf)
{
    modemStatus=ConcentratorModemStatus_Busy;
    uint8_t len=1;  // :/
    while (t.tm_min%10 < 3 || t.tm_min%10>8)
    {
        now=Seconds_get();
        t = *localtime(&now);
        Task_sleep(2000);
    }
    if (len){
                UART_write(uart, "AT+CFUN=1\r\n\r\n",13);
                //System_printf("AT command sent Wait For OK ...\n");
                //System_flush();
                len=CATM_readto(uart,(uint8_t*)&rxBuf[0],"OK",2);
                Task_sleep(2000);
                Task_sleep(3000);
                len=CATM_readto(uart,(uint8_t*)&rxBuf[0],"READY",5);
                t = CATM_getTime(modem, (uint8_t*)&rxBuf[0]);
                Task_sleep(2000);

                if (len==1)
                    System_printf("%i- AT+CFUN=1 is set: %s\n",len,rxBuf);
                else
                    System_printf("%i- ERROR to set AT+CFUN=1: %s\n",len,rxBuf);
                System_flush();
                UART_write(uart, "AT+SQNSD=1,0,80,\"thingspace.io\"\r\n\r\n",34);
                System_printf("AT command sent Wait For CONNECT...\n");
                System_flush();
                Task_sleep(1000);
                len=CATM_readto(uart,(uint8_t*)&rxBuf[0],"CONNECT",7);
                if (len!=1){
                    System_printf("%i- Let's reboot the device and try again: %s \n\n",len,rxBuf);
                    System_flush();
                    UART_write(uart, "AT^RESET\r\n",10);
                    modemStatus=ConcentratorModemStatus_FailedtoConnect;
                    Task_sleep(10000);
                    Task_sleep(3000);
                    len=CATM_readall(uart,(uint8_t*)&rxBuf[0]);
                    System_printf("%i- Rebooted the device, try again: %s \n\n",len,rxBuf);
                    System_flush();
                    // Have to resend the data (make sure)
                    //CATM_sendData
                    return 0;
                }
                if (len==1){
                    uint8_t txLen=CATM_convert2string((uint8_t*)&txBuf[0]);
                    modemStatus=ConcentratorModemStatus_WaitForNocareer;
                    //UART_write(uart, "POST /dweet/for/SensorNetM?s1=1&s2=1&s3=1&s4=1&s5=1&s6=1&s7=1 HTTP/1.1\r\n\r\n",74);
                    System_printf("txBuf: %s \n\n",txBuf);
                    System_flush();
                    UART_write(uart, txBuf,txLen);
                    System_printf("%i- Connected, wait for response: %s \n\n",len,rxBuf);
                    System_flush();
                    Task_sleep(2000);
                    len=CATM_readto(uart,(uint8_t*)&rxBuf[0],"NO CARRIER",10);
                    if(len!=1){
                            modemStatus=ConcentratorModemStatus_InvaliedResponse;
                            System_printf("%i- Invalid Response: %s \nTurn modem off\n",len,rxBuf);
                            System_flush();
                    }
                    else{
                        modemStatus=ConcentratorModemStatus_ValidResponse;
                        System_printf("%i- Data is sent: %s \nTurn modem off\n",len,rxBuf);
                        System_flush();
                    }
                    UART_write(modem, "AT+CFUN=0\r\n\r\n",13);
                    len=CATM_readto(modem,(uint8_t*)&rxBuf[0],"OK",2);
                    Task_sleep(3000);
                    len=CATM_readall(uart,(uint8_t*)&rxBuf[0]);
                }
            }
    else
        {
            System_printf("Error ...\n");
            System_flush();
            modemStatus=ConcentratorModemStatus_Off;
        }

    return len;
}

uint8_t CATM_convert2string(uint8_t *txBuf)
{
    int i,j;
    uint8_t sData[7]={2,2,2,2,2,2,2};
    uint16_t sBatt[7]={0,0,0,0,0,0,0};
    uint32_t sAdd[7]={30001,30002,30003,30004,30005,30006,30007};
    uint32_t sLastTime[7]={0,0,0,0,0,0,0};
    uint32_t tempTIme = Seconds_get();
    for (i = 0; i < 7; i++) {
        for (j=0;j<7;j++)
            if (knownSensorNodes[i].parkingAddress == sAdd[j])
            {
                sData[j]=knownSensorNodes[i].button;
                sBatt[j]=knownSensorNodes[i].batt;
                sLastTime[i] = (tempTIme - knownSensorNodes[i].time)/60;
            }
    }

    //sprintf(buf_thingspace,
//"POST /dweet/for/SensorNetM?%u=%u,%u&%u=%u,%u&%u=%u,%u&%u=%u,%u&%u=%u,%u&%u=%u,%u&%u=%u,%u HTTP/1.1\r\n\r\n",
    //sAdd[0],sData[0],sBatt[0],sAdd[1],sData[1],sBatt[1],sAdd[2],sData[2],sBatt[2],
    //sAdd[3],sData[3],sBatt[3],sAdd[4],sData[4],sBatt[4],sAdd[5],sData[5],sBatt[5],
    //sAdd[6],sData[6],sBatt[6]);
    sprintf(buf_thingspace,
"POST /dweet/for/SensorNet1?s1=%u,%u,%u&s2=%u,%u,%u&s3=%u,%u,%u&s4=%u,%u,%u&s5=%u,%u,%u&s6=%u,%u,%u&s7=%u,%u,%u&Unix=%u HTTP/1.1\r\n\r\n",
    sData[0],sBatt[0],sLastTime[0],sData[1],sBatt[1],sLastTime[1],sData[2],sBatt[2],sLastTime[2],sData[3],sBatt[3],sLastTime[3],
    sData[4],sBatt[4],sLastTime[4],sData[5],sBatt[5],sLastTime[5],sData[6],sBatt[6],sLastTime[6],Seconds_get());

    memcpy(txBuf,buf_thingspace,sizeof(buf_thingspace));
    return (uint8_t)sizeof(buf_thingspace);
}

/*
uint8_t CATM_convert2string_old(uint8_t *txBuf)
{
    int i,j;
    char sensorData[7]={'2','2','2','2','2','2','2'};
    uint32_t sensorAddress[7]={30001,30002,30003,30004,30005,30006,30007};
    for (i = 0; i < 7; i++) {
        for (j=0;j<7;j++)
            if (knownSensorNodes[i].parkingAddress == sensorAddress[j])
            {
                if(knownSensorNodes[i].button==1)
                    sensorData[j]='1';
                else if(knownSensorNodes[i].button==0)
                    sensorData[j]='0';
                else
                    sensorData[j]='2';
                break;
                //knownSensorNodes[i].batt
            }
    }

    uint8_t thingSpaceSenssorString[]="POST /dweet/for/SensorNetM?s1=2,0&s2=2,0&s3=2,0&s4=2,0&s5=2,0&s6=2,0&s7=2,0 HTTP/1.1\r\n\r\n";
    const uint8_t sensor_ind[7]={30, 35, 40, 45, 50, 55, 60};
    for (i=0;i<7;i++)
        thingSpaceSenssorString[sensor_ind[i]]=(char)sensorData[i];
    memcpy(txBuf,thingSpaceSenssorString,sizeof(thingSpaceSenssorString)+3);
    return (uint8_t)sizeof(thingSpaceSenssorString)+3;
}
*/

uint8_t CATM_readto(UART_Handle handle, uint8_t *rxBuf,const char *str,size_t size)
{
    memset(rxBuf, 0, MAX_NUM_RX_BYTES);
    uint_t i=0;
    uint8_t input;
    uint8_t j=0;
    uint8_t k=0;
    uint_t counter=0;
    while(counter<200)
    {
        while ( UART_read(handle, &input, 1) && i < MAX_NUM_RX_BYTES)
        {
            rxBuf[i] = (uint8_t)input;
            i++;
            if ((uint8_t)input==(uint8_t)str[j])
                j++;
            else if((uint8_t)input==(uint8_t)MODEM_RESPONSE[ERROR][k])
                k++;
            else
                k=j=0;
            if (j==size)
                return 1;
            else if(k==5)
                return 255;
            else if(i==MAX_NUM_RX_BYTES)
                return 100;
        }
        counter++;
        Task_sleep(10);
    }
    if(i==1)
        return 2;
    return i;
}

uint8_t CATM_readall(UART_Handle handle, uint8_t *rxBuf)
{
    memset(rxBuf, 0, MAX_NUM_RX_BYTES);
    uint_t i=0;
    uint8_t input;
    uint_t counter=0;
    while(counter<5)
    {
        while ( UART_read(handle, &input, 1) && i < MAX_NUM_RX_BYTES)
        {
            rxBuf[i] = (uint8_t)input;
            i++;
            if(i==MAX_NUM_RX_BYTES)
                return 100;
        }
        counter++;
        Task_sleep(200);
    }
    return i;
}

struct tm CATM_str2Time(uint16_t len,uint8_t *rxBuf)
{
    System_printf("%i- Parsing modem time: %s \n\n",len,rxBuf);
    System_flush();
    struct tm t2;
    uint8_t i=0;
    for (i=3;i<len;i++)
    {
        if (rxBuf[i]=='/' && rxBuf[i-3]=='"')
            t2.tm_year=10*str2dec(rxBuf[i-2])+str2dec(rxBuf[i-1]) + 100;
        if (rxBuf[i]=='/' && rxBuf[i-3]=='/')
            t2.tm_mon=10*str2dec(rxBuf[i-2])+str2dec(rxBuf[i-1])-1;
        if (rxBuf[i]==',' && rxBuf[i-3]=='/')
            t2.tm_mday=10*str2dec(rxBuf[i-2])+str2dec(rxBuf[i-1]);
        if (rxBuf[i]==':' && rxBuf[i-3]==',')
            t2.tm_hour=10*str2dec(rxBuf[i-2])+str2dec(rxBuf[i-1]);
        if (rxBuf[i]==':' && rxBuf[i-3]==':')
            t2.tm_min=10*str2dec(rxBuf[i-2])+str2dec(rxBuf[i-1]);
        if (rxBuf[i]=='-' && rxBuf[i-3]==':')
            t2.tm_sec=10*str2dec(rxBuf[i-2])+str2dec(rxBuf[i-1]);
    }
    t2.tm_isdst = -1;
    return t2;
}

uint8_t str2dec(uint8_t str)
{
    return str-48;
}


uint8_t CATM_getIMEI(UART_Handle handle, uint8_t *rxBuf)
{
    UART_write(handle, "AT+CGSN\r\n",9);
    return (CATM_readto(handle, rxBuf,"OK",2));

}
struct tm CATM_getTime(UART_Handle handle, uint8_t *rxBuf)
{
    struct tm  ts;
    // Get current time
    //time(&lastSec);
    lastSec = Seconds_get();
    UART_write(handle, "AT+CCLK?\r\n",10);
    ts = CATM_str2Time(CATM_readall(handle, rxBuf),rxBuf);
    cSec = mktime(&ts) - TIMEZONE_SEC ;
    Seconds_set( cSec );

    ts = *localtime(&cSec);
    strftime(buf_time, sizeof(buf_time), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
    System_printf("%s   -  Unix: %u    Last Unix: %u\n", buf_time,cSec,lastSec);
    System_flush();
    return ts;
}

uint32_t CATM_cTime()
{
    cSec=Seconds_get();
    return cSec;
}

//static void sendAck(uint8_t latestSourceAddress) {
    /* Set destinationAdress, but use EasyLink layers destination adress capability */
    //txPacket.dstAddr[0] = latestSourceAddress;

    /* Copy ACK packet to payload, skipping the destination adress byte.
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    //memcpy(txPacket.payload, &ackPacket.header, sizeof(ackPacket));
    //txPacket.len = sizeof(ackPacket);

    /* Send packet  */
    //if (EasyLink_transmit(&txPacket) != EasyLink_Status_Success)
    //{
        //System_abort("EasyLink_transmit failed");
    //}
//}

/*
static void notifyCommandReceived(union ConcentratorCommand* latestRxCommand)
{
    int latestRssi=0;
    if (commandReceivedCallback)
    {
        commandReceivedCallback(latestRxCommand, latestRssi);
    }
}
*/
/*
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    union ConcentratorPacket* tmpRxPacket;

    // If we received a packet successfully
    if (status == EasyLink_Status_Success)
    {
       // Save the latest RSSI, which is later sent to the receive callback
        latestRssi = (int8_t)rxPacket->rssi;

        // Check that this is a valid packet
        tmpRxPacket = (union ConcentratorPacket*)(rxPacket->payload);

        // If this is a known packet
        if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_ADC_SENSOR_PACKET)
        {
            // Save packet
            memcpy((void*)&latestRxPacket, &rxPacket->payload, sizeof(struct AdcSensorPacket));

            // Signal packet received
            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
        {
            // Save packet
            memcpy((void*)&latestRxPacket, &rxPacket->payload, sizeof(struct DualModeSensorPacket));

            // Signal packet received
            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_PM_SENSOR_PACKET)
                {
                    // Save packet
                    memcpy((void*)&latestRxPacket, &rxPacket->payload, sizeof(struct ParkingModeSensorPacket));

                    // Signal packet received
                    Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
                }
        else
        {
            // Signal invalid packet received
            Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
        }
    }
    else
    {
        // Signal invalid packet received
        Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
    }
}
*/
