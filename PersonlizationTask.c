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

/*
 *  ====================== CC2650STK.c =========================================
 *  This file is responsible for setting up the board specific items for the
 *  CC2650 SensorTag.
 */


/*
 *  ====================== Includes ============================================
 */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC26XX.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

//#include <time.h>
//#include <ti/sysbios/knl/Clock.h>
//#include <ti/sysbios/hal/Seconds.h>

#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/ioc.h>
#include <driverlib/udma.h>

#include <Board.h>

//#include "NodeRadioTask.h"
#include "PersonalizationTask.h"

/*
 *  ========================= IO driver initialization =========================
 *  From main, PIN_init(BoardGpioInitTable) should be called to setup safe
 *  settings for this board.
 *  When a pin is allocated and then de-allocated, it will revert to the state
 *  configured in this table.
 */

//static uint16_t last= 5;


uint8_t getSleepValue()
{
    now=NodeRadioTask_UnixTime();
    t = *localtime(&now);
    if (now < 799000000 )  // Check if the time is right
        return 60;
    if (t.tm_hour == 5 && t.tm_min < 5 )
        return 60;
    if (t.tm_wday==SATURDAY || t.tm_wday==SUNDAY)
        return 0;

    ///// For testing
    char       buf[80];
    strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &t);
    /////
    uint8_t i;
    for(i=NodeOperationMode_SleepingTableLength; i > 0 ; i--)
    {
        if (t.tm_hour >= NodeOperationMode_SleepingTablePeriods[i-1])
            return NodeOperationMode_SleepingValue[NodeOperationMode_SleepingTableModes[i-1]];
    }
    return 255;
}
