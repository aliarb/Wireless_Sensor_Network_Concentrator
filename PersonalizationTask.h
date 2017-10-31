/*
 * Copyright (c) Verzion Wireless
 * Aliasghar Arab
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
 ////////////////////////////////////////////////////////////////////

#ifndef PERSONALIZATIONTASK_H_
#define PERSONALIZATIONTASK_H_

#include "stdint.h"
#include "easylink/EasyLink.h"
#include <time.h>
#include <ti/sysbios/hal/Seconds.h>

#define offset         0// -4  // offset from UTC
static uint8_t NodeOperationMode_SleepingTableLength = 5;
static time_t now;
static struct tm  t;

enum NodeOperationMode {
    NodeOperationMode_Sleep,
    NodeOperationMode_Quiet,
    NodeOperationMode_SemiQuiet,
    NodeOperationMode_Busy,
    NodeOperationMode_Number,
};

static uint8_t NodeOperationMode_SleepingValue[NodeOperationMode_Number] = {  //Second
    0,
    30,
    15,
    5
};

static uint8_t NodeOperationMode_SleepingTablePeriods[] = {
    0,  // 0-6 Sleeping
    6,  // 6-10 Busy
    10, // 10-17 Semi-Quiet
    17, // 17-21 Quiet
    21, // 21-24 Sleeping
};

static uint8_t NodeOperationMode_SleepingTableModes[] = {
    NodeOperationMode_Sleep,
    NodeOperationMode_Busy,
    NodeOperationMode_SemiQuiet,
    NodeOperationMode_Quiet,
    NodeOperationMode_Sleep,
};

//list weekdays = ["Thursday", "Friday", "Saturday", "Sunday", "Monday", "Tuesday", "Wednesday"];

enum NodeWeekdaysList {
    SUNDAY,
    MONDAY,
    TUESDAY,
    WEDNESDAY,
    THURSDAY,
    FRIDAY,
    SATURDAY,
};

uint8_t getSleepValue();


#endif
