/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
//*****************************************************************************
//
// ti rtos kernal
// 2/11/23
// Jannel Bennett
//
// For this lab;
// 1. init a2d (this can be in a function)
// 2. in the a2d you will need to set up interrupt like norm. you DO NOT need to do the NVIC reg stuff, the rtos will handle
// 3. create the HWI object and associate interrupt #40, which is the a2d with that object as well as w.e. function you write to
//    handle what happens when the a2d interrupt occurs
// 4. if you add other objects, hwi or swi , you need to #include 's for those added (BIOS module Headers)
// 5. print using RTOS     System_printf("SETPT: &d\n", setpoint_value);
// 6. need to change some lines in *The System.SupportProxy , system.h
//
/* Included is imported project from Resource Explorer 'Event' as a starting block
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 */

/*
 *  ======== event_w_notes.c ========
 */
/*
 * The code is going to have two tasks, and they're going to work with each other to send some information out to
 *  the console. These are going to use READER and WRITER tasks. The WRITER task writes information into a mailbox and then
 *  the READER task reads that information from the mailbox and prints it to the console.
 *
// ** PIN 2.7 Not Working
//
// Initialize GPIO. Setup inputs and outputs as follows:
//
//  Port pin    in/out  Pullup/down     Connect
//    P1.1        In      Up               S1
//    P1.4        In      Up               S2
//    P2.0        Out     N/A             LED-R
//    P2.2        Out     N/A             LED-B
//    P2.5        In      N/A             LED - TA0.2
//
//  *********   Nokia LCD interface reference   **************
//
// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// UCA3STE       (SCE, pin 3) connected to P9.4
// Reset         (RST, pin 4) connected to P9.3
// Data/Command  (D/C, pin 5) connected to P9.2
// UCA3SIMO      (DN,  pin 6) connected to P9.7
// UCA3CLK       (SCLK, pin 7) connected to P9.5
// back light    (LED, pin 8) not connected, consists of 4 3.3 V white LEDs which draw ~80mA total

//****************************************************************************
/*
 *  ======== event.c ========
 */

/* XDC module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <ti/drivers/Board.h>

#define NUMMSGS         3       /* Number of messages */
#define TIMEOUT         12      /* Timeout value */

#define TASKSTACKSIZE   512

typedef struct MsgObj {
    Int         id;             /* Writer task id */
    Char        val;            /* Message value */
} MsgObj, *Msg;

Void clk0Fxn(UArg arg0);
Void clk1Fxn(UArg arg0);
Void readertask(UArg arg0, UArg arg1);
Void writertask(UArg arg0, UArg arg1);

Task_Struct task0Struct, task1Struct;
Char task0Stack[TASKSTACKSIZE], task1Stack[TASKSTACKSIZE];
Semaphore_Struct sem0Struct, sem1Struct;
Semaphore_Handle semHandle;
Event_Struct evtStruct;
Event_Handle evtHandle;
Clock_Struct clk0Struct, clk1Struct;
Clock_Handle clk0Handle, clk1Handle;
Mailbox_Struct mbxStruct;
Mailbox_Handle mbxHandle;


/*
 * LAB6 Specific Includes/Defines/Variables/Functions Prototypes
 */
    // RTOS - Knl includes //
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>
    // MSOE Specific - includes//
#include <stdint.h>
#include <msp.h>
#include "msp.h"
#include "msoe_lib_lcd.h"
    //  my Function Prototypes //
void init_A2D(void);
void stupLCD(void);
void init_gpio(void);


/*
 *  ======== main ========
 */
int main()
{
    /* Construct BIOS Objects */
    Task_Params taskParams;
    Semaphore_Params semParams;
    Clock_Params clkParams;
    Mailbox_Params mbxParams;

    /* Call driver init functions */
    Board_init();

    /* Construct writer/reader Task threads */
    Task_Params_init(&taskParams);
    taskParams.arg0 = (UArg)mbxHandle;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)writertask, &taskParams, NULL);

    taskParams.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)readertask, &taskParams, NULL);

    Event_construct(&evtStruct, NULL);

    /* Obtain event instance handle */
    evtHandle = Event_handle(&evtStruct);

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semParams.event = evtHandle;
    semParams.eventId = Event_Id_01;
    Semaphore_construct(&sem0Struct, 0, &semParams);

    semHandle = Semaphore_handle(&sem0Struct);

    Clock_Params_init(&clkParams);
    clkParams.startFlag = TRUE;
    Clock_construct(&clk0Struct, (Clock_FuncPtr)clk0Fxn,
                    5, &clkParams);

    Clock_construct(&clk1Struct, (Clock_FuncPtr)clk1Fxn,
                    10, &clkParams);

    clk0Handle = Clock_handle(&clk0Struct);
    clk1Handle = Clock_handle(&clk1Struct);

    /* Construct a Mailbox Instance */
    Mailbox_Params_init(&mbxParams);
    mbxParams.readerEvent = evtHandle;
    mbxParams.readerEventId = Event_Id_02;
    Mailbox_construct(&mbxStruct,sizeof(MsgObj), 2, &mbxParams, NULL);
    mbxHandle = Mailbox_handle(&mbxStruct);

    BIOS_start();    /* Does not return */
    return(0);
}

/*
 *  ======== clk0Fxn =======
 */
Void clk0Fxn(UArg arg0)
{
    /* Explicit posting of Event_Id_00 by calling Event_post() */
    Event_post(evtHandle, Event_Id_00);
}

/*
 *  ======== clk1Fxn =======
 */
Void clk1Fxn(UArg arg0)
{
    /* Implicit posting of Event_Id_01 by Sempahore_post() */
    Semaphore_post(semHandle);
}

/*
 *  ======== reader ========
 */
Void readertask(UArg arg0, UArg arg1)
{
    MsgObj msg;
    UInt posted;

    for (;;) {
        /* Wait for (Event_Id_00 & Event_Id_01) | Event_Id_02 */
        posted = Event_pend(evtHandle,
            Event_Id_00 + Event_Id_01,  /* andMask */
            Event_Id_02,                /* orMask */
            TIMEOUT);

        if (posted == 0) {
            System_printf("Timeout expired for Event_pend()\n");
            break;
        }

        if ((posted & Event_Id_00) && (posted & Event_Id_01)) {
            /*
             * The following call to Semaphore_pend() will update the embedded
             * Event object to reflect the state of the semaphore's count after
             * the return from Semaphore_pend().
             * If the count is zero, then Event_Id_01 is cleared in the Event
             * object. If the count is non-zero, then Event_Id_01 is set in
             * the Event object.
             */
            if (Semaphore_pend(semHandle, BIOS_NO_WAIT)) {
                System_printf("Explicit posting of Event_Id_00 and Implicit posting of Event_Id_01\n");
            }
            else {
                System_printf("Semaphore not available. Test failed!\n");
            }
            break;
        }
        else if (posted & Event_Id_02) {
            System_printf("Implicit posting of Event_Id_02\n");
            /*
             * The following call to Mailbox_pend() will update the embedded
             * Event object to reflect whether messages are available in the
             * Mailbox after the current message is removed.
             * If there are no more messages available, then Event_Id_02 is
             * cleared in the Event object. If more messages are available,
             * then Event_Id_02 is set in the Event object.
             */
            if (Mailbox_pend(mbxHandle, &msg, BIOS_NO_WAIT)) {
                /* Print value */
                System_printf("read id = %d and val = '%c'.\n",msg.id, msg.val);
            }
            else {
                System_printf("Mailbox not available. Test failed!\n");
            }
        }
        else {
            System_printf("Unknown Event\n");
            break;
        }
    }
    BIOS_exit(0);
}

/*
 *  ======== writer ========
 */
Void writertask(UArg arg0, UArg arg1)
{
    MsgObj      msg;
    Int i;

    for (i=0; i < NUMMSGS; i++) {
        /* Fill in value */
        msg.id = i;
        msg.val = i + 'a';

        System_printf("writing message id = %d val = '%c' ...\n",
        msg.id, msg.val);

        /* Enqueue message */
        Mailbox_post(mbxHandle, &msg, TIMEOUT);
    }

    System_printf("writer done.\n");
}
/*
 * ======= LAB6 Specific ========
 */
// initialize ports
void init_gpio(void)
{
//      // set unused pins to pullup/down enabled to avoid floating inputs
//    P1->REN |= 0xFF;
//    P2->REN |= 0xFF;
//    P3->REN |= 0xFF;
//    P4->REN |= 0xFF;
//    P5->REN |= 0xFF;
//    P6->REN |= 0xFF;
//    P7->REN |= 0xFF;
//    P8->REN |= 0xFF;
//    P9->REN |= 0xFF;
//    P10->REN |= 0xFF;


    //R&B LED's init
    P2->DIR |= (BIT0 | BIT1);  // output
    P2->OUT &=~ (BIT0 | BIT1);  //start off

//    //Push1 & Push2 init
//    P1->DIR&=~(BIT1|BIT4); // set to zero for input
//    P1->REN|=(BIT1|BIT4); // enable pullup
//    P1->OUT |= (BIT1|BIT4);  // activate pullup


//    // External Switch
//    P6->DIR&=~(BIT1); // set to zero for input
//    P6->REN|=(BIT1); // enable pullup
//    P6->OUT|=(BIT1); // activate pullup

    // A6 (P4.7), A8 (P4.5) ADC input setup - relative Humidity / ambient Temp
    P4->SEL1 |= (BIT5 | BIT7);
    P4->SEL0 |= (BIT5 | BIT7);

      return;
}
// initialize A/D
void init_A2D(void) // check settings
{
    // Sampling time, S&H=96, ADC14 on, SMCLK, repeat seq of channels, repeated conv.
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_4 | ADC14_CTL0_ON
             | ADC14_CTL0_CONSEQ_3  | ADC14_CTL0_MSC;
    ADC14->CTL1 |= ADC14_CTL1_RES_1;  // 10-bit conversion
    ADC14->CTL1 &= ~ADC14_CTL1_RES_2;  // 10-bit conversion
    ADC14->CTL1 |= (3 << ADC14_CTL1_CSTARTADD_OFS);  // start w/ MEM[3]
    ADC14->MCTL[3] |= ADC14_MCTLN_INCH_8; // input on A8 to mem3
    ADC14->MCTL[4] |= ADC14_MCTLN_INCH_6;  // input on A6 to mem4
    ADC14->IER0 |= ADC14_IER0_IE3 | ADC14_IER0_IE4;  // enable interrupt for MEM0 and MEM1
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;  // enable and start first

    NVIC->ISER[0] |= INT_ADC14_BIT;  // enable ADC interrupt in NVIC

}

// setup LCD
void stupLCD(void){
    LCD_Config();
    LCD_clear();

//    LCD_goto_xy(0,0);
//    LCD_print_str("SetP");
//
//    LCD_goto_xy(0,1);
//    LCD_print_str("Amb");
//
//    LCD_goto_xy(0,2);
//    LCD_print_str("RH");
//
//    LCD_goto_xy(0,3);
//    LCD_print_str("Defrost");
}
