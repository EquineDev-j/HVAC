//*****************************************************************************
//
// ti rtos kernal
// 2/11/22
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
