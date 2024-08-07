
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

    // RTOS - Knl includes //
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>
Hwi_Handle hwi0;
Swi_Handle swi0;

#define NUMMSGS         3       /* Number of messages */
#define TIMEOUT         12      /* Timeout value */

#define TASKSTACKSIZE   512


Void clk0Fxn(UArg arg0);
Void hwi0Fxn(UArg arg0);
Void swiFunc(UArg arg0, UArg arg1);
Void task0(UArg arg0, UArg arg1);
Void task1(UArg arg0, UArg arg1);

// each task has a structure //
Task_Struct task0Struct;
Task_Struct task1Struct;
Char task0Stack[TASKSTACKSIZE];
Char task1Stack[TASKSTACKSIZE];

Event_Struct evtStruct0, evtStruct1;
Event_Handle evtHandle0, evtHandle1;
Clock_Struct clk0Struct;
Clock_Handle clk0Handle;


/*
 * LAB6 Specific Includes/Defines/Variables/Functions Prototypes
 */

    // MSOE Specific - includes//
#define __MSP432P401R__
#define INT_ADC14_BIT (1<<24)
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
#include <msp.h>
#include "msp.h"
#include "msoe_lib_lcd.h"
    //  my Function Prototypes //
void init_A2D(void);
void stupLCD(void);
void init_gpio(void);

// Global Variables //
float raw_adc = 0;
int Fahrenheit = 0;

/*
 *  ======== main ========
 */
int main()
{
    /* Construct BIOS Objects */
    Task_Params taskParams;
    Clock_Params clkParams;
    Hwi_Params hwiParams;
    Swi_Params swiParams;

    /* Call driver init functions */
    Board_init();

    /* Construct Task threads */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 3; // less important than hwi, swi

    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)task0, &taskParams, NULL);
    Event_construct(&evtStruct0, NULL);
    evtHandle0 = Event_handle(&evtStruct0); // obtain event instance handle

    taskParams.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)task1, &taskParams, NULL);
    Event_construct(&evtStruct1, NULL);
    evtHandle1 = Event_handle(&evtStruct1); // obtain event instance handle


    Clock_Params_init(&clkParams);
    clkParams.startFlag = TRUE;
    clkParams.period = 5; // tick = 1ms or 1us?
    Clock_construct(&clk0Struct, (Clock_FuncPtr)clk0Fxn,
                    5, &clkParams);
    clk0Handle = Clock_handle(&clk0Struct);


    Hwi_Params_init(&hwiParams);
    hwi0 = Hwi_create(40, (Hwi_FuncPtr) hwi0Fxn , &hwiParams, NULL);
    // check //
    if (hwi0==NULL){
        System_abort("hwi create failed");
    }
    Hwi_enableInterrupt(40); // a2d is 24 but need to add 16 for RTOS interrupts


    Swi_Params_init(&swiParams);
    swi0 = Swi_create(swiFunc, &swiParams, NULL);
    // check //
    if(swi0==NULL){
        System_abort("swi create failed");
    }


    init_A2D();
    stupLCD();
    init_gpio(); // configure 4.5 as ADC

    BIOS_start();    /* Does not return */
    return(0);
}

/*
 *  ======== clk0Fxn =======
 */
Void clk0Fxn(UArg arg0)
{
    ADC14->CTL0 |= ADC14_CTL0_SC; // start
}
/*
 *  ======== init_gpio =======
 */
void init_gpio(void)
{

    // A8 (P4.5) ADC input setup - ambient Temp
    P4->SEL1 |= (BIT5);
    P4->SEL0 |= (BIT5);

      return;
}
/*
 *  ======== init_A2D =======
 */
void init_A2D(void) // check settings for this application !!!!!!
{
                 // S&H=96, sample-and-hold pulse-mode select, MCLK, ADC14 on ,single channel single conversion.
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_ON
             | ADC14_CTL0_CONSEQ_0 ;

    ADC14->CTL1 |= ADC14_CTL1_RES_1;  // 10-bit conversion
    ADC14->CTL1 &= ~ADC14_CTL1_RES_2;  // 10-bit conversion
    ADC14->CTL1 |= (3 << ADC14_CTL1_CSTARTADD_OFS);  // start w/ MEM[3]
    ADC14->MCTL[3] |= ADC14_MCTLN_INCH_8; // input on A8 to mem3
    ADC14->IER0 |= ADC14_IER0_IE3;  // enable interrupt for MEM3
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;  // enable and start


}
/*
 *  ======== stupLCD =======
 */
void stupLCD(void){
    LCD_Config();
    LCD_clear();

    LCD_goto_xy(0,0);
    LCD_print_str("SetPt:");

    LCD_goto_xy(0,1);
    LCD_print_str("Raw ADC Val:");

    LCD_goto_xy(0,3);
    LCD_print_str("Prev ADC Val:");

}
/*
 *  ======== hwi0Fxn =======
 */
void hwi0Fxn(UArg arg0){ // keep hwi short and only for critical stuff

    // reading clears flag
    uint16_t dummy;         // dummy variable
    dummy = P1->IV;         // clear flag 'Interrupt Vector Reg' , store val dummy
    ADC14->CLRIFGR0|=BIT3;  // clear pending interrupt flag
    Swi_post(swi0);         // swi to do more ...

}
/*
 *  ======== swi0Fxn =======
 */
void swiFunc(UArg arg0, UArg arg1){

    Event_post(evtHandle0, Event_Id_00);

}
/*
 *  ======== task0 =======
 */
void task0(UArg arg0, UArg arg1){ // manage input

    float prev_adc = 0;
    while(1){ // infinite loop in all tasks unless the intend to terminate
        Event_pend(evtHandle0,
                   Event_Id_00, // andMask
                   Event_Id_NONE, // orMask
                   BIOS_WAIT_FOREVER);

        raw_adc = ADC14->MEM[3]; // store val from mem3 to adc_Ambient

        if((prev_adc < (raw_adc-5)) || (prev_adc > (raw_adc+5))){
                    prev_adc = raw_adc;
                    LCD_goto_xy(2,4);
                    LCD_print_udec5(prev_adc);
        }
        else {

            Event_post(evtHandle1, Event_Id_01);
        }

    }
}
/*
 *  ======== task1 =======
 */
void task1(UArg arg0 , UArg arg1){ // manage LCD
    while(1){ // infinite loop in all tasks unless they intend to terminate
        Event_pend(evtHandle1,
                   Event_Id_01, // andMask
                   Event_Id_NONE, // orMask
                   BIOS_WAIT_FOREVER);

        LCD_goto_xy(2,2);
        raw_adc = ADC14->MEM[3];
        LCD_print_udec5(raw_adc);

        LCD_goto_xy(6,0);
        Fahrenheit =(50*(raw_adc/1024))+40;
        LCD_print_udec3(Fahrenheit);

    }
}
