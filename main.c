#include <msp430.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <ctl.h>
#include <UCA1_uart.h>
#include <terminal.h>
#include "timer.h"
#include "burn.h"


//make printf send over UCA1
int __putchar(int ch){
  return UCA1_TxChar(ch);
}

//make getchar get chars from UART
int __getchar(void){
    return UCA1_Getc();
}

//task structure for idle task
CTL_TASK_t idle_task;

CTL_TASK_t tasks[1];

//stack for task
unsigned stack1[1+400+1];

 //initialize the MSP430 Clocks
void initCLK(void){
  //set XT1 load caps, do this first so XT1 starts up sooner
  BCSCTL3=XCAP_0;
  //stop watchdog
  WDTCTL=WDTPW|WDTHOLD;

  //setup clocks

  //set DCO to 16MHz from calibrated values in flash
  //TODO: check to see that values are valid before using
  DCOCTL=0;
  BCSCTL1=CALBC1_16MHZ;
  DCOCTL=CALDCO_16MHZ;

  //Source Mclk and SMclk from DCO (default)
  BCSCTL2=SELM_0|DIVM_0|DIVS_0;
  
  //also initialize timing generator for flash memory
  //FCTL2=FWKEY|FSSEL_2|33;

  //set port 5 to output clocks
  //P5DIR=BIT4|BIT5|BIT6;
  //P5SEL=BIT4|BIT5|BIT6;
  P5DIR=0;
  P5SEL=0;
  
  //Maybe wait for LFXT to startup
}
  

void main(void){
  //setup clocks
  initCLK();
  //setup timerA
  init_timerA();
  //set timer to increment by 1
  ctl_time_increment=1; 
  
  //setup system specific peripherals
  
  //setup UCA1 UART
  UCA1_init_UART();

  //set baud rate
  UCA1_BR57600();

  // Set up Port 7 for LED
  P7OUT=0x80;
  P7DIR=0xFF;

  burn_init();

  //set unused pins as inputs
  P6REN&=~BIT6;
  P6DIR&=~BIT6;
  P8REN&=~BIT0;
  P8DIR&=~BIT0;

  //initialize tasking
  ctl_task_init(&idle_task, 255, "idle");  

  //start timerA
  start_timerA();

  //initialize stack
  memset(stack1,0xcd,sizeof(stack1));  // write known values into the stack
  stack1[0]=stack1[sizeof(stack1)/sizeof(stack1[0])-1]=0xfeed; // put marker values at the words before/after the stack


  //create tasks
  ctl_task_run(&tasks[0],2,terminal,"SPB Test Program Ready","terminal",sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  
  // drop to lowest priority to start created tasks running.
  ctl_task_set_priority(&idle_task,0);

  for(;;){
    LPM0;
  }

}


//==============[task library error function]==============

//something went seriously wrong
//perhaps should try to recover/log error
void ctl_handle_error(CTL_ERROR_CODE_t e){
  switch(e){
    case CTL_ERROR_NO_TASKS_TO_RUN: 
      __no_operation();
      //puts("Error: No Tasks to Run\r");
    break;
    case CTL_UNSUPPORTED_CALL_FROM_ISR: 
      __no_operation();
      //puts("Error: Wait called from ISR\r");
    break;
    case CTL_UNSPECIFIED_ERROR:
      __no_operation();
      //puts("Error: Unspesified Error\r");
    break;
    default:
      __no_operation();
      //printf("Error: Unknown error code %i\r\n",e);
  }
  //something went wrong, reset
  WDTCTL=0;
}
