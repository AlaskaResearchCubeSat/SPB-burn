#include <msp430.h>
#include "burn.h"



void burn_init(void){
  //setup burn pin
  P6OUT&=~BIT7;
  P6SEL&=~BIT7;
  P6REN&=~BIT7;
  P6DIR|= BIT7;
}

void burn_on(void){
    //turn on LED
    P7OUT|=BIT4;
    //turn on resistor
    P6OUT|=BIT7;
}

void burn_off(void){
    //turn off resistor
    P6OUT&=~BIT7;
    //turn off LED
    P7OUT&=~BIT4;
}
