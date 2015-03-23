#include <msp430.h>
#include "burn.h"



void burn_init(void){
  //setup burn pins
  P8OUT&=~BURN_PINS_ALL;
  P8SEL&=~BURN_PINS_ALL;
  P8REN&=~BURN_PINS_ALL;
  P8DIR|= BURN_PINS_ALL;
  //turn off LEDs
  P7OUT&=~(BURN_PINS_ALL<<BURN_LED_SHIFT);
}

void burn_on(unsigned char burn){
    //turn on LEDs
    P7OUT|=burn<<BURN_LED_SHIFT;
    //turn on resistors
    P6OUT|=burn;
}

void burn_off(void){
    //turn off resistor
    P6OUT&=~BURN_PINS_ALL;
    //turn off LEDs
    P7OUT&=~(BURN_PINS_ALL<<BURN_LED_SHIFT);
}
