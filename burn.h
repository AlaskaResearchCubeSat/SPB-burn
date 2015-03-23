#ifndef __BURN_H
#define __BURN_H


void burn_init(void);
void burn_on(unsigned char burn);
void burn_off(void);

#define BURN_PIN_0      (BIT0)
#define BURN_PIN_1      (BIT1)
#define BURN_PIN_2      (BIT2)

#define BURN_LED_SHIFT  (4)

#define BURN_PINS_ALL (BURN_PIN_0|BURN_PIN_1|BURN_PIN_2)

#endif
