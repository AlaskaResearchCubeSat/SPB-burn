#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>
#include <ctl.h>
#include <math.h>
#include <commandlib.h>
#include "terminal.h"
#include "burn.h"


int burnCmd(char **argv,unsigned short argc){
    float burn_time=1,wait_time=0;
    unsigned long burn_delay,wait_delay;
    char *end;
    if(argc!=0){
        if(argc>2){
            printf("Error : too many arguments\r\n");
            return -1;
        }
        if(argc>=2){
            wait_time=strtof(argv[2],&end);
            //check result
            if(!isfinite(wait_time)){
                printf("Error : failed to read time \"%s\" %f returned\r\n",argv[2],wait_time);
                return -2;
            }
            //check for extra chars
            if(*end!=NULL){
                printf("Error : unknown suffix \"%s\" while parsing \"%s\" \r\n",end,argv[2]);
                return -3;
            }
        }
        burn_time=strtof(argv[1],&end);
        //check result
        if(!isfinite(burn_time)){
            printf("Error : failed to read time \"%s\" %f returned\r\n",argv[1],burn_time);
            return -2;
        }
        //check for extra chars
        if(*end!=NULL){
            printf("Error : unknown suffix \"%s\" while parsing \"%s\" \r\n",end,argv[1]);
            return -3;
        }
    }
    //check if we need to wait
    if(wait_time>10*1/1024.0){
        //calculate delay
        wait_delay=(wait_time*1024)+0.5;
        //recalculate time
        wait_time=wait_delay/1024;
        //print message with time
        printf("Waiting %.0fs before burn\r\n",wait_time);
        //wait
        ctl_timeout_wait(ctl_get_current_time()+wait_delay);
    }
    //calculate delay in clocks
    burn_delay=(burn_time*1024)+0.5;
    //check if delay is too short
    if(burn_delay<3){
        burn_delay=3;
    }
    //recalculate time
    burn_time=burn_delay/1024.0;
    //print out delay time
    printf("Activating burn circuit for %.1f secconds\r\n",burn_time);
    //turn on resistor
    burn_on();
    //delay for specified time
    ctl_timeout_wait(ctl_get_current_time()+burn_delay);
    //turn off resistor
    burn_off();
    //print completion message
    printf("Burn Activation complete!\r\n");
    return 0;
}
  
  
//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]\r\n\t""get a list of commands or help on a spesific command.",helpCmd},
                         I2C_COMMANDS,CTL_COMMANDS,
                         {"burn","[time [delay]]""\r\n\t""trigger the burn circuit to deploy the antenna for [time] in secconds",burnCmd},
                         //end of list
                         {NULL,NULL,NULL}};
