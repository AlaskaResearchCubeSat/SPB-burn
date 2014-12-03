#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>
#include <ctl.h>
#include <math.h>
#include <commandlib.h>
#include "Mag.h"
#include "terminal.h"
#include "i2c.h"
#include "UCA1_uart.h"
#include "temp.h"
#include "LTC24XX.h"

  //Defines for set reset pin
  #define MAG_SR_PIN         BIT1
  #define MAG_SR_OUT         P8OUT
  #define MAG_SR_DIR         P8DIR
  #define MAG_SR_SEL         P8SEL
  #define MAG_SR_REN         P8REN


#define MAX_NUM_ADDR  10

//read values from the temp sensor optionally takes the I2C address to read from
int tempCmd(char **argv,unsigned short argc){
  unsigned char addr[MAX_NUM_ADDR]={X_MINUS_ADDR};
  short error=0,success=0;
  int numAddr=1;
  unsigned char aptr,temp[2];
  int i,k;
  //get addresses to transmit to
  if(argc>0){
    if(argc>(sizeof(addr)/sizeof(addr[0]))){
      printf("Error: only %i addresses allowed\r\n",MAX_NUM_ADDR);
      return -1;
    }
    if(argc==1 && !strcasecmp("all",argv[1])){
      //add all named addresses to list
      for(i=0;tempAddrSym[i].name!=NULL;i++){
        addr[i]=tempAddrSym[i].addr;
      }
      numAddr=i;
    }else{
      numAddr=argc;
      for(k=0;k<argc;k++){
          //get hex value form string
          addr[k]=getI2C_addr(argv[k+1],0,tempAddrSym);
          //check if address is 7 bits
          if(addr[k]==0xFF){
            return -2;
          }
      }
    }
  }
  
  printf("Reading Temperature press ^C to stop\r\n");
  //set address pointer to temperature register
  aptr=TEMP_VAL;
  for(i=0;i<numAddr;i++){
    if(1!=i2c_tx(addr[i],&aptr,1)){
      //error sending data
      P7OUT|=BIT2;
      P7OUT&=~BIT1;
      printf("Errror Writing Address Pointer to address %X\r\n",addr[i]);
    }
  }
  //run until ^C key is pressed
  while(UCA1_CheckKey()!=0x03){
    //clear error an sucess flags
    error=0;
    success=0;
    for(i=0;i<numAddr;i++){
      if(i!=0){
        printf("\t");
      }
      //read temperature value
      if(2==i2c_rx(addr[i],temp,2)){
        printf("%i.%02u",(short)((char)temp[0]),25*(temp[1]>>6));
        success=1;
      }else{
        //error receiving data
        error=1;
        //display error
        printf("NaN");
        //set address pointer to temperature register
        aptr=TEMP_VAL;
        i2c_tx(addr[i],&aptr,1);
      }
    }
    printf("\r\n");
    //check for an error
    if(error){
      P7OUT|=BIT2;
    }else{
      P7OUT&=~BIT2;
    }
    //check if a value was read
    if(success){
      P7OUT|=BIT1;
    }else{
      P7OUT&=~BIT1;
    }
    //toggle LED
    P7OUT^=BIT0;
    ctl_timeout_wait(ctl_get_current_time()+1024);
  }
  //clear LED's
  P7OUT&=~(BIT0|BIT1|BIT2);
}
  
  unsigned short sample[2];
  CTL_EVENT_SET_t ADC_events;
  //ADC ISR called after both channels have been samples
void adc_int(void) __ctl_interrupt[ADC12_VECTOR]{
  switch(ADC12IV){
    //not used but just in case
    case ADC12IV_ADC12IFG0:
      sample[0]=ADC12MEM0;//dummy read
    break;
    case ADC12IV_ADC12IFG1:
      //read samples
      sample[0]=ADC12MEM0;
      sample[1]=ADC12MEM1;
      //set events in the events set for ADC
      ctl_events_set_clear(&ADC_events,BIT0,0);
    break;

  }
}
  
//gain of magnetomitor amplifier
//#define AMP_GAIN    (2.49e6/5.1e3)    // V/V0
#define AMP_GAIN    (1)    // V/V
//#define AMP_GAIN    (5.11e6/5.1e3)    // V/V
//sensitivity of magnetomitor
#define MAG_SENS    (1e-3)            // mV/V/Gauss

//convert ADC value to voltage
float ADCtoV(long adc){
  //TODO: maybe allow other references
  return ((float)adc)*3.3/(2*65535.0);
}

//compute ADC value to magnetic field value in gauss
float ADCtoGauss(long adc){
  return  ADCtoV(adc)/(AMP_GAIN*MAG_SENS);
}


//convert returned data from 16bit LTC24xx ADC into a signed long integer
long adc16Val(unsigned char *dat){
  long val;
  short sig,msb;
  //extract magnitude bits from data
  //val=(((unsigned long)dat[0])<<(16-6))|(((unsigned long)dat[1])<<(8-6))|((unsigned long)dat[2]>>6);
  val=(((unsigned long)dat[0])<<16)|(((unsigned long)dat[1])<<8)|((unsigned long)dat[2]);
  val>>=6;
  //check sign bit
  sig=!!(val&(0x20000));
  //check MSB bit
  msb=!!(val&(0x10000));
  //remove MSB and sig bits
  val&=~0x30000;
  //check for negative values
  if(!sig){
    val|=0xFFFF0000;
  }
  //check for positive overflow
  if(msb && sig){
    return 65536;
  }
  //check for negative overflow
  if(!msb && !sig && val!=0){
    return -65536;
  }

  return val;
}

  #define MAG_A_CH        LTC24xx_CH_1
  #define MAG_B_CH        LTC24xx_CH_0

//the time that the ADC can next be sampled
CTL_TIME_t adc_ready_time=153;

unsigned short mag_ADC_gain=64;

//take a reading from the magnetomitor ADC
short single_sample(unsigned short addr,long *dest){
  unsigned char rxbuf[4],txbuf[4];
  int res;
  CTL_TIME_t ct;
  //get current time
  ct=ctl_get_current_time();
  //check if ADC is ready
  if((ct-adc_ready_time)<-3){
    ctl_timeout_wait(adc_ready_time);
  }
  //turn on LED while measuring
 // meas_LED_on();
 //generate set pulse
  MAG_SR_OUT|=MAG_SR_PIN;
  //delay for pulse
  ctl_timeout_wait(ctl_get_current_time()+2);
  
  //configure for first conversion convert in the A-axis
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_A_CH;
  #ifdef MAG_ADC_GAIN
    txbuf[1]=LTC24xx_EN2|LTC24xx_FA|MAG_ADC_GAIN;                   
  #else
    txbuf[1]=LTC24xx_EN2|LTC24xx_FA;
    switch(mag_ADC_gain){
      case 1:
        txbuf[1]|=LTC24xx_GAIN1;
      break;
      case 4:
        txbuf[1]|=LTC24xx_GAIN4;
      break;
      case 8:
        txbuf[1]|=LTC24xx_GAIN8;
      break;
      case 16:
        txbuf[1]|=LTC24xx_GAIN16;
      break;
      case 32:
        txbuf[1]|=LTC24xx_GAIN32;
      break;
      case 64:
        txbuf[1]|=LTC24xx_GAIN64;
      break;
      case 128:
        txbuf[1]|=LTC24xx_GAIN128;
      break;
      case 264:
        txbuf[1]|=LTC24xx_GAIN264;
      break;
      default:
        printf("Error : Invalid gain %i\r\n",mag_ADC_gain);
        //generate reset pulse
        MAG_SR_OUT&=~MAG_SR_PIN;
        return 1;
    }
  #endif  
  if((res=i2c_tx(addr,txbuf,2))<0){
    if(res==I2C_ERR_NACK){
      //perhaps a conversion is in progress, wait for it to complete
      ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
      //report a warning
      printf("Warning : Failed to setup sensor\r\n");
      //try sending again
      res=i2c_tx(addr,txbuf,2);
    }
    if(res<0){
      //report error
      printf("Error : Failed to setup sensor\r\n");
      //generate reset pulse
      MAG_SR_OUT&=~MAG_SR_PIN;
      //turn on error LED
      //sens_err_LED_on();
      //turn LED off, done measuring
     // meas_LED_off();
      //TODO : provide real error codes
      return 2;
    }
  }
  //wait for conversion to complete
  ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
  //config for next conversion in the B-axis
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_B_CH;
  //read in data and start next conversion
  if((res=i2c_txrx(addr,txbuf,1,rxbuf,3))<0){
    //report error
    printf("Error : failed to read sensor data\r\n");
    //generate reset pulse
    MAG_SR_OUT&=~MAG_SR_PIN;
    //turn on error LED
   // sens_err_LED_on();
    //turn LED off, done measuring
   // meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //save result
  dest[0]=adc16Val(rxbuf);
  //wait for conversion to complete
  ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
  //read in data
  if((res=i2c_rx(addr,rxbuf,3))<0){
    //report error
    printf("Error : failed to read sensor data\r\n");
    //generate reset pulse
    MAG_SR_OUT&=~MAG_SR_PIN;
    //turn on error LED
   // sens_err_LED_on();
    //turn LED off, done measuring
   // meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //save result
  dest[1]=adc16Val(rxbuf);
  //generate reset pulse
  MAG_SR_OUT&=~MAG_SR_PIN;
  //turn LED off, done measuring
 // meas_LED_off();
  //get time that ADC can next be read
  adc_ready_time=ctl_get_current_time()+153;
  //TODO: provide real return codes
  return 0;
}


int gain_Cmd(char **argv,unsigned short argc){
  #ifdef MAG_ADC_GAIN
    if(argc>0){
      printf("Error : gain has been hard coded %s takes no arguments\r\n",argv[0]);
      return 1;
    }
    switch(MAG_ADC_GAIN){
      case LTC24xx_GAIN1:
        printf("ADC gain = 1\r\n");
      break;
      case LTC24xx_GAIN4:
        printf("ADC gain = 4\r\n");
      break;
      case LTC24xx_GAIN8:
        printf("ADC gain = 8\r\n");
      break;
      case LTC24xx_GAIN16:
        printf("ADC gain = 16\r\n");
      break;
      case LTC24xx_GAIN32:
        printf("ADC gain = 32\r\n");
      break;
      case LTC24xx_GAIN64:
        printf("ADC gain = 64\r\n");
      break;
      case LTC24xx_GAIN128:
        printf("ADC gain = 128\r\n");
      break;
      case LTC24xx_GAIN264:
        printf("ADC gain = 264\r\n");
      break;
      default:
        printf("Error : unknown hardcoded gain\r\n");
      break;
    }
  #else
    int gain;
    if(argc>1){
      printf("Error : Too many Arguments\r\n");
      return 2;
    }
    if(argc==1){
      gain=atoi(argv[1]);
      switch(gain){
        case 1:
        case 4:
        case 8:
        case 16:
        case 32:
        case 64:
        case 128:
        case 264:
        break;
        default:
          printf("Error : %i is not a valid gain fo the LTC2487\r\n");
          return 3;
      }
      mag_ADC_gain=gain;
   }
   printf("ADC gain = %u\r\n",mag_ADC_gain);
  #endif  
}

//run magnetometer
//read data from the magnetometer and print to terminal
int magCmd(char **argv,unsigned short argc){
  signed short set[2],reset[2],os[2],val[2];
  unsigned short single=0,gauss=0,addr=0x14;
  unsigned char c,mag_addr=0x14;
  long result[2];
  float time=0;
  int i,res;

  //parse arguments
  for(i=1;i<=argc;i++){
    if(!strcmp("single",argv[i])){
      single=1;
    }else if(!strcmp("gauss",argv[i])){
      gauss=1;
    }else if((addr=getI2C_addr(argv[i],0,magAddrSym))!=0xFF){
      mag_addr=addr;
    }else{
      
      printf("Error Unknown argument \'%s\'.\r\n",argv[i]);
      return -1;
    }
  }
  //run until abort is detected
  do{
    res=single_sample(mag_addr,result);
    if(res!=0){
      printf("Error encountered. Aborting\r\n");
      break;
    }
    if(gauss){
      printf("%f %f\r\n",ADCtoGauss(result[0])/2,ADCtoGauss(result[1])/2);
    }else{
      printf("%li %li\r\n",result[0],result[1]);
    }
    c=UCA1_CheckKey();
  }while(!(c==0x03  || c=='Q' || c=='q' || single));
  return 0;
}


//read a value from the ADC
int adcCmd(char **argv,unsigned short argc){
  unsigned short addr=0x14;
  char *end;
  long val,gain=1;
  int temp=0;
  unsigned char buf[4];
  
  //setup tx buffer
  buf[0]=0xA0;                      //preamble requiored by ADC
  buf[1]=LTC24xx_EN2|LTC24xx_FA;    //setup normal speed 60Hz rejection 1x gain
  //lok at args for channel
  if(argc>=1){
    if(!strcmp(argv[1],"0")){
      //channel 0 differential
      buf[0]|=LTC24xx_CH_0;
    }else if(!strcmp(argv[1],"1")){
      //channel 1 differential
      buf[0]|=LTC24xx_CH_1;
    }else if(!strcmp(argv[1],"0s")){
      //channel 0 single ended
      buf[0]|=LTC24xx_SGL|LTC24xx_CH_0;
    }else if(!strcmp(argv[1],"1s")){
      buf[0]|=LTC24xx_SGL|LTC24xx_SIGN|LTC24xx_CH_0;
    }else if(!strcmp(argv[1],"2s")){
      buf[0]|=LTC24xx_SGL|LTC24xx_CH_1;
    }else if(!strcmp(argv[1],"3s")){
      buf[0]|=LTC24xx_SGL|LTC24xx_SIGN|LTC24xx_CH_1;
    }else if(!strcmp(argv[1],"temp")){
      //temperature channel
      buf[1]|=LTC24xx_IM;
      //temperature channel needs special conversion to celcius
      temp=1;
    }else{
      //bad argument print error and exit
      printf("Error : Unknown channel \'%s\'\r\n",argv[1]);
      return -1;
    }
    //check if gain also given
    if(argc>=2){
      gain=strtol(argv[2],&end,0);      
      if(*end!=0){
        printf("Error : unknown sufix \"%s\" at end of gain\r\n",end);
        return -2;
      }
      switch(gain){
        case 1:
          //default gain is 1 so do nothing
        break;
        case 4:
          buf[1]|=LTC24xx_GAIN4;
        break;
        case 8:
          buf[1]|=LTC24xx_GAIN8;
        break;
        case 16:
          buf[1]|=LTC24xx_GAIN16;
        break;
        case 32:
          buf[1]|=LTC24xx_GAIN32;
        break;
        case 64:
          buf[1]|=LTC24xx_GAIN64;
        break;
        case 128:
          buf[1]|=LTC24xx_GAIN128;
        break;
        case 264:
          buf[1]|=LTC24xx_GAIN264;
        break;
        default:
          printf("Error : unknown gain %li. Valid gains are 1, 4, 8, 16, 32, 64, 128 and 264.\r\n",gain);
        return -3;
      }      
      //check if two arguments are given
      if(argc>=3){
          if((addr=getI2C_addr(argv[3],0,magAddrSym))==0xFF){ 
          //error reading address prin error and return
          printf("Error : bad address \'%s\'\r\n",argv[2]);
          return -1;
        }
        //output address for confermation
        printf("using address 0x%X\r\n",addr);
      }
    }
  }else{
    //no arguments default to channel 0 differential
    buf[0]|=LTC24xx_CH_0;
  }
     
  //setup conversion
  if(i2c_tx(addr,buf,2)<0){
    printf("Error seting up ADC at file: %s:%i\r\n",__FILE__,__LINE__);
    return 1;
  }
    ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
  //read in data
  if(i2c_rx(addr,buf,3)<0){
    printf("Error reading conversion at file: %s:%i\r\n",__FILE__,__LINE__);
    return 2;
  }
  //print raw data
  printf("dat = 0x%02X%02X%02X\r\n",buf[0],buf[1],buf[2]);
  //convert result to convert
  val=adc16Val(buf);
  printf("counts = %li\r\n",val);
  //print result
  printf("Value = %f V\r\n",ADCtoV(val)/(float)gain);
  //if temperature print in proper units
  if(temp){
    //printf("Temperature = %f C\r\n",ADCtoV(val)/93.5e-6 - 273.15);
    printf("Temperature = %f C\r\n",val*3.3/12.25 - 273.15);
  }
  return 0;
}


const char *(addr_name_mag[])={"X+","X-","Y+","Y-","Z+","Z-"};
const char name_addrs_mag[]={MAG_X_PLUS_ADDR ,MAG_X_MINUS_ADDR,MAG_Y_PLUS_ADDR,MAG_Y_MINUS_ADDR,MAG_Z_PLUS_ADDR,MAG_Z_MINUS_ADDR};

//pulse set reset cmd
int SRcmd(char **argv,unsigned short argc){
int i;
  //run until ^C key is pressed
  while(UCA1_CheckKey()!=0x03){
  //print loop iteration 
  printf("pulse number %d\n\r\t",i);
  //increment i
  i++;
  //turn on LED for test
  P7OUT^=BIT2;
 //pulse S/R for a good mag reading
  P8OUT^=BIT1;
  // wait for set pulse
   ctl_timeout_wait(ctl_get_current_time()+100);    //wait about 100ms
   }
   // Turn off LED 
   P7OUT&=~BIT2;
   return 0;
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
                         {"temp"," [[addr1] [addr2] ... ]""\r\n\t""read temperature data from given addresses",tempCmd},
                         {"adc","[chan] [gain] [addr]""\r\n\t""Read a channel of the LTC2487 ADC.",adcCmd},
                         {"SR","C to exit cmd""\r\n\t""pulses set rest for magnetometer reading",SRcmd},
                         {"mag","qwerys magnetometer",magCmd},
                         {"gain","[gain]\r\n\t""get/set ADC gain for magnetometer",gain_Cmd},
                         {"burn","[time [delay]]""\r\n\t""trigger the burn circuit to deploy the antenna for [time] in secconds",burnCmd},
                         //end of list
                         {NULL,NULL,NULL}};
