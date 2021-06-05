/*\\******************************************************************************\\
\\    AVR Bluetooth based Over-The-Air programming (OTA) bootloader. This         \\
\\    bootloader is capable of updating application code, debugging as well as    \\
\\    establising communication based control to the app code.                    \\                                                    \\
\\                                                                                \\
\\                   **********FOR ATMEGA328p ONLY**********                      \\
\\    Developed by:                                                               \\
\\    MD. Faridul Islam                                                           \\
\\    Department of Mechanical Engineering, RUET.                                 \\
\\                                                                                \\
\\                                                                                \\
\\                                                                                \\
\\                                                                                \\
\\    ************************************************************************    \\
\\    ************************* PERIPHERALS USED *****************************    \\
\\    ************************************************************************    \\
\\                                                                                \\
\\    +----------------------------------------------------------------------+    \\
\\    |  UART   |   ADC   |    TIMER1    |    EEPROM   |   WATCHDOG TIMER    |    \\
\\    +----------------------------------------------------------------------+    \\
\\                                                                                \\
\\    +----------------------------------------------------------------------+    \\
\\    |    | BAUD rate     : 19200                                           |    \\
\\    |    | Data config   : 8 bits, No pairity, 1 stop bits (8N1)           |    \\
\\    |    | Speed         : Normal (double speed disabled U2X0=0)           |    \\                                                                     
\\    |    | Mode          : Polling (Both RX & TX)                          |    \\
\\    |    | Purpose       : Download program OTA, communication             |    \\
\\    |UART|                                                                 |    \\
\\    |    | Customization : After entering bootloader, this peripheral is   |    \\
\\    |    |                 auto-configured and when exiting bootloader,    |    \\
\\    |    |                 all registers are cleared same as it is after   |    \\
\\    |    |                 power on. So, user has full access to this      |    \\
\\    |    |                 peripheral in application code.                 |    \\
\\    |----+-----------------------------------------------------------------|    \\
\\    |    | Refence       : Internal 1.1V                                   |    \\
\\    |    | Channels used : ADC6 & ADC7                                     |    \\
\\    |    | Prescaler     : 128                                             |    \\
\\    |    | Resolution    : 10 bits                                         |    \\
\\    |    | Mode          : Polling (single conversion)                     |    \\
\\    |    | Purpose       : Voltage measurement (ADC6-Supply, ADC7-V_CPU)   |    \\
\\    |ADC |                                                                 |    \\
\\    |    | Customization : After entering bootloader, this peripheral is   |    \\
\\    |    |                 auto-configured and when exiting bootloader,    |    \\
\\    |    |                 all registers are cleared same as it is after   |    \\
\\    |    |                 power on. So, user has full access to this      |    \\
\\    |    |                 peripheral except ADC6-ADC7 in application code |    \\
\\    |----+-----------------------------------------------------------------|    \\
\\    |    | Prescaler     : 1024                                            |    \\
\\    |    | Mode          : General (No PWM or Interrupt)                   |    \\
\\    |    | Purpose       : Timeout management                              |    \\
\\    |    |                                                                 |    \\
\\    |TIM1| Customization : After entering bootloader, this peripheral is   |    \\
\\    |    |                 auto-configured and when exiting bootloader,    |    \\
\\    |    |                 all registers are cleared same as it is after   |    \\
\\    |    |                 power on. So, user has full access to this      |    \\
\\    |    |                 peripheral in application code .                |    \\
\\    |----+-----------------------------------------------------------------|    \\
\\    |    | ADRESS RANGE  : 980 - 1023                                      |    \\
\\    |    | AUTHOR INFO   : 980 - 982                                       |    \\
\\    |    | VERSION NO    : 983 - 986                                       |    \\
\\    |    | DATE OF PRGM  : 987 - 994                                       |    \\
\\    |EEP | TIME OF PRGM  : 995 - 1002                                      |    \\
\\    |ROM | RESERVED      : 1003- 1022 (For classified purpose only)        |    \\
\\    |    | ADC CAL       : 1023- 1023 (ADC Calibration value)              |    \\
\\    |    | Purpose       : Store info (Stored information in ASCII code)   |    \\
\\    |    |                                                                 |    \\
\\    |    | Customization : Do not use for storing data in 980-1023 (Read-  |    \\
\\    |    |                 only) range. Rest of the memory can be used.    |    \\
\\    |----+-----------------------------------------------------------------|    \\
\\    |    | Cycles to RST : 2048 WDT clock                                  |    \\
\\    |    | RST delay     : 16 ms                                           |    \\
\\    |    | Mode          : Reset only (No interrupt)                       |    \\
\\    |    | Purpose       : Force CPU to enter bootloader mode              |    \\
\\    |    |                                                                 |    \\
\\    |WDT | Customization : After entering bootloader, this peripheral is   |    \\
\\    |    |                 auto-configured and when exiting bootloader,    |    \\
\\    |    |                 all registers are cleared same as it is after   |    \\
\\    |    |                 power on. So, user has full access to this      |    \\
\\    |    |                 peripheral in application code .                |    \\
\\    +----------------------------------------------------------------------+    \\
\\                                                                                \\
\\    ************************************************************************    \\
\\    **************************  FEATURES  **********************************    \\
\\    ************************************************************************    \\
\\                                                                                \\
\\    1. Fully OTA (over The Air/ Wireless) Programming.                          \\
\\    2. Remote firmware upgrade without touching the host (i.e. No manual reset) \\
\\    3. Secure Program download (128 bit encryption key, 7 bit checksum)         \\
\\    4. Timeout management (After start-up and while receiving data)             \\
\\    5. Prevents broken firmware (For checksum error or broken firmware, the     \\
\\       bootloader erases the whole flash section where broken data was written, \\
\\       allowing bootloader to write flash again without stopping operation)     \\
\\    6. Access Info (While writing firmware, prints author info, last            \\
\\       programming date, last programming time, bootloader version, BAUD rate)  \\
\\    7. Monitoring (While writing firmware, automatically measures supply        \\
\\       voltage, regulated volatge/VCC of the CPU, CPU temperature, warns user   \\
\\       if VCC is below 3.00v, dynamic programming progress bar etc.)            \\
\\    8. ADC calibration (Easy to calibrate adc, just upload calibration program  \\
\\       & wait a few seconds, adc calibrated and value stored in EEPROM)         \\
\\    9. Automatic Bluetooth address find out (Upload BT addr program and follow  \\
\\       the process.)                                                            \\
\\                                                                                \\
\\                                                                                \\
\\    ************************************************************************    \\
\\    *************************** CODE EXAMPLE *******************************    \\
\\    ************************************************************************    \\
\\                                                                                \\
\\    #include "OTA_BOOT.h"                                                       \\
\\                                                                                \\
\\    int main(){                                                                 \\
\\                                                                                \\
\\    UART_init(19200);                                                           \\
\\    Boot_unlock_handler();                                                      \\
\\    Page_write_handler();                                                       \\
\\                                                                                \\
\\    }                                                                           \\
\\                                                                                \\
\\******************************************************************************\\*/



/*INCLUDE LIBRARIES*/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <string.h>

//-------------------------------------------------------------------------------------
//DEFINE    NAME                        VALUE                   COMMENTS              |
//-------------------------------------------------------------------------------------

#define     TIMEOUT_MS                  1200                    /*USER CONFIGURABLE*/
#define     PROCESSOR_RX_WAIT           (8*TIMEOUT_MS)          /*DO NOT CHANGE*/


/*LED config*/
#define     LED_DDR                     DDRD                    /*USER CONFIGURABLE*/
#define     LED_PORT                    PORTD                   /*USER CONFIGURABLE*/
#define     LED_PIN                     2                       /*USER CONFIGURABLE*/


/*Info region start*/
#define     APP_INFO_LENGTH             24                      /*DO NOT CHANGE*/
#define     APP_INFO_STRT_ADDRESS       980                     /*DO NOT CHANGE*/
#define     BOOT_VERSION_MAJOR          4                       /*USER CONFIGURABLE*/
#define     BOOT_VERSION_MINOR          35                      /*USER CONFIGURABLE*/
#define     BOOT_DAY_CREATED            18                      /*USER CONFIGURABLE*/
#define     BOOT_MONTH_CREATED          7                       /*USER CONFIGURABLE*/
#define     BOOT_YEAR_CREATED           19                      /*USER CONFIGURABLE*/
#define     BOOT_BAUD_RATE              19200                   /*DO NOT CHANGE*/
/*Info region end*/     



/*BOOT config start*/
#define     BOOT_UNLOCK_KEY             24 /*EXCEPTION*/        /*DO NOT CHANGE*/
#define     BOOT_RETRY                  25 /*EXCEPTION*/        /*DO NOT CHANGE*/
#define     BOOT_ENTER_KEY              26                      /*DO NOT CHANGE*/
#define     BOOT_READY_FOR_ENCRYPT_KEY  27                      /*DO NOT CHANGE*/
#define     BOOT_READY_FOR_DATA         28                      /*DO NOT CHANGE*/
#define     BOOT_PAGE_STRT_ADDR         224                     /*DO NOT CHANGE*/
#define     BOOT_PAGE_SIZE_BYTES        128                     /*DO NOT CHANGE*/
/*BOOT config end*/


/*LED functions start*/
#define     LED_on()                    {LED_DDR|=(1<<LED_PIN);  LED_PORT|=(1<<LED_PIN);}
#define     LED_off()                   {LED_PORT&=~(1<<LED_PIN);LED_DDR&=~(1<<LED_PIN);}
#define     JUMP_APP()                  {asm("jmp 0x0000");}    /*DO NOT CHANGE*/
/*LED functions end*/

/*ADC measurement start*/
#define     ADC_SUPPLY                  7                       /*ACCORDING TO HARDWARE*/           
#define     ADC_VCC                     6                       /*ACCORDING TO HARDWARE*/ 
#define     ADC_TEMP                    8
int      adc_calib_status=0;
int      adc_constant=167;
uint32_t adc_val=0;
uint32_t adc_supply=0;
uint32_t adc_vcc=0;
uint32_t adc_temp=0;
/*ADC measurement end*/


uint8_t  ENCRYPT_KEY[16]="0123456789ABCDEF";
uint8_t  ENCRYPT_KEY_BUFFER[32];
uint8_t  LAST_PROGRAMMED_DATE[8];
uint8_t  LAST_PROGRAMMED_TIME[8];

uint8_t  BOOT_buffer[BOOT_PAGE_SIZE_BYTES];
uint8_t  app_info[APP_INFO_LENGTH];
uint8_t  chip_erase_allowed=0,last_programmed_info_available=1,adc_cal_available=1;
uint32_t data_sum=0,adc_sum=0,adc_avg=0;



void WDT_disable(void){
cli();
MCUSR=0;
WDTCSR|=(1<<WDCE)|(1<<WDE);
WDTCSR=0;
}

void WDT_reset(void){

UCSR0B=0x00;
UCSR0C=0x00;
UBRR0H=0x00;
UBRR0L=0x00;
TCCR1B=0x00;
ADCSRA=0x00;
ADMUX =0x00;
TCNT1=0;

WDTCSR=(1<<WDCE)|(1<<WDE);
WDTCSR=(1<<WDE);
while(1);
}

void CLEAR_peripherals(void){
UBRR0H=0x00;
UBRR0L=0x00;
UCSR0B=0x00;
UCSR0C=0x00;
TCCR1B=0x00;
ADCSRA=0x00;
ADMUX =0x00;
TCNT1=0;
}

void TIM1_init(void){
TCCR1B|=(1<<CS10)|(1<<CS12);
TCNT1=0;
}

uint32_t ADC_read(uint8_t channel){
ADMUX  = 0xC0;
ADMUX |= channel;
ADCSRA=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADEN);
_delay_ms(10);
ADCSRA|=(1<<ADSC);
while((ADCSRA & (1<<ADIF))==0);
uint16_t adc_val=ADCW;
ADCSRA|=(1<<ADIF);
return adc_val;
}


void UART_init(uint32_t BAUD){
WDT_disable();
uint16_t UBRR_VAL=(((F_CPU/16)/BAUD)-1);
UBRR0H=UBRR_VAL>>8;
UBRR0L=UBRR_VAL;
UCSR0B=(1<<RXEN0)|(1<<TXEN0);
UCSR0C=(1<<UCSZ00)|(1<<UCSZ01);
TIM1_init();
_delay_ms(100);
}


void UART_tx_single(unsigned char data){
while((UCSR0A & (1<<UDRE0))==0);
UDR0=data;
}

void UART_tx(char *c){
for(uint8_t i=0;i<strlen(c);i++){
   UART_tx_single(c[i]);
   }
}


void boot_program_page (uint32_t page, uint8_t *buf){

    uint16_t i;
    uint8_t sreg;
    sreg = SREG;
    cli();
    eeprom_busy_wait ();
    boot_page_erase (page);
    boot_spm_busy_wait (); 
	
    for (i=0; i<SPM_PAGESIZE; i+=2)
    {
        uint16_t w = *buf++;
        w += (*buf++) << 8;
        boot_page_fill (page + i, w);
    }
	
    boot_page_write (page); 
    boot_spm_busy_wait();   
    boot_rww_enable ();
    SREG = sreg;
}

void print_percent(uint8_t x){
UART_tx_single('|');
for(uint8_t i=0;i<x;i++){
   UART_tx_single('=');
   }
UART_tx_single('|');
}

void chip_erase(uint8_t strt, uint8_t stop){
uint8_t erase_buf[BOOT_PAGE_SIZE_BYTES];
for(uint8_t i=0;i<BOOT_PAGE_SIZE_BYTES;i++){
    erase_buf[i]=0xFF;
	}
for(uint8_t page_erase=strt;page_erase<stop;page_erase++){
    boot_program_page((page_erase*BOOT_PAGE_SIZE_BYTES),erase_buf);
	}
}

unsigned char UART_rx(void){
TCNT1=0;
while((UCSR0A & (1<<RXC0))==0){
                                  if(TCNT1>PROCESSOR_RX_WAIT){
								        if(chip_erase_allowed==1)
										    {
										     chip_erase(0,BOOT_PAGE_STRT_ADDR);
								             UART_tx("\nData incomplete!\n");
										     UART_tx("Chip Erased!\n");
											}
								        LED_off();
										CLEAR_peripherals();
										JUMP_APP();
										}
								  
								}
return UDR0;
}

void EEPROM_write(uint16_t EE_address, unsigned char EE_data){
while(EECR & (1<<EEPE));
//while(SPMCR & (1<<SPMEN));
EEAR=EE_address;
EEDR=EE_data;
EECR=(1<<EEMPE);
EECR|=(1<<EEPE);
}

unsigned char EEPROM_read(uint16_t EE_address){
while(EECR & (1<<EEPE));
EEAR=EE_address;
EECR|=(1<<EERE);
return EEDR;
}


void LED_blink(uint8_t x){
for(uint8_t i=0;i<x;i++){
   LED_on();
   _delay_ms(20);
   LED_off();
   _delay_ms(20);
   }
}


void UART_print_dec(uint16_t x){
if(x!=0){
  uint8_t i=0;
  uint16_t y=x;
  while(y!=0){ y=y/10; i++; }
  uint8_t num[i];
  uint16_t z=x;
  for(int j=(i-1);j>-1;j--) { num[j]=(z%10)+48; z=z/10;}
  for(uint8_t k=0;k<i;k++)  { UART_tx_single(num[k]); }
  }
else{UART_tx_single('0');}
}

void UART_num(uint16_t mv){
UART_print_dec(mv/100);
UART_tx_single('.');
uint8_t d2=mv%100;
if(d2<10){UART_tx_single('0');}
UART_print_dec(d2);
}


void Boot_unlock_handler(void){
LED_on();
chip_erase_allowed=0;
UART_tx_single(BOOT_READY_FOR_ENCRYPT_KEY);
while(UART_rx()!=BOOT_UNLOCK_KEY);

int retry=0;
RETRY_ENCRYPTION:

for(uint8_t i=0;i<32;i++){ENCRYPT_KEY_BUFFER[i]=UART_rx();}
retry++;
if(retry>5)              {
                            LED_off();
							UART_tx("\nAuthentication Error!\n");
							WDT_reset();
							
							}
for(uint8_t i=0;i<16;i++){
                            if(ENCRYPT_KEY[i]!=ENCRYPT_KEY_BUFFER[i])
							   {
							      UART_tx_single(BOOT_RETRY);
							      goto RETRY_ENCRYPTION;
						        }
							}
for(uint8_t i=16;i<32;i++){
                            if(ENCRYPT_KEY_BUFFER[16]!=ENCRYPT_KEY_BUFFER[i])
							   {
							      UART_tx_single(BOOT_RETRY);
							      goto RETRY_ENCRYPTION;
							    }
							}
}



void Page_write_handler(void){

adc_constant=EEPROM_read(1023);
if(adc_constant==0xFF){adc_cal_available=0;}
for(uint8_t i=0;i<8;i++){ADC_read(ADC_SUPPLY);}
for(uint8_t i=0;i<16;i++){adc_sum+=ADC_read(ADC_SUPPLY);}
adc_supply=((adc_sum*300)/adc_constant)/16;
adc_sum=0;

for(uint8_t i=0;i<8;i++){ADC_read(ADC_VCC);}
for(uint8_t i=0;i<16;i++){adc_sum+=ADC_read(ADC_VCC);}
adc_vcc=((adc_sum*300)/adc_constant)/16;
adc_sum=0;
ADC_read(ADC_TEMP);
ADC_read(ADC_TEMP);
adc_temp  =(((ADC_read(ADC_TEMP)*782)/1000)-250);
for(uint8_t i=0;i<8;i++){
                           LAST_PROGRAMMED_DATE[i]=EEPROM_read(APP_INFO_STRT_ADDRESS+7+i);
						   if(LAST_PROGRAMMED_DATE[i]==0xFF){last_programmed_info_available=0;}
						  }
for(uint8_t i=0;i<8;i++){
                           LAST_PROGRAMMED_TIME[i]=EEPROM_read(APP_INFO_STRT_ADDRESS+15+i);
						   if(LAST_PROGRAMMED_TIME[i]==0xFF){last_programmed_info_available=0;}
						 }

chip_erase_allowed=1;
if     (ENCRYPT_KEY_BUFFER[16]>(BOOT_PAGE_STRT_ADDR-1))
                                                       {
													     UART_tx("\nMax data size crossed!\n");
														 CLEAR_peripherals();
														 JUMP_APP();
														}
else if(ENCRYPT_KEY_BUFFER[16]==0)                    
                                   {
								     chip_erase(0,BOOT_PAGE_STRT_ADDR);
									 UART_tx("\nChip erase successful!\n");
									 WDT_reset();
									}


UART_tx("\n\n\n");
UART_tx("----------------------------------\n");
UART_tx("        AVR OTA BOOTLOADER\n");
UART_tx("----------------------------------\n");
UART_tx("Author               : MFI_132097\n");

UART_tx("Bootloader created   : ");
UART_print_dec(BOOT_DAY_CREATED);
UART_tx_single('/');
UART_print_dec(BOOT_MONTH_CREATED);
UART_tx_single('/');
UART_print_dec(BOOT_YEAR_CREATED);
UART_tx_single('\n');

UART_tx("Bootloader version   : ");
UART_print_dec(BOOT_VERSION_MAJOR);
UART_tx_single('.');
UART_print_dec(BOOT_VERSION_MINOR);
UART_tx("\n");

UART_tx("UART BAUD rate       : ");
UART_print_dec(BOOT_BAUD_RATE);
UART_tx(" bps\n");

/*Voltage measurement start*/
UART_tx("Supply voltage       : ");
if(adc_cal_available==0){UART_tx("N/A\n");}
else{ UART_num(adc_supply); UART_tx(" V\n");}

UART_tx("CPU voltage          : ");
if(adc_cal_available==0){UART_tx("N/A\n");}
else{ UART_num(adc_vcc); UART_tx(" V\n");}

UART_tx("CPU temperature      : ");
UART_print_dec(adc_temp);
UART_tx("'C\n");
/*Voltage measurement end*/


UART_tx("Last programmed      : ");
if(last_programmed_info_available==0){UART_tx("N/A");}
else{for(uint8_t i=0;i<8;i++){UART_tx_single(LAST_PROGRAMMED_DATE[i]);}}
UART_tx_single('\n');
UART_tx("                       ");
if(last_programmed_info_available==0){UART_tx("N/A");}
else{for(uint8_t i=0;i<8;i++){UART_tx_single(LAST_PROGRAMMED_TIME[i]);}}
UART_tx_single('\n');

UART_tx("Currently writing    : ");
UART_print_dec(ENCRYPT_KEY_BUFFER[16]);
UART_tx(" pages\n");
UART_tx("                       ");
UART_print_dec(ENCRYPT_KEY_BUFFER[16]*128);
UART_tx(" bytes\n");


if(adc_vcc<300)
                {
				 
				 UART_tx("\nCPU voltage is below safe zone!\n");
				 UART_tx("Not recommended to enter DFU!\n\n");
				 //WDT_reset();
				 }
				 
UART_tx("Writing new firmware image...\n");
print_percent(ENCRYPT_KEY_BUFFER[16]);
UART_tx("\n|");
data_sum=0;


for(uint8_t i=0;i<16;i++){ENCRYPT_KEY_BUFFER[i]=0;}

UART_tx_single(BOOT_READY_FOR_DATA);


for(uint8_t page=0;page<ENCRYPT_KEY_BUFFER[16];page++){
        for(uint16_t i=0;i<BOOT_PAGE_SIZE_BYTES;i++){
		       BOOT_buffer[i]=UART_rx();
			   data_sum+=BOOT_buffer[i];
			   }
		boot_program_page((page*BOOT_PAGE_SIZE_BYTES),BOOT_buffer);
		UART_tx_single('=');
	}
	
for(uint8_t i=0;i<APP_INFO_LENGTH;i++){app_info[i]=UART_rx();}
for(uint8_t i=0;i<16;i++){ENCRYPT_KEY_BUFFER[i]=UART_rx();}
UART_tx("|\n");

for(uint8_t i=0;i<16;i++){
                            if(ENCRYPT_KEY[i]!=ENCRYPT_KEY_BUFFER[i])
							   {
							      chip_erase(0,BOOT_PAGE_STRT_ADDR);
                                  UART_tx("Checksum Error!\n");
	                              UART_tx("Firmware write failed!\n");
								  CLEAR_peripherals();
								  JUMP_APP();
						        }
							}
chip_erase(ENCRYPT_KEY_BUFFER[16],BOOT_PAGE_STRT_ADDR);
for(uint8_t i=0;i<(APP_INFO_LENGTH-1);i++){EEPROM_write(APP_INFO_STRT_ADDRESS+i,app_info[i]);}
UART_tx_single(data_sum & 0x7F);
UART_tx("\nSuccess!\n");
UART_tx("Starting Application\n");
UART_tx("----------------------------------\n\n\n");


LED_blink(20);
CLEAR_peripherals();
JUMP_APP();

}

