#include <avr/io.h>
#include <util/delay.h>
#include "OTA_BOOT.h"



int main(void){

UART_init(19200);

Boot_unlock_handler();
  
Page_write_handler();

}