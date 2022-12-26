
/* Name: main.c
 * Author: <Raghavesh Viswanath>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file
#include "m_usb.h"

#define PRINTNUM(x) m_usb_tx_uint(x); m_usb_tx_char(10); m_usb_tx_char(13)
#define VAL1 340 
#define VAL2 11

void checkPC7() {
	
	static int oldstate;
	int pinstate = bit_is_set(PIND,6);
	if(pinstate != oldstate){
		PRINTNUM(pinstate);
	}
	oldstate =pinstate;
}

float waitforpress() {
	while(!bit_is_set(TIFR3,ICF3))
	set(TIFR3,ICF3);
	PRINTNUM(ICR3);
	return TCNT3;
}



int main(void){
    m_usb_init(); //Clear it before start
    checkPC7();
    	
    
      
    return 0;   /* never reached */
}
