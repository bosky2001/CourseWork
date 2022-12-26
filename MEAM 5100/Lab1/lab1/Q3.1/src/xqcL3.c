/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file
#define  COMPAREVALUE 6250   
int main(void){

    _clockdivide(0); //set the clock speed to 16Mhz
    /*set_led(ON);		*/	// turn on the on board LED
    /*_delay_ms(1000);*/		// wait 1000 ms when at 16 MHz

    /* insert your hardware initialization here */
    DDRB=0x04; 
    
    set(TCCR3B,CS30);
    set(TCCR3B,CS31); // using prescaler /64
     
    
    
    for(;;){
    	if(TCNT3 > COMPAREVALUE){
    		PORTB^= 0x04;
    		TCNT3 =0;
    	}
    }
    return 0;   /* never reached */
}
