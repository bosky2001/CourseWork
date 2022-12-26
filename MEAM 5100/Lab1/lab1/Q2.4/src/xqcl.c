/* Name: main.c
 * Author: <Raghavesh Viswanath>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file

int main(void){

    _clockdivide(0); //set the clock speed to 16Mhz
    /*set_led(ON);		*/	// turn on the on board LED
    /*_delay_ms(1000);*/		// wait 1000 ms when at 16 MHz

    /* insert your hardware initialization here */
    set(PORTB,2);
    
    float duty =0.25
    
     for(;;){
        /* insert your main loop code here */ 
        	
       _delay_ms(1000*duty);
       toggle(PORTB,2);
       _delay_ms(1000*(1-duty));
    }  
    return 0;   /* never reached */
}
