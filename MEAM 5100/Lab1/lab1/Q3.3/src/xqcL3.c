/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file
#define  COMPAREVALUE 512   //0X03FF
int main(void){

    _clockdivide(0); //set the clock speed to 16Mhz
    /*set_led(ON);		*/	// turn on the on board LED
    /*_delay_ms(1000);*/		// wait 1000 ms when at 16 MHz

    /* insert your hardware initialization here */
    DDRC=0x40; // Pc6 timer 16, channel B (8 bit)
    //PORTC^=0x40;
    TCCR3A= 0x83;   // WGM31 &30 AND SET IN TOGGLE SET (WGM 31 &30)
    set(TCCR3A,COM3A0);
    TCCR3B=0x09;   //WGM32 AND CS30 ENABLED
    set(TCCR3B,CS32); // using prescaler /1024
     
    
    
    OCR3A = 200; // using a duty cycle of  200/1024
    
    while(1); 
    return 0;   /* never reached */
}
