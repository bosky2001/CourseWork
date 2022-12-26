/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file
#define  COMPAREVALUE 512   //0X03FF
void inc_light(float time, float intensity){
	float L= 1/time;  // defining a variable to correspond to increase time
	float t =1/intensity;// defining a variable to correspond to value to count till/from to regulate intensity
	for(float i = 1024.0;i>1024*(1.0-intensity);i-=L/t){
	 		OCR3A= i;
	 		_delay_ms(1);
	 } 
}

void dec_light(float time, float intensity){
	float L= 1/time;// defining a variable to correspond to increase time
	float t =1/intensity;// defining a variable to correspond to value to count till/from to regulate intensity
	for(float i =1024*(1-intensity);i<1024.0;i+= L/t){
	 		OCR3A= i;
	 		_delay_ms(1);
	 } 
}
int main(void){

    _clockdivide(0); //set the clock speed to 16Mhz
    
    DDRC=0x40; // Pc6 timer 16, channel B (8 bit)
    
    
    
     
    
    // Using MODE3
    TCCR3A=0x83;// WGM 31 AND 30 AND in set
    set(TCCR3A,COM3A0);
    TCCR3B=0x02;
    
    
    while(1) {
    		
		 inc_light(0.3,1.0);
		 dec_light(0.7,1.0);

		 
		 _delay_ms(2000);
	 
	 
	 
	     }
    return 0;   /* never reached */
}




