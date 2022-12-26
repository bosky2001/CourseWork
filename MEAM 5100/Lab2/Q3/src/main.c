
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
unsigned long tperiod = 0;
unsigned long oldtime = 0;
long double real_time;
long double Total = 0;

void presstime()
{
    while(!bit_is_set(TIFR3, ICF3));
    set(TIFR3, ICF3); // clear flag by writing 1 to flag
    tperiod = ICR3 - oldtime;
    oldtime = ICR3;
    real_time = (tperiod*1024.0/16000000.0)*1000.0;
    m_usb_tx_uint(real_time);
    Total += real_time;
}



int main(void){
    m_usb_init(); //Clear it before start
    set(TIFR3,ICF3);  //starting the clock
    set(TCCR3B,CS32); // setting the prescaler
    set(TCCR3B,CS30);
    set(DDRB,2); //Set port C6 to be output
	
    while(1){
	time =presstime();
	if(if time>=25.0 && time <=75.0){
		set(PORTB,3);// turn on red led port when 23 hz
		clear(PORTB,2);
		}	
	else if( time <=2.0){
		set(PORTB,2);  // turn on green led when 700 hz
		clear(PORTB,3);
		}
}

      
    return 0;   /* never reached */
}
