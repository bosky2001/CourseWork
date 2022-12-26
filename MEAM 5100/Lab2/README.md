# Lab 2 - Capturing Inputs

Sheil Sarda <sheils@seas.upenn.edu>

## 1. Switches, Debouncing and Input Capture

#### C Code

````c
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

int main(void){
    m_usb_init(); //Clear it before start
    checkPC7();
    return 0;   /* never reached */
}

````




## 2. Phototransistors


#### Phototransistor Behavior w.r.t. Light

- In no light circumstances, there is no base current produced in the phototransistor
- Under normal light, current is produced in the phototransistor
- If curcuit does not have a pull-up, `PIND6` will be high when no light and low in normal light
- Increasing the resistor value makes the phototransistor circuit more sensitive to light changes due to Ohm's law `V = IR`

#### C Code

````c
#include "teensy_general.h"

#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file
#include "m_usb.h"

#define PRINTNUM(x) m_usb_tx_uint(x); m_usb_tx_char(10); m_usb_tx_char(13)
#define VAL1 340 
#define VAL2 11
int main(void){
    m_usb_init(); //Clear it before start
    set(TCCR3B,CS32);
    set(TCCR3B,CS30);
    set(DDRB,2); //Set port C6 to be output
	
    while(1){
    	if(bit_is_set(PIND,6)){
    		set(PORTB,2);
    	}
    	if(!bit_is_set(PIND,6)){
    		clear(PORTB,2);
    	}
    }
    
      
    return 0;   /* never reached */
}
````

#### [Video Demo of LED Phototransistor](https://youtube.com/shorts/10NT4-59IJA)

### 2.2.2 Varying Resistance to make it more sensitive

#### [Video Demo of Hand-Waiving](https://youtube.com/shorts/OLkk4m0PEag)


## 2.3 Operational Amplifier



#### C Code

````c
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

````
