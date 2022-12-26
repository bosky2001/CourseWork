# Lab 1 - Microcontrollers


## 1. LED

## 2. Introduction to Itsy Bitsy 32u4 

### 1.2.1 Modified Registers

- Data Direction Register `DDRC7`  or  `DDRB2`
- GPIO Pinout `PORTC7`   or  `PORTB2`

### 1.2.3 Variable Frequency


#### C Code

````c
#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file

int main(void){

    _clockdivide(0); //set the clock speed to 16Mhz

    /* insert your hardware initialization here */
    DDRC=0x40;
    for(;;){
        /* insert your main loop code here */ 	
        _delay_ms(1000);          
        PORTC^= 0x40;                   //LED Blinking  at 1 Hz
        
    }
      
    return 0;   /* never reached */
}
````

### 1.2.4 Variable Duty Cycle

````c

#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file

int main(void){

    _clockdivide(0); //set the clock speed to 16Mhz
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
````
## 3. Timers

### 1.3.1 Use the timer to get 20Hz

#### C Code

````c
#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file
#define  COMPAREVALUE 6250   
int main(void){

    _clockdivide(0); //set the clock speed to 16Mhz
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


````

### 1.3.2 Adjust the Pre-scaler

Adjusted the pre-scalar in the code snippet for 1.3.1.

The default system clock frequency is 16MHz.

### 1.3.3 PWM functions of timer

Do the same thing you did in 1.2.4

#### Timer options used

<img src="../imgs/fast_pwm.png" width=500>

- Output compare is set on the compare match between `TCNT1` and `OCR1A`
- Mode 7 on Timer 3 to count up to `1024`
- Toggling Port `C6` at Output Compare Match
- Set Compare match registers `0x03FF` and `OCR3A`
- `1024x` prescalar
- Set `OCr3A` on compare match when up-counting and clear on compare match



#### C Code

````c
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


```` 

#### [Video Demonstration](https://www.youtube.com/watch?v=AvenU_oT04E)

## 4. Practice with Loops

### 1.4.1 Make the External LED Pulse

#### Part A

The pulse should start at 0 intensity, take 1 second to increase in intensity until it is full brightness, then 1 second to decrease in brightness and repeat.

````c
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
    		
		 inc_light(1,1.0);
		 dec_light(1,1.0);
		 _delay_ms(2000);
	     }
    return 0;   /* never reached */
}
````

#### Part B

- Make the time to increase and the time to decrease a variable. 
- Make a video with the following configuration:
    - 0.3 second to full intensity 
    - 0.7 seconds to 0 intensity

##### [Video of Asymetric Pulses](https://youtube.com/shorts/7oierEto6Hg)

##### C Code

````c
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

```` 


### 1.4.2 Pulsing LED like Heartbeat

#### Refactor code to use Subroutines and make Max Intensity variable

Uses the following defined variables:
- `RISE_TIME`
- `MAX_INTENSITY`
- `FREQ_HZ`
- `PRESCALAR`
- `SYS_CLOCK`

Assumes LED is plugged into Port `B5`, and that Timer 1 is not being used.

#### Make the LED blink as if it were a heartbeat

LED percentage intensity `i` should follow the pattern below at time `t` seconds but smoothly interpolated intensities between each value:

|             	|     	|     	|     	|     	|     	|     	|     	|     	|     	|     	|
|-------------	|-----	|-----	|-----	|-----	|-----	|-----	|-----	|-----	|-----	|-----	|
| Time `t`      	| 0.0 	| 0.1 	| 0.5 	| 0.6 	| 1.0 	| 3.0 	| 3.1 	| 3.5 	| 3.6 	| 4.0 	|
| Intensity `i` 	| 0   	| 100 	| 0   	| 50  	| 0   	| 0   	| 100 	| 0   	| 50  	| 0   	|
|             	|     	|     	|     	|     	|     	|     	|     	|     	|     	|     	|

##### [Heartbeat LED Video](https://youtube.com/shorts/I4FAk_8U_F0)

##### C Code

````c
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
    		
		 inc_light(0.1,1.0);
		 dec_light(0.4,1.0);

		 inc_light(0.1,0.5);
		 dec_light(0.4,0.5);
		 _delay_ms(2000);
	     }
    return 0;   /* never reached */
}

````

### 1.4.3 **Extra Credit** Weaker Heartbeat

- Heartbeat stays at a constant frequency, but maximum intensity slowly fades as if the heartbeat is getting weaker
- Have it take 20 beats before it is reduced to 0 intensity
- Show a TA your LED blinking; answer his/her questions, and get signed off

##### [Weakening Heartbeat LED Video](https://youtube.com/shorts/3AidncDLvz4)

#### C Code

````c
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
    		for(float z=20.0 ; z>0.0 ; z-=1.0){	
		 	float p = z/20.0;               // for decreasing brigthness
		 	inc_light(0.1,p);		 // first heartbeat	
		 	dec_light(0.4,p);
	 		
	 		inc_light(0.1,p/2.0); 		 // second heartbeat
		 	dec_light(0.4,p/2.0);
		 	_delay_ms(3000);
	 }
	 	_delay_ms(5000);
	 
	     }
    return 0;   /* never reached */
}

````
