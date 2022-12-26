/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "avr/io.h"
#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file
#include "m_usb.h"

void setup( int channel)
{
	set(ADMUX,REFS0);
	//set(ADMUX,REFS1); if you want 2.56 refV
	
	set(ADCSRA,ADPS0);
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS2);// SETTING CLOCK TO /128
	
	if(channel == 1){
		set(DIDR0,ADC1D); // setting up ADC Channel1
		
		set(ADMUX,MUX0);
		clear(ADMUX,MUX1);
		clear(ADMUX,MUX2);
		
		clear(ADCSRB,MUX5);	
	}
	
	else if(channel == 5){
		set(DIDR0,ADC5D);// setting up ADC Channel 5
		
		set(ADMUX,MUX0);
		clear(ADMUX,MUX1);
		set(ADMUX,MUX2);
		
		clear(ADCSRB,MUX5);	
	}

}

int read()
{
    int value;

    set(ADCSRA, ADEN);  // setting up the ADC Counter 

    while(1){
        
        set(ADCSRA, ADSC);
        
        //read the ADC
        if (bit_is_set(ADCSRA, ADIF)) { 
            // if flag is set (conversion complete) update  
            set(ADCSRA, ADIF); // reset the flag

            value = ADC;
            return value;
            break;                      
                                                       
        }
        
        
    }
}

void PWM_init(int Output_Channel)
{
    if (Output_Channel == 6)
    {
        set(DDRC, 6); // set PIN C6
    
        clear(TCCR3B, CS31);clear(TCCR3B, CS30);set(TCCR3B, CS32); // set prescaler to /256
    
        set(TCCR3A, COM3A1);
    
        set(TCCR3B, WGM33); // set to mode 14
        set(TCCR3B, WGM32); // set to mode 14
        set(TCCR3A, WGM31); // set to mode 14
        clear(TCCR3A, WGM30); // set to mode 14
    
        ICR3 = 1250; // 16000000/(256 * 50) = 1250

    }
    
    else if(Output_Channel == 5)
    {
        set(DDRB, 5); // set PIN B5
    
        clear(TCCR1B, CS11);clear(TCCR1B, CS10);set(TCCR1B, CS12); // set prescaler to /256
    
        set(TCCR1A, COM1A1);
    
        set(TCCR1B, WGM13); // set to mode 14
        set(TCCR1B, WGM12); // set to mode 14
        set(TCCR1A, WGM11); // set to mode 14
        clear(TCCR1A, WGM10); // set to mode 14
    
        ICR1 = 1250; // 16000000/(256 * 50) = 1250

    }
    
}

void setPWM( int adc, int channel){
    if (channel == 6)
    {
        
        OCR3A = 30 + 0.054*(1023-adc)+10; // writing down the equation to have the necessary pulse width
    
        m_usb_tx_string("OCR3A   ");  // printing the OCR3A values to note the interval of responses
        m_usb_tx_int(OCR3A);
        m_usb_tx_string("\n");
    }
    else if (channel == 5)
    {
        float reval = (float)adc + 312;// writing down the equation to have the necessary pulse width
        float duty = (((1024.0 - reval) / 1024.0) + 1.0) / 20.0;
        OCR1A = duty* 1250;
    
        m_usb_tx_string("OCR1A   ");// printing the OCR1A values to note the interval of responses
        m_usb_tx_int(OCR1A);
        m_usb_tx_string("\n");
    }
}
int main(void){
	
    m_usb_init();

    while(1)
    {
    setup(1); //setting up ADC channel 1
    PWM_init(5); // setting up timer at B5
    int a1 = read();

    m_usb_tx_string("\rADC1 = ");  // \r is for a carriage return, ascii 13   
    m_usb_tx_uint(a1);
    m_usb_tx_string("		"); // Clears any overflow chars for \r.
    setPWM(a1,5);  //setting the target for the timer or the duty cycle
    
    
    setup(5);//setting up ADC channel 5
    PWM_init(6);// setting up timer at C6
    int a2 = read();

    m_usb_tx_string("\rADC2 = ");  // \r is for a carriage return, ascii 13   
    m_usb_tx_uint(a2);
    m_usb_tx_string("\n"); // Clears any overflow chars for \r.

    setPWM(a2,6);//setting the target for the timer or the duty cycle
    m_usb_tx_string("\n");
    
   } 
    
      
    return 0;   /* never reached */
}
