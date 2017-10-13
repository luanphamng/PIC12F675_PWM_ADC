/*
 * File:   main.c
 * Author: CAMLAI
 *
 * Created on October 2, 2017, 10:43 AM
 */


//#include <xc.h>
#include <pic12f675.h>
//#include "delay.h"

#define PWM_Pin    GP0
#define _XTAL_FREQ   4000000  

void CUS_delay_ms(unsigned int i)
{
    while(i--);
}
unsigned char PWM = 0;
unsigned int ADC = 0;
// CONFIG
#pragma config FOSC = INTOSCCLK        // Oscillator Selection bits (XT oscillator: Crystal/resonator on GP4/OSC2/CLKOUT and GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-Up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // GP3/MCLR pin function select (GP3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

void initPWM(void)
{
    OPTION_REG &= 0xC0; // init timer 0
    
    T0IE = 1;
    GIE = 1;
}

void InitADC(unsigned char Channel)
{
	ANSEL  |= 0x02;   // Select Channel	
	TRISIO |= 0x02;	 // Make selected channel pins input
	ADCON0 = 0x81;		 // Turn on the A/D Converter
	CMCON  = 0x07;		 // Shut off the Comparator, so that pins are available for ADC
	VRCON  = 0x00;	     // Shut off the Voltage Reference for Comparator
}

/*
 * Function Name: GetADCValue
 * Input(s) :     Channel name, it can be AN0, AN1, AN2 or AN3 only.
 *                Channel is selected according to the pin you want to use in
 *                the ADC conversion. For example, use AN0 for GP0 pin.
 *				  Similarly for GP1 pin use AN1 etc.
 * Output(s):     10 bit ADC value is read from the pin and returned.
 * Author:        M.Saeed Yasin   20-06-12
 */
unsigned int GetADCValue(unsigned char Channel)
{
	ADCON0 &= 0xf3;      // Clear Channel selection bits

	switch(Channel)
	{
		case 0:	ADCON0 |= 0x00; break;      // Select GP0 pin as ADC input
		case 1:	ADCON0 |= 0x04; break;      // Select GP1 pin as ADC input
		case 2:	ADCON0 |= 0x08; break;      // Select GP2 pin as ADC input
		case 3:	ADCON0 |= 0x0c; break;      // Select GP4 pin as ADC input

		default:	return 0; 					//Return error, wrong channel selected
	}
    
    CUS_delay_ms(10);      // Time for Acqusition capacitor to charge up and show correct value

	GO_nDONE  = 1;		 // Enable Go/Done

	while(GO_nDONE);     //wait for conversion completion

	return ((ADRESH<<8)+ADRESL);   // Return 10 bit ADC value
}

void interrupt ISR(void)
{
    if(T0IF)
    {
        if(PWM_Pin) // if pin pwm is high
        {
            TMR0 = PWM;
            PWM_Pin = 0;
        }
        else
        {
            TMR0 = 255 - PWM;
            PWM_Pin = 1;
        }
        
        T0IF = 0; //clear isr
    }
}



// Main function
void main()
{	
	ANSEL  = 0x00;       // Set ports as digital I/O, not analog input
	ADCON0 = 0x00;		 // Shut off the A/D Converter
	CMCON  = 0x07;		 // Shut off the Comparator
	VRCON  = 0x00;	     // Shut off the Voltage Reference
	TRISIO = 0x08;       // GP3 input, rest all output
	GPIO   = 0x00;       // Make all pins 0
	
	initPWM();			 // Initialize PWM
    GP2 = 1;
    
    InitADC(1);
    
	// PWM=0 means 0% duty cycle and 
	// PWM=255 means 100% duty cycle
	PWM = 50;			 // 50% duty cycle 
	
	while(1)
	{
        ADC = GetADCValue(1);
//        if(ADC)
//        {
//            PWM = (unsigned char) ADC & 0xFF;
//        }
//        else
        {
            //PWM = 1;
        }
        GP2 = 0;
        CUS_delay_ms(1000);
        GP2 = 1;
        CUS_delay_ms(1000);
        PWM++;
	}
}
