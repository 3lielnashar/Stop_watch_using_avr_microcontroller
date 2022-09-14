/*
 * Final_mini_project2.c
 *
 *  Created on: Sep 14, 2021
 *      Author: aliel
 */


#include<avr/interrupt.h>
#include"util/delay.h"
#include<avr/io.h>
typedef unsigned char uint8;
uint8 second = 0;
uint8 minute = 0;
uint8 hour = 0;
void TIMER1_Init(void)
{
	SREG &= ~(1<<7);  // Disable interrupts by clearing I-bit
	TCCR1A |= (1<<FOC1A) |(1<<FOC1B) ; // active non-pwm mode
	TCCR1B = (1<<WGM12) | (1<<CS10)|(1<<CS12);  // mode 4 and the prescaler is 1024
	TCNT1 = 0; // start from
	OCR1A = 1000; // when the timer reaches this value compare match occurs and timer flag module set
	TIMSK |= (1<<OCIE1A); // enable Compare A Match Interrupt Enable

	SREG |= (1<<7);// enable interrupts by clearing I-bit
}

void INT0_Init(void)
{
	SREG &= ~ (1<<7); // Disable interrupts by clearing I-bit
	GICR |=(1<<INT0); // External Interrupt Request 0 Enable
	DDRB &= ~ (1<<INT0); // Configure INT0/PD2 as input pin
	PORTB |= (1<<INT0); // turn on the internal pull- up resistor
    MCUCR |=(1<<ISC01); // falling edge
    SREG |= (1<<7);// enable interrupts by clearing I-bit

}
void INT1_Init(void)
{
	SREG &= ~ (1<<7); // Disable interrupts by clearing I-bit
	GICR |=(1<<INT1); //External Interrupt Request 1 Enable
	DDRB &= ~ (1<<INT1); // Configure INT1/PD2 as input pin
	MCUCSR |=(1<<ISC11)|(1<<ISC10); //rising edge
	SREG |= (1<<7);// enable interrupts by clearing I-bit

}
void INT2_Init(void)
{
	SREG &= ~ (1<<7); // Disable interrupts by clearing I-bit
	GICR |=(1<<INT2);//External Interrupt Request 2 Enable
	DDRB &= ~ (1<<INT2);// Configure INT0/PD2 as input pin
	PORTB |= (1<<INT2); // turn on the internal pull- up resistor
	MCUCR &= ~(1<<ISC2); // falling edge
	SREG |= (1<<7);// enable interrupts by clearing I-bit

}

int main (void)
{
	DDRC |= 0x0F; // first four pins in port c are output
	DDRA |= 0x3F; // first six pins in port a are output and enable the seven segment
	PORTC |= 0x00;  // clear PORT C pins
//	SREG |= (1<<7); /* Enable global interrupts in MC */

	INT0_Init();
	INT1_Init();
	INT2_Init();
	TIMER1_Init();


	while(1)
	{
		/*

		 */
       PORTA = (PORTA & 0xC0) | (1<<5); ;
       PORTC = (PORTC & 0xF0) | ((second % 10)& 0x0F);
       _delay_ms(2);
       PORTA = (PORTA & 0xC0) | (1<<4);
       PORTC = (PORTC & 0xF0) | ((second / 10)& 0x0F);
       _delay_ms(2);
       PORTA = (PORTA & 0xC0) | (1<<3);
       PORTC = (PORTC & 0xF0) | ((minute % 10)& 0x0F);
       _delay_ms(2);
       PORTA = (PORTA & 0xC0) | (1<<2);
       PORTC = (PORTC & 0xF0) | ((minute / 10)& 0x0F);
       _delay_ms(2);
       PORTA = (PORTA & 0xC0) | (1<<1);
       PORTC = (PORTC & 0xF0) | ((hour % 10)& 0x0F);
       _delay_ms(2);
       PORTA = (PORTA & 0xC0) | (1<<0);
       PORTC = (PORTC & 0xF0) | ((hour / 10)& 0x0F);
       _delay_ms(2);

	}

}
ISR (TIMER1_COMPA_vect)
{
	second++;
	if(second == 60)
	{
		second=0;
		minute++;
	}
	if(minute == 60)
	{
		second =0;
		minute=0;
		hour++;
	}
	if (hour == 24)
	{
		second = 0;
		minute = 0;
		hour =0;
	}

}
ISR(INT0_vect)
{
	second = 0;
	minute = 0;
	hour = 0;

}
ISR(INT1_vect)
{
	TIMSK =0; // stop the timer (timer interrupt module off)
}
ISR(INT2_vect)
{
	TIMSK |= (1<<OCIE1A); // enable the timer interrupt again
}
