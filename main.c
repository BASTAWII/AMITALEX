/*
* Festival LEDs.c
*
* Created: 1/3/2021 3:07:08 PM
* Author : KING
*/
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "GPIO.h"
#include "mykit.h"
//#include "LCD.h"
#include "LCD4bits.h"
#include "UART.h"
#include <stdio.h>
#include <avr/interrupt.h>
#include "ADC.h"
#include "Timer.h"

ISR(TIMER0_OVF_vect){
	PORTA ^= 0xff;
}
int main(void)
{
	DDRA= 0xff;
	Timer_init();
	sei();
	while (1)
	{
		
	}
}

/*
init_uart(9600);
ADC_init(AVCC, PS_128, ADC1);
DDRB = 0xFF;
while(1){
	ADC_SC();
	ADC_waitConversion();
	int Dataa = (5000/1024.0)* readADC();
	uart_send_str("Volt = ");
	uart_send_num(Dataa);
	uart_send_str("mv");
	uart_newline();
	_delay_ms(500);
}
*/

/*
ISR(USART_RXC_vect){
char data = UDR;
if (data == 'A')
{
PORTC ^= (1<<2);
LCD_clear_4bits();
LCD_str_4bits("Button 1 is pressed");
}
else if (data == 'B')
{
PORTC ^= (1<<7);
LCD_clear_4bits();
LCD_str_4bits("Button 2 is pressed");
}
}

config_PC(out_port);
config_PD(in_port);
config_pinB_dir(in_pin, 0);
init_uart(9600);
sei();
while(1){
if (ispressed_A(6))
{
uart_send_char('A');
}
else if (ispressed_A(2))
{
uart_send_char('B');
}
}
return 0;
*/
/*

ISR(USART_RXC_vect){
char data = UDR;
switch(data){
case 'A':
case 'a':
PORTA = 0xFF;
break;
case 'B':
case 'b':
PORTA = 0x00;
break;
}
}



int main(void)
{
init_uart(9600);
sei();
while(1){

}

}
*/
/*while(1){
uart_send_char('A');
_delay_ms(500);
uart_send_str("\rHallo Bastawy\r");
_delay_ms(500);
uart_send_num(7);
_delay_ms(500);
uart_send_str("\r");
}
*/

/*
init_LCD_4bits();
config_pinD_dir(in_pin, 2 );
config_pinD_dir(out_pin, 3 );
config_pinB_dir(in_pin, 0 );
config_pinC_dir(out_pin, 2 );
config_pinD_dir(in_pin, 6 );
config_pinC_dir(out_pin, 7 );
while (1){
if (ispressed_B(0))
{
LCD_clear_4bits();
setPinC(2);
LCD_str_4bits("LED0 is ON");
}else
{
LCD_clear_4bits();
resetPinC(2);
LCD_str_4bits("LED0 is OFF");
}
_delay_ms(500);
if (ispressed_D(6))
{
LCD_clear_4bits();
setPinC(7);
LCD_str_4bits("LED1 is ON");
}else
{
LCD_clear_4bits();
resetPinC(7);
LCD_str_4bits("LED1 is OFF");
}
_delay_ms(500);
if (ispressed_D(2))
{
LCD_clear_4bits();
setPinD(3);
LCD_str_4bits("LED2 is ON");
}else
{
LCD_clear_4bits();
resetPinD(3);
LCD_str_4bits("LED2 is OFF");
}
_delay_ms(500);
}
*/

/*
DDRD |= (1<<LED2);
while (1)
{
for (int i=0; i<7; i++)
{
PORTC |= (1<<LED0);
_delay_ms(500);
PORTC &= ~(1<<LED1);
_delay_ms(500);
PORTD &= ~(1<<LED2);

}
}
}
*/

/*
init_LCD();
LCD_data('A');
*/

/*
DDRD |= (1<<LED2);
while (1)
{
for (int i=0; i<7; i++)
{
PORTC |= (1<<LED0);
_delay_ms(500);
PORTC &= ~(1<<LED1);
_delay_ms(500);
PORTD &= ~(1<<LED2);

}
}
*/
/*	{
for (int i=0; i<7; i++)
{
_delay_ms(500);
PORTA = (PORTA << 1);
}
for (int i=7; i>0; i--)
{
_delay_ms(500);
PORTA = (PORTA >> 1);
}
}
}

*/