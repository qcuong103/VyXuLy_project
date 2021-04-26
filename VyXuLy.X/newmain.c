/*
 * File:   Newman.c
 * Author: CUONG
 *
 * Created on Ngày 23 tháng 4 n?m 2021, 21:36
 */

 // PIC frequency (Hz)

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000

#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT ensabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits
#pragma config CP = OFF         // Flash Program M

#define BUZZ RB0
#define LED1 RB1
#define LED2 RB2
#define LED_BUILTIN RB3
#define SERVO1 RB4
#define SENSOR1 RD0
#define SENSOR2 RD1

//String inputString = "";
//byte dataFromI2C;
static unsigned long millisPower;
short stringComplete = 0;

short alarm_signals, tick_blink;
short trigger_s1, trigger_s2, latch_s1, latch_s2;
short ext_comm_blinking, ext_comm_lamp, ext_comm_barriers;
short barriers_lifting, barriers_lowering;
short crossing_blocked, barriers_open = 1;

uint8_t counter0 = 0;

void servoRotate0() //0 Degree
{
  unsigned int i;
  for(i=0;i<50;i++)
  {
    SERVO1 = 1;
    __delay_us(800);
    SERVO1 = 0;
    __delay_us(19200);
  }
}

void servoRotate90() //90 Degree
{
  unsigned int i;
  for(i=0;i<50;i++)
  {
    SERVO1 = 1;
    __delay_us(1500);
    SERVO1 = 0;
    __delay_us(18500);
  }
}


void main(void) {
    TRISD = 0xFF;   // Set direction of port D as Input
    TRISB = 0x00;   // Set direction of port B as Output
    PORTB = 0x00;   // Set value of port B as 0x00
    
    // Config Interrupt cho Timer1
    TMR1IE = 1;                 // Timer1 Interrupt enable bit
    TMR1IF = 0;                 // Clear the Interrupt flag bit for timer1        
    PEIE = 1;                   // Peripherals Interrupts enable bit 
    GIE = 1;                    // Global Interrupt Enable bit
    
    // Config Timer1
    // Clear the Timer1 register to start counting from 0
    TMR1 = 0;                   
    // Clear the Timer1 clock select bit to choose local clock source
    TMR1CS = 0;                 
    // Prescaler ratio 1:1
    T1CKPS0 = 0;
    T1CKPS1 = 0;
    
    servoRotate0();
    servoRotate90(); //Gate open
    
    // Switch ON Timer1
    TMR1ON = 1;
    
    while(1)
    {
        // SENSORS CODE
        if (SENSOR1 == 1) {
            trigger_s1 = 1;
            latch_s1 = 1;
        } else trigger_s1 = 0;
        
        if (SENSOR2 == 1) {
            trigger_s2 = 1;
            latch_s2 = 1;
        } else trigger_s2 = 0;
        
        if (latch_s1 && latch_s2 && !trigger_s1 && !trigger_s2) {
            latch_s1 = 0;
            latch_s2 = 0;
        }
        
        if (latch_s1 || latch_s2) {
            crossing_blocked = 1;
            LED_BUILTIN = 1;
        } else {
            crossing_blocked = 0;
            LED_BUILTIN = 0;
        }
        
        // GATE CONTROL
        if (!ext_comm_barriers) {
            if (barriers_open && crossing_blocked) barriers_lowering = 1;
            if (!barriers_open && !crossing_blocked) barriers_lifting = 1;
        }
        
        // SERVO CONTROL
        if (barriers_lifting) { // Lifting of barriers
            servoRotate90();
            barriers_lifting = 0;
            barriers_open = 1;
            LED1 = 0;
            LED2 = 0;
        }
        
        if (barriers_lowering) { // Lowering of barriers
            servoRotate0();
            barriers_lowering = 0;
            barriers_open = 0;
        }
        
        // BLINK CODE
//        if (ext_comm_blinking || crossing_blocked) {
//            if (counter0 == 8) {
//                tick_blink = !tick_blink;
//                counter0 = 0;
//            }
//            LED1 = tick_blink;
//            LED2 = !tick_blink;
//        } else {
//            LED1 = 0;
//            LED2 = 0;
//        }
    }
    return;
}

void __interrupt() ISR(void){
    if(TMR1IF == 1){               // Check the flag bit 
                        // Count 15 times of overflow
        if (ext_comm_blinking || crossing_blocked) {
            counter0++; 
            if (counter0 == 8) {
                BUZZ = ~BUZZ;
                tick_blink = !tick_blink;
                counter0 = 0;
            }
            LED1 = tick_blink;
            LED2 = !tick_blink;
        }
        TMR1IF = 0;                 // Clear interrupt bit for timer1
    }               
} 