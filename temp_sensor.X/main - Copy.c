#include <xc.h>
#include <pic18f4520.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "conbits.h"
#include "uart_layer.h"


#define TIME_PER_OVERFLOW   (65535u) // us 65535u x 8 = 524,280 ms
#define TIME_PER_TICK       (1u) // us

#define ADC_RES (5.0/1024.0)
#define C_PER_VOLT (0.010)

#define SEG_DATALINE LATB

#define SEG_DATA_DP (1 << 0)
#define SEG_DATA_A  (1 << 5)
#define SEG_DATA_B  (1 << 4)
#define SEG_DATA_C  (1 << 1)
#define SEG_DATA_D  (1 << 2)
#define SEG_DATA_E  (1 << 3)
#define SEG_DATA_F  (1 << 6)
#define SEG_DATA_G  (1 << 7)

#define SEG_SEL0 LATDbits.LATD7   
#define SEG_SEL1 LATDbits.LATD6
#define SEG_SEL2 LATDbits.LATD5
#define SEG_SEL3 LATDbits.LATD4

#define SEG_DELAY 5000

static volatile float sensor_temp = 0.0;

static volatile uint16_t tmr0_overflow = 0;

const uint8_t program_start[18]="\r\nProgram start\n\r";
uint8_t print_buffer[100] = {0}; // buffer to print stuff to serial

volatile uint8_t uart_char = 0;
volatile bool uart_rcv_data = false;


void led_test(void){
    
    uint16_t k = 1;
    for(uint16_t i = 0; i < 8; i ++){
        LATB = k;
        k *= 2;
        if(k > 0x80){
            k = 1;
        }
        __delay_ms(500);
    }
    
}

void seg_test(void){
    
    SEG_SEL0 = 1;
    led_test();
    SEG_SEL0 = 0;
    
    SEG_SEL1 = 1;
    led_test();
    SEG_SEL1 = 0;
    
    SEG_SEL2 = 1;
    led_test();
    SEG_SEL2 = 0;
    
    SEG_SEL3 = 1;
    led_test();
    SEG_SEL3 = 0;
    
}

void seg_numbers(uint8_t num){
    LATB &= 1;
    
    
    switch(num){
        case 0:
            LATB |= SEG_DATA_A | SEG_DATA_B | SEG_DATA_C | SEG_DATA_D | SEG_DATA_E | SEG_DATA_F; 
            break; 
        case 1:
            LATB |= SEG_DATA_B | SEG_DATA_C; 
            break;

        case 2:
            LATB |= SEG_DATA_A | SEG_DATA_B | SEG_DATA_D | SEG_DATA_E | SEG_DATA_G; 
            break;

        case 3:
            LATB |= SEG_DATA_A | SEG_DATA_B | SEG_DATA_C | SEG_DATA_D | SEG_DATA_G; 
            break;
        case 4:
            LATB |= SEG_DATA_B | SEG_DATA_C | SEG_DATA_F | SEG_DATA_G;
            break;
        case 5:
            LATB |= SEG_DATA_A | SEG_DATA_C | SEG_DATA_D | SEG_DATA_F | SEG_DATA_G;
            break;
        case 6:
            LATB |= SEG_DATA_C | SEG_DATA_D | SEG_DATA_E | SEG_DATA_F | SEG_DATA_G;
            break;
        case 7:
            LATB |= SEG_DATA_A | SEG_DATA_B | SEG_DATA_C;
            break;
        case 8:
            LATB |= SEG_DATA_A | SEG_DATA_B | SEG_DATA_C | SEG_DATA_D | SEG_DATA_E | SEG_DATA_F | SEG_DATA_G;
            break;
        case 9:
            LATB |= SEG_DATA_A | SEG_DATA_B | SEG_DATA_C | SEG_DATA_F | SEG_DATA_G;
            break;   
    }
    
}

void number_test(void){
    for(uint16_t i = 0; i < 10; i ++){
        seg_numbers(i);
        __delay_ms(1000);
    }
}

void seg_convert_num(uint16_t num){

    uint16_t rem = num / 1000;
    SEG_SEL0 = 1;
    seg_numbers(rem);
    __delay_us(SEG_DELAY);
    SEG_SEL0 = 0;
    num = num - (rem * 1000);
    
    rem = num / 100;
    SEG_SEL1 = 1;
    seg_numbers(rem);
    __delay_us(SEG_DELAY);
    SEG_SEL1 = 0;
    num = num - (rem * 100);
    
    rem = num / 10;
    SEG_SEL2 = 1;
    seg_numbers(rem);
    __delay_us(SEG_DELAY);
    SEG_SEL2 = 0;
    num = num - (rem * 10);
    
    SEG_SEL3 = 1;
    seg_numbers(num);
    __delay_us(SEG_DELAY);
    SEG_SEL3 = 0; 
}

void seg_temp(float temp){
    
    uint16_t num = temp * 100;
    
    uint16_t rem = num / 1000;
    SEG_SEL0 = 1;
    seg_numbers(rem);
    __delay_us(SEG_DELAY);
    SEG_SEL0 = 0;
    num = num - (rem * 1000);
    
    rem = num / 100;
    SEG_SEL1 = 1;
    LATB |= SEG_DATA_DP;
    seg_numbers(rem);
    __delay_us(SEG_DELAY);
    SEG_SEL1 = 0;
    LATB &= 0xFE;
    num = num - (rem * 100);
    
    rem = num / 10;
    SEG_SEL2 = 1;
    seg_numbers(rem);
    __delay_us(SEG_DELAY);
    SEG_SEL2 = 0;
    num = num - (rem * 10);
    
    SEG_SEL3 = 1;
    seg_numbers(num);
    __delay_us(SEG_DELAY);
    SEG_SEL3 = 0; 
    
    
}

void main(void){

    OSCCONbits.IDLEN = 0;
    OSCCONbits.IRCF = 0x07;
    OSCCONbits.SCS = 0x03;
    while(OSCCONbits.IOFS!=1); // 8Mhz
    
    TRISDbits.RD4 = 0;
    TRISDbits.RD5 = 0;
    TRISDbits.RD6 = 0;
    TRISDbits.RD7 = 0;
    TRISB = 0x00;
    
    TRISAbits.RA0 = 1;
    
    SEG_SEL0 = 0;
    SEG_SEL1 = 0;
    SEG_SEL2 = 0;
    SEG_SEL3 = 0;
    
    LATB = 0x00;

    //adc init
    
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON1bits.PCFG = 0x0E;
    
    ADCON2bits.ADFM = 1;
    ADCON2bits.ACQT = 0x07;
    ADCON2bits.ADCS = 0x04;
    
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON = 1;
    
    PIE1bits.ADIE = 1;
    IPR1bits.ADIP = 1;
    PIR1bits.ADIF = 0;
    
    T0CONbits.TMR0ON=0; 
    T0CONbits.T08BIT=1;
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 0;
    T0CONbits.T0PS=2; //1:8
    INTCON2bits.TMR0IP=1;
    INTCONbits.T0IE=1;

    //uart init
    uart_init(51,0,1,0);//baud 9600

    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;// base interrupt setup

    //__delay_ms(2000);
    uart_send_string((uint8_t *)program_start); // everything works in setup
    
    printf("/");

    //seg_test();
    static float sensor_cap = 0.0;
    ADCON0bits.GODONE = 1;
    while(ADCON0bits.GODONE == 1){
        Nop();
    }
    sensor_cap = sensor_temp;
    

    T0CONbits.TMR0ON=1;
    uart_char = 'a';
    for(;;){ // while(1)
        
        if(tmr0_overflow >= 500){
            sensor_cap = sensor_temp;
            tmr0_overflow = 0;
        }
        if(uart_char == 'd'){
            sprintf((char*)print_buffer,"temp: %f\r",sensor_temp);
            uart_send_string((uint8_t *)print_buffer);
        }
        seg_temp(sensor_cap);

        if(ADCON0bits.GODONE == 0){
            ADCON0bits.GODONE = 1;
        }
        
        if(uart_rcv_data){
            //uart_send(uart_char);
            uart_rcv_data = false;
        }
        
    } 
}



void __interrupt() high_isr(void){
    INTCONbits.GIEH = 0;


    if(PIR1bits.RCIF){
        uart_receiver(&uart_char,&uart_rcv_data);
        PIR1bits.RCIF=0;
    }
    
    if(PIR1bits.ADIF){
        sensor_temp = (((ADRESH << 8) | (ADRESL)) * ADC_RES) / C_PER_VOLT;
        PIR1bits.ADIF = 0;
    }
    
    if(INTCONbits.TMR0IF){
        tmr0_overflow++;
        INTCONbits.TMR0IF = 0;
    }

    INTCONbits.GIEH = 1;
}

void __interrupt(low_priority) low_isr(void){
    INTCONbits.GIEH = 0;

    if(0){

    }

    INTCONbits.GIEH = 1;
}
