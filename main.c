/*
 * File:   main.c
 * Author: Pablo
 * Ejemplo de uso de I2C Master
 * Created on 17 de febrero de 2020, 10:32 AM
 */
//*****************************************************************************
// Palabra de configuración
//*****************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*****************************************************************************
// Definición e importación de librerías
//*****************************************************************************
#include <stdint.h>
#include <pic16f887.h>
#include "I2C.h"
#include "oscilador.h"
#include "adc.h"
#include "I2C_librery.h"
#include "LCD.h"
#include "SPI.h"
#include "tmr0.h"
#include <xc.h>
#include <stdio.h>
//*****************************************************************************
// Definición de variables
//*****************************************************************************
#define _XTAL_FREQ 4000000

//*****************************************************************************
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void setup2(void);
uint8_t z;
uint8_t dato;

uint8_t hours = 12;                             // Variable de 8 bits para las horas.
uint8_t minutes = 59;                           // Variable de 8 bits para los minutos.
uint8_t seconds = 45;                           // Variable de 8 bits para los segundos.
uint8_t POT1 = 0;
uint8_t POT2 = 0;
uint8_t canal_ADC = 0;
uint8_t VALOR_RECIBIDO = 0;
uint8_t VALOR_A_ENVIAR = 0;
uint8_t entrada = 0;
uint8_t entrada2 = 0;
uint8_t bandera = 0;
uint8_t PUSH = 0;
uint8_t PUSH2 = 0;
uint8_t bandera2 = 0;
uint8_t bandera3 = 0;
uint8_t POTENCIOMETRO2 = 25;
uint8_t cambiar = 0;
uint8_t temperatura = 0;
uint8_t adelante = 0;
uint8_t estado = 0;
uint8_t direccion = 0;
uint8_t mover = 1;

char s[];
unsigned short VOLTAJE_0 = 0;
uint8_t init_POT_0 = 0;
uint8_t dec_POT_0 = 0;
uint8_t VAL_POT_0 = 0;

uint8_t BCD_a_Decimal (uint8_t numero);
uint8_t Decimal_a_BCD (uint8_t numero);    // Función que convierte un número decimal a BCD.
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, //Función del mapeo
            unsigned short out_min, unsigned short out_max);
//*****************************************************************************
// Main
//*****************************************************************************

void __interrupt() isr(void){
    if(PIR1bits.RCIF){
        entrada = RCREG;
        if (bandera == 0){ //Recibimos valor y guardamos segun la bandera
            seconds = entrada;
            seconds = BCD_a_Decimal(seconds);
        }
        else if (bandera == 1){
            minutes = entrada;
            minutes = BCD_a_Decimal(minutes);
        }
        else if (bandera == 2){
            hours = entrada;
            hours = BCD_a_Decimal(hours);
        }
        else if (bandera == 3){
            POT1 = entrada;
        }
        else if (bandera == 4){
            PUSH = entrada;
        }
        else if (bandera == 5){
            PUSH2 = entrada;
        }
        else if (bandera == 6){
            temperatura = entrada;
        }
    }
    if(PIR1bits.SSPIF){             // ¿Recibió datos el esclavo?
        entrada2 = SSPBUF;
        if (entrada2 == 0){         //Enviamos seconds
            spiWrite(seconds);
        }
        else if (entrada2 == 1){ //Enviamos minutes
            spiWrite(minutes);
        }
        else if (entrada2 == 2){ //Enviamos hours
            spiWrite(hours);
        }
        else if (entrada2 == 3){ //Enviamos POT1
            spiWrite(POT1);
        }
        else if (entrada2 == 4){ //Enviamos PUSH1
            spiWrite(PUSH);
        }
        else if (entrada2 == 5){ //Enviamos PUSH2
            spiWrite(PUSH2);
        }
        else if (entrada2 == 6){ //Enviamos temperatura
            spiWrite(temperatura);
        }
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción
    }
    return;
}

void main(void) {
    setup2();
    Lcd_Init(); //inicialización
    Lcd_Clear(); //limpiamos LCD
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("    ");
    while(1){
        VOLTAJE_0 = map(POT1, 0, 255, 0, 500); //mapeamos el voltaje de 0 a 500
        init_POT_0 = VOLTAJE_0/100; //guardo el entero
        dec_POT_0 = VOLTAJE_0-init_POT_0*100; //guardo el decimal
        sprintf(s, " %d%d:%d%d:%d%d | %d  %d", hours/10 ,hours%10, minutes/10, minutes%10, seconds/10, seconds%10, PUSH, PUSH2);
        Lcd_Set_Cursor(1,1); //escojo la fila
        Lcd_Write_String(s); //Escribimos en LCD
        sprintf(s, "   %d.%d%dV  | %d%dC",  init_POT_0, dec_POT_0/10, dec_POT_0%10, temperatura/10, temperatura%10);
        Lcd_Set_Cursor(2,1); //escojo la fila
        Lcd_Write_String(s); //Escribimos en LCD
        PORTB = seconds;
        if (PIR1bits.TXIF){             // Esperamos a que esté libre el TXREG para poder enviar por el serial
            if (bandera == 0){
                TXREG = 0;    // Cargamos caracter a enviar para segundos
                __delay_ms(10);
                bandera = 1;
            }
            else if (bandera == 1){
                TXREG = 1;    // Cargamos caracter a enviar para minutos
                __delay_ms(10);
                bandera = 2;
            }
            else if (bandera == 2){
                TXREG = 2;    // Cargamos caracter a enviar para horas
                __delay_ms(10);
                bandera = 3;
            }
            else if (bandera == 3){
                TXREG = 3;    // Cargamos caracter a enviar para POT
                __delay_ms(10);
                bandera = 4;
            }
            else if (bandera == 4){
                TXREG = 4;    // Cargamos caracter a enviar para PUSH1
                __delay_ms(10);
                bandera = 5;
            }
            else if (bandera == 5){
                TXREG = 5;    // Cargamos caracter a enviar para PUSH2
                __delay_ms(10);
                bandera = 6;
            }
            else if (bandera == 6){
                TXREG = 6;    // Cargamos caracter a enviar para temperatura
                __delay_ms(10);
                bandera = 0;
            }
        }
    }
    return;
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************

void setup2(void){
    ANSEL = 0;
    ANSELH = 0;
    
    TRISA = 0b00100000;
    TRISB = 0;
    TRISD = 0;
    TRISC = 0b10011001;
    
    PORTB = 0;
    PORTD = 0;
    init_osc_MHz(2);
    I2C_Slave_Init(0x50);   
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción
    PIR1bits.SSPIF = 0; 
    PIE1bits.SSPIE = 1; 
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_ACTIVE_2_IDLE);//inicalizamos el SPI del esclavo
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
uint8_t BCD_a_Decimal (uint8_t numero)            // Función que convierte un número BCD a decimal.
{
  return ((numero >> 4) * 10 + (numero & 0x0F));  // Retornamos la variable "numero" desplazado 4 posiciones a la izquierda, multiplicado por 10 más "numero" &&  1111.
}
uint8_t Decimal_a_BCD (uint8_t numero)            // Función que convierte un número decimal a BCD. 
{
    return (((numero / 10) << 4) + (numero % 10));// Retornamos la decena de la variable "numero" desplazado 4 bits a la derecha y las unidades de la variable "numero" en el nibble menos significativo
}