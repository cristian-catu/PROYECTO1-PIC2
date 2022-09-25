#ifndef XC_I2C_LIBRERY_H
#define	XC_I2C_LIBRERY_H
#include <stdint.h>
#include <xc.h>                // include processor files - each processor file is guarded.4
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 4000000

#endif

/*==========================================================================================================
 ===========================================================================================================*/
void I2C_Init (uint32_t frecuencia);//Funci�n de inicializaci�n del periferico I2C.
void I2C_Start (void);              // Funci�n que inicia la comunicaci�n I2C.
void I2C_ReStart (void);            // Funci�n que reinicia la comuncaci�n I2C.
void I2C_Stop (void);               // Funci�n que detiene la comunicaci�n I2c.
void I2C_Ack (void);                // Funci�n para transmitir Acknowledge.
void I2C_NO_Ack (void);             // Funci�n para transmitir NO Acknowledge.
void I2C_Write (uint8_t data);      // Funci�n para escribir el SSPBUF.
uint8_t I2C_Read (void);            // Funci�n para leer el SSPBUF.

#endif	/* XC_I2C_LIBRERY_H */