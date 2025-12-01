#ifndef MAIN_H
#define MAIN_H

#include "omsp_system.h"

//--------------------------------------------------
// Hardware UART register address mapping
//--------------------------------------------------

#define UART_CTL          (*(volatile unsigned char *) 0x0080)  // UART Control register (8bit)
#define UART_STAT         (*(volatile unsigned char *) 0x0081)  // UART Status register (8bit)
#define UART_BAUD         (*(volatile unsigned int  *) 0x0082)  // UART Baud rate configuration (16bit)
#define UART_TXD          (*(volatile unsigned char *) 0x0084)  // UART Transmit data register (8bit)
#define UART_RXD          (*(volatile unsigned char *) 0x0085)  // UART Receive data register (8bit)


//--------------------------------------------------
// Hardware UART register field mapping
//--------------------------------------------------

// UART Control register fields
#define  UART_IEN_TX_EMPTY  0x80
#define  UART_IEN_TX        0x40
#define  UART_IEN_RX_OVFLW  0x20
#define  UART_IEN_RX        0x10
#define  UART_SMCLK_SEL     0x02
#define  UART_EN            0x01

// UART Status register fields
#define  UART_TX_EMPTY_PND  0x80
#define  UART_TX_PND        0x40
#define  UART_RX_OVFLW_PND  0x20
#define  UART_RX_PND        0x10
#define  UART_TX_FULL       0x08
#define  UART_TX_BUSY       0x04
#define  UART_RX_BUSY       0x01


//--------------------------------------------------
// Diverse
//--------------------------------------------------

// BAUD = (mclk_freq/baudrate)-1

//#define BAUD           2916            //   9600  @28.0MHz
//#define BAUD           1457            //  19200  @28.0MHz
//#define BAUD            728            //  38400  @28.0MHz
//#define BAUD            485            //  57600  @28.0MHz
#define BAUD              242            // 115200  @28.0MHz
//#define BAUD            121            // 230400  @28.0MHz



#endif // MAIN_H
