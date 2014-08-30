/*! \file *********************************************************************
 *
 * \brief  XMEGA USART interrupt driven driver example source.
 *
 *      This file contains an application that simulates an Radio connection between the rexus rocket and the Ground statioin.
 *		For the communication it uses 2 Uarts, potis and swiches as control.
 *
 * \par Simulator for Radio Connection
 *
 * \author
 *      Kholodkov Jakov, Thomas Gruebler \n
 *      Support email: thomas.gruebler@tum.de
 *
 * $Date: 2014-04-17 13:20 +0200   \n
 * \license
 * ----------------------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Mister X wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can give me something back in return. A beer for example.
 * Thomas Gruebler, Kholodkov Jakov
 * ----------------------------------------------------------------------------
 *****************************************************************************/

#include <avr/io.h>             // PINout and Registers
#include <avr/interrupt.h>      // interrupt driver
#include "usart_driver.h"       // Driver for usart
#include <util/delay.h>
#include "adc_driver.h"

// Led defines
#define LED_PORT PORTE
#define LED5_PORT PORTC
#define LLO_bm PIN2_bm          //1 and 3 changed in pcb layout  //Lift off indicator
#define LSOE_bm PIN1_bm         //Start of Experiment indicator
#define LSODS_bm PIN0_bm        //Stop of Experiment indicator
#define LPS_bm PIN3_bm          //Pass-through / No noise indicator
#define LPWR_bm PIN0_bm         //Experiment Power indicator

#define LED_RXTX_PORT PORTD
#define LTX_bm PIN4_bm
#define LRX_bm PIN5_bm

// Switch defines
#define SW_PORT PORTB
#define SWPWR_PORT PORTC
#define SWLO_bm PIN0_bm         //Lift off switch
#define SWSOE_bm PIN1_bm        //Start of Experiment switch
#define SWSODS_bm PIN2_bm       //Stop of Experiment switch
#define SWPS_bm PIN3_bm         //Pass-through / No noise activator
#define SWPWR_bm PIN1_bm        //Experiment Power switch

// Rocket flight signals
#define RXSIG_PORT PORTA
#define LO_bm PIN5_bm
#define SOE_bm PIN6_bm
#define SODS_bm PIN7_bm

// Power supply control
#define SUPPLY_CTRL_PORT PORTD
#define SUPPLY_CTRL_bm PIN1_bm

// Analog inputs
#define AD_PORT PORTA
#define POTI1_bm PIN1_bm
#define POTI2_bm PIN2_bm
#define POTI3_bm PIN3_bm
#define ISENSE_bm PIN4_bm

//USART
#define USARTGROUND USARTD1
#define USARTROCKET USARTD0

// Constants
#ifndef F_CPU
#define F_CPU 32000000
#endif
