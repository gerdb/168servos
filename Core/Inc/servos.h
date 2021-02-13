/*
 *  Project:      168servos
 *  File:         servos.h
 *  Author:       Gerd Bartelt - www.sebulli.com
 *
 *  Description:  header file for servos module
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef INC_SERVOS_H_
#define INC_SERVOS_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Constants  ----------------------------------------------------------------*/

// Set the maximum number of servos as amultiple of 8. Max 168.
#define SERVOS_TOTAL 168


#define SERVOS_SERIAL 8
#define SERVOS_SERIAL_MASK 0x0007
#define SERVOS_SERIAL_SHIFT 3
#define SERVOS_PARALLEL (SERVOS_TOTAL / SERVOS_SERIAL)


#define SERVOS_MINVAL 0
#define SERVOS_MAXVAL 1000
#define SERVOS_DEFAULT_VAL 500

#define SERVOS_TIME_OFFSET_PARALLEL 10
#define SERVOS_TIME_PULSE_GRID 2250
#define SERVOS_TIME_PULSE_BASE 1000
#define SERVOS_TIME_PERIODE 20000

#define SERVOS_ID_EMPTY 0xFF

#define SERVOS_ERROR_OK 0
#define SERVOS_ERROR_ID_OUT_OF_RANGE 1
#define SERVOS_ERROR_NOT_CONFIGURED 2
#define SERVOS_ERROR_ALREADY_CONFIGURED 3


/* Types ---------------------------------------------------------------------*/

// Structure for one servo
typedef struct
{
  unsigned int   value:10;
  unsigned int attached:1;
  unsigned int reserved:5;
  uint16_t u16GpioPin;
  GPIO_TypeDef* pGpioPort;
  uint8_t parallelSortPos;
  uint8_t servoIndex;
} SERVO_s;

// Contains all servos that are controlled in parallel (1..21)
typedef struct
{
  unsigned int   cnt;
  uint8_t servos[SERVOS_PARALLEL];
} SERVO_PARALLEL_s;

// Action containing the data for the ISR
typedef struct
{
	uint16_t u16ARR;
	__IO uint32_t* pServosBSSR;
	uint32_t u32ServosBSSRVal;
} SERVO_ACTION_s;


/* Function prototypes  ------------------------------------------------------*/
void SERVOS_Init();
int SERVOS_Configure(uint8_t u8ServoId, uint16_t u16GpioPin, GPIO_TypeDef* pGpioPort);
int SERVOS_Set(uint8_t u8ServoId, uint32_t u32Value);
int SERVOS_Detach(uint8_t u8ServoId);
int SERVOS_Attach(uint8_t u8ServoId);
void SERVOS_Task1ms();
void SERVOS_ISR(void);


#endif /* INC_SERVOS_H_ */
