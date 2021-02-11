/*
 * servos.h
 *
 *  Created on: Feb 7, 2021
 *      Author: gerd
 */

#ifndef INC_SERVOS_H_
#define INC_SERVOS_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Constants  ----------------------------------------------------------------*/
#define SERVOS_TOTAL 168

#define SERVOS_SERIAL 8
#define SERVOS_SERIAL_MASK 0x0007
#define SERVOS_SERIAL_SHIFT 3
#define SERVOS_PARALLEL (SERVOS_TOTAL / SERVOS_SERIAL)


#define SERVOS_MINVAL 0
#define SERVOS_MAXVAL 1000
#define SERVOS_DEFAULT_VAL 500

#define SERVOS_TIME_OFFSET_PARALLEL 10
#define SERVOS_TIME_PULSE_GRID 2200
#define SERVOS_TIME_PULSE_BASE 1000
#define SERVOS_TIME_PERIODE 20000

#define SERVOS_ID_EMPTY 0xFF

#define SERVOS_ERROR_OK 0
#define SERVOS_ERROR_ID_OUT_OF_RANGE 1
#define SERVOS_ERROR_NOT_CONFIGURED 2
#define SERVOS_ERROR_ALREADY_CONFIGURED 3


/* Types ---------------------------------------------------------------------*/
typedef struct
{
  unsigned int   value:10;
  unsigned int attached:1;
  unsigned int modified:1;
  unsigned int reserved:4;
  uint16_t u16GpioPin;
  GPIO_TypeDef* pGpioPort;
  uint8_t parallelSortPos;
  uint8_t servoIndex;
} SERVO_s;

typedef union
{
	SERVO_s lin[SERVOS_TOTAL];
	SERVO_s ps[SERVOS_PARALLEL][SERVOS_SERIAL];
} SERVOS_s;

typedef struct
{
  unsigned int   cnt;
  uint8_t servos[SERVOS_PARALLEL];
} SERVO_PARALLEL_s;

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
