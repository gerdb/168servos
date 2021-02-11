/*
 * servos.c
 *
 *  Created on: Feb 7, 2021
 *      Author: gerd
 */

/* Includes ------------------------------------------------------------------*/
#include "servos.h"
#include "tim.h"

/* Variables -----------------------------------------------------------------*/
uint8_t servos_index[SERVOS_TOTAL];
int servos_cnt;
SERVO_PARALLEL_s servos_parallel[SERVOS_SERIAL];
SERVO_ACTION_s servos_actions[2*SERVOS_TOTAL+2];
int servos_actions_cnt;
int servos_actions_len;
int bServos_RequestUpdate;

uint16_t u16servosARR;
uint32_t u32ServosBSSRVal;
__IO uint32_t* pServosBSSR;
__IO uint32_t u32ServosDummyBSSR;
SERVO_ACTION_s* pServosAction;

SERVOS_s  servos;

/* Prototypes of static function ---------------------------------------------*/
static void SERVOS_Update(void);


/* Functions -----------------------------------------------------------------*/

/**
 * Initialize this module
 *
 */
void SERVOS_Init()
{
	servos_cnt = 0;
	for (int i=0; i<SERVOS_TOTAL; i++)
	{
		servos_index[i] = SERVOS_ID_EMPTY;
	}
	for (int i=0; i<SERVOS_SERIAL; i++)
	{
		servos_parallel[i].cnt = 0;
	}

	servos_actions[0].pServosBSSR = &(u32ServosDummyBSSR);
	servos_actions[0].u32ServosBSSRVal = 0;
	servos_actions[0].u16ARR = 0 ;

	pServosBSSR      = servos_actions[1].pServosBSSR = &(u32ServosDummyBSSR);
	u32ServosBSSRVal = servos_actions[1].u32ServosBSSRVal = 0;
	u16servosARR     = servos_actions[1].u16ARR = 1000-1;

	servos_actions_cnt = 0;
	servos_actions_len = 2;

	bServos_RequestUpdate = 0;
	HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * Add a new servo
 *
 */
int SERVOS_Configure(uint8_t u8ServoId, uint16_t u16GpioPin, GPIO_TypeDef* pGpioPort)
{
	int serialIndex;
	int parallelIndex;
	// Check range of u8ServoId
	if (u8ServoId >= SERVOS_TOTAL)
	{
		return SERVOS_ERROR_ID_OUT_OF_RANGE;
	}

	if (servos_index[u8ServoId] != SERVOS_ID_EMPTY)
	{
		return SERVOS_ERROR_ALREADY_CONFIGURED;
	}

	// Add servoID to table
	servos_index[u8ServoId] = servos_cnt;
	servos.lin[servos_cnt].value = SERVOS_DEFAULT_VAL;
	servos.lin[servos_cnt].attached = 0;
	servos.lin[servos_cnt].servoIndex = servos_cnt;
	servos.lin[servos_cnt].pGpioPort = pGpioPort;
	servos.lin[servos_cnt].u16GpioPin = u16GpioPin;

	// update the parallel counter
	serialIndex = servos_cnt & SERVOS_SERIAL_MASK;
	parallelIndex = servos_cnt >> SERVOS_SERIAL_SHIFT;
	servos_parallel[serialIndex].cnt = parallelIndex + 1;
	servos_parallel[serialIndex].servos[parallelIndex] = servos_cnt;
	servos.lin[servos_cnt].parallelSortPos = parallelIndex;

	//added
	servos_cnt++;

	return SERVOS_ERROR_OK;
}


__STATIC_INLINE void SERVOS_Swap(SERVO_s* thisServo, SERVO_s* otherServo, SERVO_PARALLEL_s* servosParallel)
{
	// Swap
	thisServo->parallelSortPos--;
	otherServo->parallelSortPos++;
	servosParallel->servos[thisServo->parallelSortPos] = thisServo->servoIndex;
	servosParallel->servos[otherServo->parallelSortPos] = otherServo->servoIndex;
}


/**
 * Set the value of a servo
 *
 */
int SERVOS_Set(uint8_t u8ServoId, uint32_t u32Value)
{
	int servoindex;
	uint32_t u32ValueOld;
	int serialIndex;
	int bFound;
	SERVO_s* thisServo;
	SERVO_s* otherServo;
	SERVO_PARALLEL_s* servosParallel;

	// Check range of u8ServoId
	if (u8ServoId >= SERVOS_TOTAL)
	{
		return SERVOS_ERROR_ID_OUT_OF_RANGE;
	}

	// Limit to maximum value
	if (u32Value > SERVOS_MAXVAL)
	{
		u32Value = SERVOS_MAXVAL;
	}

	// Set value and attach servo
	servoindex = servos_index[u8ServoId];

	if (servoindex == SERVOS_ID_EMPTY)
	{
		return SERVOS_ERROR_NOT_CONFIGURED;
	}

	thisServo = &servos.lin[servoindex];

	u32ValueOld = thisServo->value;
	thisServo->value = u32Value;
	thisServo->attached = 1;

	// has the value changed?
	if (u32ValueOld != u32Value)
	{
		thisServo->modified = 1;
		// update the parallel counter
		serialIndex = servoindex & SERVOS_SERIAL_MASK;
		//parallelIndex = servos_cnt >> SERVOS_SERIAL_SHIFT;
		servosParallel = &servos_parallel[serialIndex];

		// Are there more than 1 servos, we have to sort them
		if ( servosParallel->cnt > 1)
		{
			// sort downwards
			do
			{
				bFound = 0;

				if (thisServo->parallelSortPos > 0)
				{
					otherServo = &servos.lin[servosParallel->servos[(thisServo->parallelSortPos)-1]];
					if (u32Value < 	otherServo->value )
					{
						bFound = 1;
						// Swap
						SERVOS_Swap(thisServo, otherServo, servosParallel);
					}
				}
			} while (bFound);

			// sort upwards
			do
			{
				bFound = 0;

				if (thisServo->parallelSortPos < (servosParallel->cnt-1))
				{
					otherServo = &servos.lin[servosParallel->servos[(thisServo->parallelSortPos)+1]];
					if (u32Value  >	otherServo->value )
					{
						bFound = 1;
						// Swap
						SERVOS_Swap(thisServo, otherServo, servosParallel);
					}
				}
			} while (bFound);

		}
	}

	return SERVOS_ERROR_OK;
}


/**
 * Detach a servo
 *
 */
int SERVOS_Detach(uint8_t u8ServoId)
{
	int servoindex;

	// Check range of u8ServoId
	if (u8ServoId >= SERVOS_TOTAL)
	{
		return SERVOS_ERROR_ID_OUT_OF_RANGE;
	}

	// Detach servo
	servoindex = servos_index[u8ServoId];
	if (servoindex == SERVOS_ID_EMPTY)
	{
		return SERVOS_ERROR_NOT_CONFIGURED;
	}

	servos.lin[servoindex].attached = 0;

	return SERVOS_ERROR_OK;
}

/**
 * Detach a servo
 *
 */
int SERVOS_Attach(uint8_t u8ServoId)
{
	int servoindex;

	// Check range of u8ServoId
	if (u8ServoId >= SERVOS_TOTAL)
	{
		return SERVOS_ERROR_ID_OUT_OF_RANGE;
	}

	// Attach servo
	servoindex = servos_index[u8ServoId];
	if (servoindex == SERVOS_ID_EMPTY)
	{
		return SERVOS_ERROR_NOT_CONFIGURED;
	}

	servos.lin[servoindex].attached = 1;

	return SERVOS_ERROR_OK;
}


/**
 * Update the timer table
 *
 */
static void SERVOS_Update(void)
{
	int time_us = 0;
	int time_us_last = 0;
	int action_cnt;
	int parallelPulsesTime;
	SERVO_s* pServo;
	SERVO_PARALLEL_s* servosParallel;

	action_cnt = 1;
	for (int s=0; s<SERVOS_SERIAL; s++)
	{
		servosParallel = &servos_parallel[s];
		parallelPulsesTime = s * SERVOS_TIME_PULSE_GRID;

		for (int p=0; p<servosParallel->cnt; p++)
		{
			pServo = &servos.lin[servosParallel->servos[p]];
			time_us = p * SERVOS_TIME_OFFSET_PARALLEL + parallelPulsesTime;
			servos_actions[action_cnt].pServosBSSR = &pServo->pGpioPort->BSRR;
			if (pServo->attached)
			{
				servos_actions[action_cnt].u32ServosBSSRVal = pServo->u16GpioPin;
			}
			else
			{
				servos_actions[action_cnt].u32ServosBSSRVal = 0;
			}
			servos_actions[action_cnt-1].u16ARR = time_us - time_us_last - 1;
			time_us_last = time_us;
			action_cnt++;
		}
		for (int p=0; p<servosParallel->cnt; p++)
		{
			pServo = &servos.lin[servosParallel->servos[p]];
			time_us = pServo->value + SERVOS_TIME_PULSE_BASE
					+  p * SERVOS_TIME_OFFSET_PARALLEL + parallelPulsesTime;
			servos_actions[action_cnt].pServosBSSR = &pServo->pGpioPort->BSRR;
			servos_actions[action_cnt].u32ServosBSSRVal = (uint32_t)pServo->u16GpioPin << 16;
			servos_actions[action_cnt-1].u16ARR = time_us - time_us_last - 1;
			time_us_last = time_us;
			action_cnt++;
		}
	}
	time_us = SERVOS_TIME_PERIODE;
	servos_actions[action_cnt].pServosBSSR = &u32ServosDummyBSSR;
	servos_actions[action_cnt].u32ServosBSSRVal = 0;
	servos_actions[action_cnt-1].u16ARR = time_us - time_us_last - 1;
	servos_actions_len = action_cnt;
}


/**
 * Call this function every 1ms from main
 *
 */
void SERVOS_Task1ms()
{
	if (bServos_RequestUpdate)
	{
		SERVOS_Update();
		bServos_RequestUpdate = 0;
	}
}

/**
 * ISR
 *
 */
// __STATIC_INLINE
void SERVOS_ISR(void)
{
	*pServosBSSR = u32ServosBSSRVal;
	//htim6.Instance->ARR = 301;


	htim6.Instance->ARR = u16servosARR;

	servos_actions_cnt++;


//	htim6.Instance->ARR = 100;

	if (servos_actions_cnt >= servos_actions_len)
	{
		servos_actions_cnt = 1;
		bServos_RequestUpdate = 1;
	}
	pServosAction = &servos_actions[servos_actions_cnt];
	u16servosARR = pServosAction->u16ARR;
	pServosBSSR = pServosAction->pServosBSSR;
	u32ServosBSSRVal= pServosAction->u32ServosBSSRVal;
}
