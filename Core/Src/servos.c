/*
 *  Project:      168servos
 *  File:         servos.c
 *  Author:       Gerd Bartelt - www.sebulli.com
 *
 *  Description:  servos module
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


/* Includes ------------------------------------------------------------------*/
#include "servos.h"
#include "tim.h"

/* Variables -----------------------------------------------------------------*/
uint8_t servos_au8Indices[SERVOS_TOTAL];
int servos_iCnt;
SERVO_s servos_asAll[SERVOS_TOTAL];
SERVO_PARALLEL_s servos_asParallel[SERVOS_SERIAL];
SERVO_ACTION_s servos_asActions[2*SERVOS_TOTAL+2];

int servos_iActionCnt;
int servos_iActionLen;
int servos_bRequestUpdate;
uint16_t servos_u16ARR;
uint32_t servos_u32BSSRVal;
__IO uint32_t* servos_pBSSR;
__IO uint32_t servos_u32DummyBSSR;


/* Prototypes of static function ---------------------------------------------*/
static void SERVOS_Update(void);
__STATIC_INLINE void SERVOS_Swap(SERVO_s* thisServo, SERVO_s* otherServo, SERVO_PARALLEL_s* servosParallel);


/* Functions -----------------------------------------------------------------*/

/**
 * Initialize this module
 * Initializes the data fields and starts the timer with an empty sequence
 * with 20ms period time
 *
 */
void SERVOS_Init()
{
	servos_iCnt = 0;
	for (int i=0; i<SERVOS_TOTAL; i++)
	{
		servos_au8Indices[i] = SERVOS_ID_EMPTY;
	}
	for (int i=0; i<SERVOS_SERIAL; i++)
	{
		servos_asParallel[i].cnt = 0;
	}

	servos_asActions[0].pServosBSSR = &(servos_u32DummyBSSR);
	servos_asActions[0].u32ServosBSSRVal = 0;
	servos_asActions[0].u16ARR = 0 ;

	servos_pBSSR      = servos_asActions[1].pServosBSSR = &(servos_u32DummyBSSR);
	servos_u32BSSRVal = servos_asActions[1].u32ServosBSSRVal = 0;
	servos_u16ARR     = servos_asActions[1].u16ARR = 1000-1;

	servos_iActionCnt = 1;
	servos_iActionLen = 2;

	servos_bRequestUpdate = 0;
	HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * Add a new servo
 *
 * \param u8ServoId a unique ID of the servo from 0..167 to identify it
 * \param u16GpioPin pin of the servo pin
 * \param pGpioPort port of the servo pin
 *
 * \return error result
 *
 */
int SERVOS_Configure(uint8_t u8ServoId, uint16_t u16GpioPin, GPIO_TypeDef* pGpioPort)
{
	int iSerialIx;
	int iParallelIx;

	// Check range of u8ServoId
	if (u8ServoId >= SERVOS_TOTAL)
	{
		return SERVOS_ERROR_ID_OUT_OF_RANGE;
	}

	// configure a servo only one time
	if (servos_au8Indices[u8ServoId] != SERVOS_ID_EMPTY)
	{
		return SERVOS_ERROR_ALREADY_CONFIGURED;
	}

	// Add servoID to table
	servos_au8Indices[u8ServoId] = servos_iCnt;
	servos_asAll[servos_iCnt].value = SERVOS_DEFAULT_VAL;
	servos_asAll[servos_iCnt].attached = 0;
	servos_asAll[servos_iCnt].servoIndex = servos_iCnt;
	servos_asAll[servos_iCnt].pGpioPort = pGpioPort;
	servos_asAll[servos_iCnt].u16GpioPin = u16GpioPin;

	// update the parallel counter
	iSerialIx = servos_iCnt & SERVOS_SERIAL_MASK;
	iParallelIx = servos_iCnt >> SERVOS_SERIAL_SHIFT;
	servos_asParallel[iSerialIx].cnt = iParallelIx + 1;
	servos_asParallel[iSerialIx].servos[iParallelIx] = servos_iCnt;
	servos_asAll[servos_iCnt].parallelSortPos = iParallelIx;

	//added
	servos_iCnt++;

	return SERVOS_ERROR_OK;
}

/**
 * Swaps 2 entries of the parallel structure to sort it
 *
 * \param thisServo one servo
 * \param otherServo and the other servo to swap
 * \param servosParallel the current parallel structure
 *
 */
__STATIC_INLINE void SERVOS_Swap(SERVO_s* thisServo, SERVO_s* otherServo, SERVO_PARALLEL_s* servosParallel)
{
	// Swap
	thisServo->parallelSortPos--;
	otherServo->parallelSortPos++;
	servosParallel->servos[thisServo->parallelSortPos] = thisServo->servoIndex;
	servosParallel->servos[otherServo->parallelSortPos] = otherServo->servoIndex;
}


/**
 * Set the value of a servo, attach it if not and sort the parallel structure
 *
 * \param u8ServoId Id of the servo
 * \param u32Value the new value
 *
 * \return error result
 *
 */
int SERVOS_Set(uint8_t u8ServoId, uint32_t u32Value)
{
	int servoindex;
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


	// Get reference of the servo
	servoindex = servos_au8Indices[u8ServoId];
	// Check if configured
	if (servoindex == SERVOS_ID_EMPTY)
	{
		return SERVOS_ERROR_NOT_CONFIGURED;
	}
	thisServo = &servos_asAll[servoindex];

	// Attach it if not
	thisServo->attached = 1;

	// has the value changed?
	if (thisServo->value != u32Value)
	{
		thisServo->value = u32Value;

		// update the parallel counter
		serialIndex = servoindex & SERVOS_SERIAL_MASK;
		//parallelIndex = servos_cnt >> SERVOS_SERIAL_SHIFT;
		servosParallel = &servos_asParallel[serialIndex];

		// Are there more than 1 servos, we have to sort them
		if ( servosParallel->cnt > 1)
		{
			// sort downwards
			do
			{
				bFound = 0;

				if (thisServo->parallelSortPos > 0)
				{
					otherServo = &servos_asAll[servosParallel->servos[(thisServo->parallelSortPos)-1]];
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
					otherServo = &servos_asAll[servosParallel->servos[(thisServo->parallelSortPos)+1]];
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
 * \param u8ServoId Id of the servo
 *
 * \return error result
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
	servoindex = servos_au8Indices[u8ServoId];
	if (servoindex == SERVOS_ID_EMPTY)
	{
		return SERVOS_ERROR_NOT_CONFIGURED;
	}

	servos_asAll[servoindex].attached = 0;

	return SERVOS_ERROR_OK;
}

/**
 * Attach a servo
 *
 * \param u8ServoId Id of the servo
 *
 * \return error result
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
	servoindex = servos_au8Indices[u8ServoId];
	if (servoindex == SERVOS_ID_EMPTY)
	{
		return SERVOS_ERROR_NOT_CONFIGURED;
	}

	servos_asAll[servoindex].attached = 1;

	return SERVOS_ERROR_OK;
}


/**
 * Update the action table for the ISR
 *
 */
static void SERVOS_Update(void)
{
	int iTimeUs = 0;
	int iTimeUsLast = 0;
	int iActionCnt;
	int iParallelPulsesTime;
	SERVO_s* psServo;
	SERVO_PARALLEL_s* psServosParallel;

	iActionCnt = 1;
	for (int s=0; s<SERVOS_SERIAL; s++)
	{
		psServosParallel = &servos_asParallel[s];
		iParallelPulsesTime = s * SERVOS_TIME_PULSE_GRID;

		// Set the action for all positive edged of the servo pulses that
		// are controlled in parallel
		for (int p=0; p<psServosParallel->cnt; p++)
		{
			psServo = &servos_asAll[psServosParallel->servos[p]];
			iTimeUs = p * SERVOS_TIME_OFFSET_PARALLEL + iParallelPulsesTime;
			servos_asActions[iActionCnt].pServosBSSR = &psServo->pGpioPort->BSRR;
			if (psServo->attached)
			{
				servos_asActions[iActionCnt].u32ServosBSSRVal = psServo->u16GpioPin;
			}
			else
			{
				servos_asActions[iActionCnt].u32ServosBSSRVal = 0;
			}
			servos_asActions[iActionCnt-1].u16ARR = iTimeUs - iTimeUsLast - 1;
			iTimeUsLast = iTimeUs;
			iActionCnt++;
		}
		// Set the action for all negative edged of the servo pulses that
		// are controlled in parallel
		for (int p=0; p<psServosParallel->cnt; p++)
		{
			psServo = &servos_asAll[psServosParallel->servos[p]];
			iTimeUs = psServo->value + SERVOS_TIME_PULSE_BASE
					+  p * SERVOS_TIME_OFFSET_PARALLEL + iParallelPulsesTime;
			servos_asActions[iActionCnt].pServosBSSR = &psServo->pGpioPort->BSRR;
			servos_asActions[iActionCnt].u32ServosBSSRVal = (uint32_t)psServo->u16GpioPin << 16;
			servos_asActions[iActionCnt-1].u16ARR = iTimeUs - iTimeUsLast - 1;
			iTimeUsLast = iTimeUs;
			iActionCnt++;
		}
	}

	// Set the last action to a dummy but with time to get a 20ms period
	iTimeUs = SERVOS_TIME_PERIODE;
	servos_asActions[iActionCnt].pServosBSSR = &servos_u32DummyBSSR;
	servos_asActions[iActionCnt].u32ServosBSSRVal = 0;
	servos_asActions[iActionCnt-1].u16ARR = iTimeUs - iTimeUsLast - 1;
	servos_iActionLen = iActionCnt;
}


/**
 * Call this function every 1ms from main
 *
 * It starts the creation of the action tabled triggered from the ISR
 *
 */
void SERVOS_Task1ms()
{
	if (servos_bRequestUpdate)
	{
		SERVOS_Update();
		servos_bRequestUpdate = 0;
	}
}

/**
 * ISR
 *
 * Generates a pulse on one of the 168 GPIOs
 *
 */
__INLINE void SERVOS_ISR(void)
{
	// set port value as fast as possible
	*servos_pBSSR = servos_u32BSSRVal;
	// set timer for next edge
	htim6.Instance->ARR = servos_u16ARR;

	// Get the next action for the next IRQ
	servos_iActionCnt++;
	if (servos_iActionCnt >= servos_iActionLen)
	{
		servos_iActionCnt = 1;
		servos_bRequestUpdate = 1;
	}
	servos_u16ARR = servos_asActions[servos_iActionCnt].u16ARR;
	servos_pBSSR = servos_asActions[servos_iActionCnt].pServosBSSR;
	servos_u32BSSRVal= servos_asActions[servos_iActionCnt].u32ServosBSSRVal;
}
