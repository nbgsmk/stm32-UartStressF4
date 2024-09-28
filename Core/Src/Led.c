/*
 * Led.cpp
 *
 *  Created on: Sep 27, 2024
 *      Author: peca
 */


#include <main.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "Led.h"

//Led::Led(){
//
//}
//
//Led::~Led(){
//
//}


void ledOn(void){
	HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
}

void ledOff(void){
	HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);
}

void ledTogl(void) {
	HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
}

void ledBlink(uint32_t ticksOn, uint32_t ticksOff){
	ledOn();
	osDelay(ticksOn);
	ledOff();
	osDelay(ticksOff);
}

void ledBlinkCount(uint32_t count, uint32_t ticksOn, uint32_t ticksOff){
	for(uint32_t i = 0; i < count; i++){
		ledBlink(ticksOn, ticksOff);
	}
}


void ledBlinkPeriod(uint32_t count, uint32_t ticksOn, uint32_t ticksOff, uint32_t ticksTotalPeriod){
	for(uint32_t i = 0; i < count; i++){
		ledBlink(ticksOn, ticksOff);
	}
	uint32_t treptanje = count * (ticksOn + ticksOff);
	uint32_t ostatak;
	if (treptanje >= ticksTotalPeriod) {
		ostatak = 1;
	} else {
		ostatak = ticksTotalPeriod - (count * (ticksOn + ticksOff));
	}
	osDelay(ostatak);
}

