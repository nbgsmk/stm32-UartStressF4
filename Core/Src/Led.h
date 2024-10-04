/*
 * Led.h
 *
 *  Created on: Sep 27, 2024
 *      Author: peca
 */

#ifndef SRC_LED_H_
#define SRC_LED_H_

//class Led {
//public:
//	Led();
//	virtual ~Led();

	void ledOn(void);
	void ledOff(void);
	void ledTogl(void);

	void ledBlink(uint32_t ticksOn, uint32_t ticksOff);
	void ledBlinkCount(uint32_t count, uint32_t ticksOn, uint32_t ticksOff);
	void ledBlinkPeriod(uint32_t count, uint32_t ticksOn, uint32_t ticksOff, uint32_t ticksTotalPeriod);

	void ledBlink_Rpi(uint32_t ticksOn, uint32_t ticksOff);
	void ledBlink_RpiCount(uint32_t count, uint32_t ticksOn, uint32_t ticksOff);
//};

#endif /* SRC_LED_H_ */
