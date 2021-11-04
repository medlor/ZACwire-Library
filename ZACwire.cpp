/*	ZACwire - Library for reading temperature sensors TSIC 206/306/506
	created by Adrian Immer in 2020
	v2.0.0b5
*/

#include "Arduino.h"
#include "ZACwire.h"

#if defined(ESP32) || defined(ESP8266)
	#include <FunctionalInterrupt.h>
#else								//initialize static variables for AVR boards
	uint8_t ZACwire::bitThreshold;
	uint16_t ZACwire::measuredTimeDiff;
	volatile uint8_t ZACwire::bitCounter;
	volatile bool ZACwire::backUP;
	volatile uint16_t ZACwire::rawData[2];
	volatile uint8_t ZACwire::heartbeat;
#endif


ZACwire::ZACwire(uint8_t pin, int16_t sensor) : _pin{pin}, _sensor{sensor} {
	bitCounter = 0;
}


bool ZACwire::begin(uint8_t bitWindow) {			//start collecting data, needs to be called over 2ms before first getTemp()
	pinMode(_pin, INPUT);
	while (pulseIn(_pin, LOW, timeout*2)) yield();		//wait for time without transmission
	uint8_t strobeTime = pulseIn(_pin, LOW);		//check for signal and measure strobeTime
	measuredTimeDiff = micros();				//set timestamp of the rising edge for ISR
	if (!strobeTime) return false;				//check if there is incoming signal
	if (!bitWindow) bitWindow = strobeTime<<1;
	bitThreshold = bitWindow * 1.25;			//expected BitWindow in µs, depends on sensor & temperature
	
	uint8_t isrPin = digitalPinToInterrupt(_pin);
	if (isrPin == 255) return false;
	#if defined(ESP32) || defined(ESP8266)
		attachInterrupt(isrPin, [this]{read();}, RISING);
	#else
		attachInterrupt(isrPin, read, RISING);
	#endif
	return true;
}


float ZACwire::getTemp() {					//return temperature in °C
	if (connectionCheck()) adjustBitThreshold();
	else return errorNotConnected;
	
	if (bitCounter != Bit::finished) useBackup = true;	//when newer reading is incomplete
	uint16_t temp{rawData[backUP^useBackup]};		//get rawData from ISR
	
	if (tempCheck(temp)) {
		temp >>= 1;					//delete second parity bit
		temp = (temp >> 2 & 1792) | (temp & 255);	//delete first    "     "
		if (!prevTemp) prevTemp = temp;
		int16_t grad ((temp - prevTemp) / (heartbeat|1));//grad is [°C/s]
		if (abs(grad) < maxChangeRate) {		//limit change rate to detect misreadings
			prevTemp = temp;
			heartbeat = useBackup = 0;
			if (_sensor < 400) return ((temp * 250L >> 8) - 499) / 10.0;	//use different formula for 206&306 or 506
			else return ((temp * 175L >> 9) - 99) / 10.0;
		}
	}
	useBackup = !useBackup;					//reset useBackup after use
	return useBackup ? getTemp() : errorMisreading;		//restart with backUP rawData or return error value 222
}


void ZACwire::end() {
	detachInterrupt(digitalPinToInterrupt(_pin));
}


void ZACwire::read() {						//get called with every rising edge
	if (++bitCounter >= Bit::lastZero) {			//first 4 bits always =0
		uint16_t microtime = micros();
		measuredTimeDiff = microtime - measuredTimeDiff;//measure time to previous rising edge
		if (measuredTimeDiff >> 10) {			//first start bit, so big time difference
			backUP ^= bitCounter == 20;		//save backup, if it successfully counted 20 bits
			bitCounter = 0;
			++heartbeat;				//give a sign of life to getTemp()
		}
		else if (bitCounter == Bit::lastZero) rawData[backUP] = measuredTimeDiff<<1 | 2;//send measuredTimeDiff for calculating bitThreshold and add prefix "10" to temp
		else {
			if (bitCounter == Bit::afterStop) microtime += bitThreshold>>2;	//convert timestamp at stop bit to normal 0 bit
			if (~rawData[backUP] & 1) measuredTimeDiff += bitThreshold>>1;	//add 1/2 bitThreshold if previous bit was 0 (for normalisation)
			rawData[backUP] <<= 1;
			rawData[backUP] |= measuredTimeDiff < bitThreshold;
		}
		measuredTimeDiff = microtime;
	}
}


bool ZACwire::connectionCheck() {
	if (heartbeat) timeLastHB = 0;
	else {
		if (!bitCounter) {				//use bitCounter to check if begin() was already called
			begin();
			delay(200);
		}
		else if (!timeLastHB) timeLastHB = millis();	//record first missing heartbeat
		else if ((uint16_t)millis() - timeLastHB > timeout) return false;
	}
	return true;
}


void ZACwire::adjustBitThreshold() {
	if (bitCounter >= Bit::lastZero && bitCounter <= Bit::afterStop) {	//adjust bitThreshold just before rawData is filled with temperature
		uint16_t newBitThreshold = (rawData[backUP] >> (bitCounter - 4));//separate newBitThreshold from temperature bits
		newBitThreshold *=  1.25 / 5.25;
		bitThreshold < newBitThreshold ? ++bitThreshold : --bitThreshold;
	}
}


bool ZACwire::tempCheck(uint16_t rawTemp) {					
	if (rawTemp >> 14 != 2 || rawTemp & 512) return false;	//check for prefix "10" and stop bit=0
	
	bool parity{true};
	for (uint8_t i{0}; i<9; ++i) parity ^= rawTemp & 1 << i;
	if (parity) for (uint8_t i{10}; i<14; ++i) parity ^= rawTemp & 1 << i;
	return parity;
}
