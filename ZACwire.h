/*	ZACwire - Library for reading temperature sensors TSIC 206/306/506
	created by Adrian Immer in 2020
	temporary fixed by medlor: v1.3.1 code put into 1.3.3 base 
*/

#ifndef ZACwire_h
#define ZACwire_h

#include "Arduino.h"

#ifdef ARDUINO_ESP8266_RELEASE_
	#include <gpio.h>
	#warning "Arduino ESP8266 3.0.0 has issues with IRAM. Please downgrade to 2.7.4 for more efficient memory usage"
	void IRAM_ATTR tsicIsrHandler(uint8_t gpio);
#endif


template <uint8_t pin>
class ZACwire {
	
	friend void tsicIsrHandler(uint8_t);
	
	public:

		ZACwire(int sensor = 306, byte defaultBitWindow=125, bool core=1) {
			_Sensortype = sensor;
			bitWindow = defaultBitWindow + (defaultBitWindow>>2);		//expected BitWindow in µs, depends on sensor & temperature
			_core = core;							//only ESP32: choose cpu0 or cpu1
		}

		bool begin() {								//start collecting data, needs to be called 100+ms before first getTemp()
			pinMode(pin, INPUT);
			if (!pulseInLong(pin, LOW)) return false;			//check if there is incoming signal
			isrPin = digitalPinToInterrupt(pin);
			if (isrPin == 255) return false;
			#ifdef ESP32
			xTaskCreatePinnedToCore(attachISR_ESP32,"attachISR",2000,NULL,1,NULL,_core); //freeRTOS^
			#elif defined(ARDUINO_ESP8266_RELEASE_)				//In ARDUINO_ESP8266_RELEASE_3.0.0 a new version of gcc with bug 70435
			ETS_GPIO_INTR_ATTACH(tsicIsrHandler,(intptr_t)isrPin);		//...is included, which has issues with IRAM and template classes.
			gpio_pin_intr_state_set(isrPin,GPIO_PIN_INTR_POSEDGE);		//...That's the reason to use nonOS here
			ETS_GPIO_INTR_ENABLE();
			#else
			attachInterrupt(isrPin, read, RISING);
			#endif
			return true;
		}
	  
	float getTemp() {								//give back temperature in °C
		static bool misreading;
		uint16_t temp[2];
		if ((unsigned int)millis() - lastISR > 255) {	//check wire connection for the last 255ms
			if (bitWindow) return 221;				// temp=221 if sensor not connected
			else {									// w/o bitWindow, begin() wasn't called before
				begin();
				delay(110);
			}
		}
		if (BitCounter == 19) {
			byte newBitWindow = ((ByteTime << 1) + ByteTime >> 5) + (range >> 1);  //divide by around 10.5 and add half range
			bitWindow < newBitWindow ? ++bitWindow : --bitWindow;	//adjust bitWindow
		}
		else misreading = true;						//use misreading-backup when newer reading is incomplete
		bool _backUP = backUP^misreading;
		temp[0] = rawTemp[0][_backUP];				//get high significant bits from ISR
		temp[1] = rawTemp[1][_backUP];				//get low   ''    ''
		misreading = !misreading;					//reset misreading after use
		
		bool parity = true;
		for (byte j=0; j<2; ++j) {
			for (byte i=0; i<9; ++i) parity ^= temp[j] & 1 << i;	//check parity
			temp[j] >>= 1;							//delete parity bit
		}
		if (parity) { 								//check for failure
			temp[1] |= temp[0] << 8;				//join high and low significant figures
			misreading = false;
			if (_Sensortype < 400) return float((temp[1] * 250L >> 8) - 499) / 10;  //calculate °C
			else return float((temp[1] * 175L >> 9) - 99) / 10;
		}
		else if (misreading) return getTemp();		//restart with backUP raw temperature
		return 222;									//temp=222 if reading failed
	}

		void end() {
			detachInterrupt(digitalPinToInterrupt(pin));
		}
	
	private:

		#ifdef ESP32
		static void attachISR_ESP32(void *arg){			//attach ISR in freeRTOS because arduino can't attachInterrupt() inside of template class
			gpio_pad_select_gpio((gpio_num_t)isrPin);
			gpio_set_direction((gpio_num_t)isrPin, GPIO_MODE_INPUT);  //TOBI NEW
			gpio_set_intr_type((gpio_num_t)isrPin, GPIO_INTR_POSEDGE);
			gpio_install_isr_service(0);
			gpio_isr_handler_add((gpio_num_t)isrPin, read, NULL);
			vTaskDelete(NULL);
		}
		static void IRAM_ATTR read(void*) {
		#elif defined(ARDUINO_ESP8266_RELEASE_)
		static inline void read() __attribute__((always_inline)) {
			uint16 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);	//get GPIO that caused the interrupt
			if (gpio_status != BIT(pin)) return;				//check if the right GPIO triggered
			GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS,gpio_status);		//clear interrupt flag
		#elif defined (ESP8266)
		static void ICACHE_RAM_ATTR read() {
		#else
		static void read() {							//get called with every rising edge
		#endif
			if (++BitCounter > 4) {						//first 4 bits always =0
				static bool ByteNr;
				unsigned int microtime = micros();
				static unsigned int deltaTime;
				deltaTime = microtime - deltaTime;		//measure time to previous rising edge
				if (deltaTime >> 10) {					//true at start bit
					ByteTime = microtime;				//for measuring Tstrobe/bitWindow
					backUP = !backUP;
					BitCounter = 0;
					lastISR = millis();					//for checking wire connection
				}
				else if (BitCounter == 6) rawTemp[ByteNr][backUP] = ByteNr = 0;
				else if (BitCounter == 10) {			//after stop bit
					ByteTime = microtime - ByteTime;
					rawTemp[ByteNr = 1][backUP] = 0;
				}
				rawTemp[ByteNr][backUP] <<= 1;      
				if (deltaTime > bitWindow);				//Logic 0
				else if (rawTemp[ByteNr][backUP] & 2 || deltaTime < bitWindow - (range >> (BitCounter==11))) rawTemp[ByteNr][backUP] |= 1;  //Logic 1
				deltaTime = microtime;
			}
		}
		
		static int isrPin;
		bool _core;
	
		int _Sensortype;
		static volatile byte BitCounter;
		static volatile unsigned int ByteTime;
		static volatile uint16_t rawTemp[2][2];
		static byte bitWindow;
		static const byte range = 62;
		static volatile bool backUP;
		static volatile unsigned int lastISR;
};

template<uint8_t pin>
volatile byte ZACwire<pin>::BitCounter = 20;
template<uint8_t pin>
volatile unsigned int ZACwire<pin>::ByteTime;
template<uint8_t pin>
volatile bool ZACwire<pin>::backUP;
template<uint8_t pin>
volatile uint16_t ZACwire<pin>::rawTemp[2][2];
template<uint8_t pin>
int ZACwire<pin>::isrPin;
template<uint8_t pin>
byte ZACwire<pin>::bitWindow;
template<uint8_t pin>
volatile unsigned int ZACwire<pin>::lastISR;

#ifdef ARDUINO_ESP8266_RELEASE_
void tsicIsrHandler(uint8_t gpio) {
	switch (gpio) {
		case 2:
			ZACwire<2>::read();
			break;
		case 4:
			ZACwire<4>::read();
			break;
		case 5:
			ZACwire<5>::read();
			break;
		case 12:
			ZACwire<12>::read();
			break;
		case 13:
			ZACwire<13>::read();
			break;
		case 14:
			ZACwire<14>::read();
			break;
	}
}
#endif

#endif
