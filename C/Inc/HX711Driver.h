#ifndef HX711_DRIVER_H
#define HX711_DRIVER_H

#include "stm32f4xx_hal.h"

typedef struct { //struct made for storing load cell data
	GPIO_TypeDef *PDSCK_GPIOx; //clock gpio port
	uint16_t PDSCK_GPIO_Pin; //clock gpio pin number

	GPIO_TypeDef *DOUT_GPIOx; //data gpio port
	uint16_t DOUT_GPIO_Pin; //data gpio pin number

	uint8_t GAIN; //gain constant used for setting the gain of the amplifier
	//1 == 128, 3 == 64, 2 == 32
	//the data sheet calls for a certain number of clock pulses to set the gain, that is why the gain is set in terms of 1, 2 or 3
	long OFFSET; //the offset variable is used for taring the load cell
	float SCALE; //the scale variable is used for converting the raw reading into a specific unit (ie grams, pounds, etc)
	TIM_HandleTypeDef timer; //timer needs to be set to 1MHz, i.e. clock is 84Mhz, so prescaler is 84-1. 84Mhz/84 = 1MHz
} HX711_struct_TypeDef;

void delay_us(uint16_t, HX711_struct_TypeDef); //this function is used to cause a microsecond delay
uint8_t shiftInData(uint8_t, HX711_struct_TypeDef); //this function is made to shift the data into the nucleo in accordance with the specifications on the data sheet
long read(HX711_struct_TypeDef); //this function reads one reading from the load cell
long read_average(uint8_t, HX711_struct_TypeDef); //this function reads an average of multiple reading from the load cell
int is_ready(HX711_struct_TypeDef);
void wait_ready(unsigned long, HX711_struct_TypeDef); //wait until the hx711 chip is ready
int wait_ready_retry(int, unsigned long, HX711_struct_TypeDef); //wait until the hx711 is ready for a certain amount of tries
int wait_ready_timeout(unsigned long, unsigned long, HX711_struct_TypeDef); //wait until the hx711 is ready for a certain amount of time
void power_down(HX711_struct_TypeDef); //powers down the hx711 using the PDSCK GPIO pin
void power_up(HX711_struct_TypeDef); //powers up the hx711 using the PDSCK GPIO pin

#endif
