/**
******************************************************************************
* acknowledgment: This Library is based on https://github.com/bogde/HX711
* @file    hx711.cpp v1.0
* @author  Amjad Halis
* @brief   HX711 class implementation
*            
*          This file is the unique include file that the application programmer
*          is using in the C source code, usually in main.c. This file contains:
*           - Configuration section that allows to select:
*              - The STM32F4xx device used in the target application
*              - To use or not the peripheral�s drivers in application code(i.e. 
*                code will be based on direct access to peripheral�s registers 
*                rather than drivers API), this option is controlled by 
*                "#define USE_HAL_DRIVER"
******************************************************************************
*/

#include "HX711Driver.hpp"

#ifdef STM32F401xE
#include "stm32f4xx.h"
#endif

#define GET_VARIABLE_NAME(Variable) (#Variable)

/***********************************************************************
* Implements a delay expressed in microseconds using timer1
************************************************************************/
void delay_us (uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the microsecond input in the parameter
}


/***********************************************************************
* Shifting in data at the correct speed as outlined in the datasheet
************************************************************************/
//let MSB first be equal to 0 and LSB first be 1 (or any none zero number)
uint8_t shiftInSlow(GPIO_TypeDef * dataGPIOx, uint16_t dataGPIO_Pin, GPIO_TypeDef * clockGPIOx, uint16_t clockGPIO_Pin, int bitOrder) {
    uint8_t value = 0;
    uint8_t i;
	HAL_TIM_Base_Start(&htim1);

	//clock must be low before collecting data, DOUT is rising edge sensitive
	HAL_GPIO_WritePin(clockGPIOx, clockGPIO_Pin, GPIO_PIN_RESET);

    for(i = 0; i < 8; ++i) {
        HAL_GPIO_WritePin(clockGPIOx, clockGPIO_Pin, GPIO_PIN_SET);
        delay_us(1);

        if(bitOrder)
            value |= HAL_GPIO_ReadPin(dataGPIOx, dataGPIO_Pin) << i;
        else
            value |= HAL_GPIO_ReadPin(dataGPIOx, dataGPIO_Pin) << (7 - i);
		HAL_GPIO_WritePin(clockGPIOx, clockGPIO_Pin, GPIO_PIN_RESET);
        delay_us(1);
    }
    return value;
}

#define SHIFTIN_WITH_SPEED_SUPPORT(dataGPIOx,dataGPIO_Pin,clockGPIOx,clockGPIO_Pin,bitOrder) shiftInSlow(dataGPIOx,dataGPIO_Pin,clockGPIOx,clockGPIO_Pin,bitOrder)

HX711::HX711() {
}

HX711::~HX711() {
}

/***********************************************************************
* Switch cases for initializing GPIO clocks
************************************************************************/
void enableClock(char* gpioType){
	switch (gpioType){
	    case "GPIOA":
			__HAL_RCC_GPIOA_CLK_ENABLE();
			break;
	    case "GPIOB":
    		__HAL_RCC_GPIOB_CLK_ENABLE();
    		break;
	    case "GPIOC":
			__HAL_RCC_GPIOC_CLK_ENABLE();
			break;
	    case "GPIOD":
    		__HAL_RCC_GPIOD_CLK_ENABLE();
    		break;
	   case "GPIOE":
			__HAL_RCC_GPIOE_CLK_ENABLE();
			break;
	    case "GPIOF":
    		__HAL_RCC_GPIOF_CLK_ENABLE();
    		break;
	    case "GPIOG":
			__HAL_RCC_GPIOG_CLK_ENABLE();
			break;
	    case "GPIOH":
    		__HAL_RCC_GPIOH_CLK_ENABLE();
    		break;
    	case "GPIOI":
			__HAL_RCC_GPIOI_CLK_ENABLE();
			break;
    	case "GPIOJ":
    		__HAL_RCC_GPIOJ_CLK_ENABLE();
    		break;
	    case "GPIOK":
			__HAL_RCC_GPIOK_CLK_ENABLE();
			break;
	    default:
    		break;
	}
}

//Make sure to set PD_SCK as a GPIO, and DOUT as INPUT_PULLUP in the cube GUI. 
/***********************************************************************
* Initialize the private class variables
************************************************************************/
void HX711::begin(GPIO_TypeDef * PD_SCK, uint16_t PD_SCK_pin, GPIO_TypeDef * DOUT, uint16_t DOUT_pin, byte gain) {
	PDSCK_GPIOx = PD_SCK;
	PDSCK_GPIO_Pin = PD_SCK_pin;
	DOUT_GPIOx = DOUT;
	DOUT_GPIO_Pin = DOUT_pin;

	char* PD_SCK_GPIOtype = GET_VARIABLE_NAME(PD_SCK);
	char* DOUT_GPIOtype = GET_VARIABLE_NAME(DOUT);

	enableClock(PD_SCK_GPIOtype);
	enableClock(DOUT_GPIOtype);

	HAL_GPIO_WritePin(PD_SCK, PD_SCK_pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = PD_SCK_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PD_SCK, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = DOUT_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DOUT, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	set_gain(gain);
}

/***********************************************************************
* Ensures the dataout clock is zero
************************************************************************/
bool HX711::is_ready() {
	return HAL_GPIO_ReadPin(DOUT_GPIOx, DOUT_GPIO_Pin) == 0;
}

/***********************************************************************
* Sets the GAIN (a private variable) 
************************************************************************/
void HX711::set_gain(byte gain) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}

}

/***********************************************************************
* Reads data from the DOUT pin
************************************************************************/
long HX711::read() {

	// Wait for the chip to become ready.
	wait_ready();

	// Define structures for reading data into.
	unsigned long value = 0;
 	uint8_t filler = 0x00;

	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read()
	
	__disable_irq();
	
	// Pulse the clock pin 24 times to read the data.
	data[2] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT_GPIOx, DOUT_GPIO_Pin, PDSCK_GPIOx, PDSCK_GPIO_Pin, 0); //zero for the MSB first
	data[1] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT_GPIOx, DOUT_GPIO_Pin, PDSCK_GPIOx, PDSCK_GPIO_Pin, 0);
	data[0] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT_GPIOx, DOUT_GPIO_Pin, PDSCK_GPIOx, PDSCK_GPIO_Pin, 0);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < GAIN; i++) {
		HAL_GPIO_WritePin(PDSCK_GPIOx, PDSCK_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PDSCK_GPIOx, PDSCK_GPIO_Pin, GPIO_PIN_RESET);
	}

	__enable_irq();

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<unsigned long>(filler) << 24
			| static_cast<unsigned long>(data[2]) << 16
			| static_cast<unsigned long>(data[1]) << 8
			| static_cast<unsigned long>(data[0]) );

	return static_cast<long>(value);
}

/***********************************************************************
* Waits until the HX711 is ready
************************************************************************/
void HX711::wait_ready(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the code until a load cell is connected.
	while (!is_ready()) {
		HAL_Delay(delay_ms);
	}
}

/***********************************************************************
* Waits until the HX711 is ready, and tries "retries" number of times
************************************************************************/
bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	int count = 0;
	while (count < retries) {
		if (is_ready()) {
			return true;
		}
		HAL_Delay(delay_ms);
		count++;
	}
	return false;
}

/***********************************************************************
* Waits until the HX711 is ready, with a time limit
************************************************************************/
bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = HAL_GetTick();
	while (HAL_GetTick() - millisStarted < timeout) {
		if (is_ready()) {
			return true;
		}
		HAL_Delay(delay_ms);
	}
	return false;
}

/***********************************************************************
* reads the average of the values
************************************************************************/
long HX711::read_average(byte times) {
	long sum = 0;
	for (byte i = 0; i < times; i++) {
		sum += read();
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		HAL_Delay(0);
	}
	return sum / times;
}

/***********************************************************************
* returns the average value with an adjustment for the tare
************************************************************************/
double HX711::get_value(byte times) {
	return read_average(times) - OFFSET;
}

/***********************************************************************
* returns the units
************************************************************************/
float HX711::get_units(byte times) {
	return get_value(times) / SCALE;
}

/***********************************************************************
* Tares, or zeros the scale
************************************************************************/
void HX711::tare(byte times) {
	double sum = read_average(times);
	set_offset(sum);
}

/***********************************************************************
* Sets the scale
************************************************************************/
void HX711::set_scale(float scale) {
	SCALE = scale;
}

/***********************************************************************
* Returns the scale
************************************************************************/
float HX711::get_scale() {
	return SCALE;
}

/***********************************************************************
* Sets the offset
************************************************************************/
void HX711::set_offset(long offset) {
	OFFSET = offset;
}

/***********************************************************************
* Returns the offset
************************************************************************/
long HX711::get_offset() {
	return OFFSET;
}

/***********************************************************************
* Powers down the HX711 using the PDSCK GPIO pin
************************************************************************/
void HX711::power_down() {
	HAL_GPIO_WritePin(PDSCK_GPIOx, PDSCK_GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PDSCK_GPIOx, PDSCK_GPIO_Pin, GPIO_PIN_SET);
}

/***********************************************************************
* Powers up the HX711 using the PDSCK GPIO pin
************************************************************************/
void HX711::power_up() {
	HAL_GPIO_WritePin(PDSCK_GPIOx, PDSCK_GPIO_Pin, GPIO_PIN_RESET);
}