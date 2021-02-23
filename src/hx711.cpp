#include "hx711.h"
#include "stm32f4xx.h"
#include "stm32f401xe.h"

#if !defined  (STM32F401xE)
#define STM32F401xE
#endif

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

//let MSB first be equal to 0 and LSB first be nER, n!=0 (most likly 1 from user input)
//the data pin will be reading the data
uint8_t shiftInSlow( GPIO_TypeDef * dataGPIOx, uint16_t dataGPIO_Pin, GPIO_TypeDef * clockGPIOx, uint16_t clockGPIO_Pin, int bitOrder) {
    uint8_t value = 0;
    uint8_t i;
	HAL_TIM_Base_Start(&htim1);

    for(i = 0; i < 8; ++i) {
        HAL_GPIO_WritePin(clockGPIOx, clockGPIO_Pin, GPIO_PIN_SET);
        delay_us(1); //one microsecond delay

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

//dont really need? cause the pin mode is set using the gui?
void HX711::begin(GPIO_TypeDef * PD_SCK, uint16_t PD_SCK_pin, GPIO_TypeDef * DOUT, uint16_t DOUT_pin, byte gain) {
	PDSCK_GPIOx = PD_SCK;
	PDSCK_GPIO_Pin = PD_SCK_pin;
	DOUT_GPIOx = DOUT;
	DOUT_GPIO_Pin = DOUT_pin;

	//pinMode(PD_SCK, OUTPUT);
	//pinMode(DOUT, INPUT_PULLUP);
	//just do it using the GUI

	set_gain(gain);
}

//change to use HAL
bool HX711::is_ready() {
	return HAL_GPIO_ReadPin(DOUT_GPIOx, DOUT_GPIO_Pin) == 0;
}

//from the datasheet, related to the number of pulses
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
	// corrupting the value returned by read().  The ATOMIC_BLOCK macro disables
	// interrupts during the sequence and then restores the interrupt mask to its previous
	// state after the sequence completes, insuring that the entire read-and-gain-set
	// sequence is not interrupted.  The macro has a few minor advantages over bracketing
	// the sequence between `noInterrupts()` and `interrupts()` calls.
	
	__disable_irq();
	
	//change to suit HAL
	// Pulse the clock pin 24 times to read the data.
	data[2] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT_GPIOx, DOUT_GPIO_Pin, PDSCK_GPIOx, PDSCK_GPIO_Pin, 0);
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

void HX711::wait_ready(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!is_ready()) {
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		HAL_Delay(delay_ms);
	}
}

bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	// https://github.com/bogde/HX711/issues/76
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

//https://stackoverflow.com/questions/37375602/arduino-millis-in-stm32
//change the millis() to use systick
//tbh I dont really need this, but its good to include
bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (is_ready()) {
			return true;
		}
		HAL_Delay(delay_ms);
	}
	return false;
}

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

double HX711::get_value(byte times) {
	return read_average(times) - OFFSET;
}

float HX711::get_units(byte times) {
	return get_value(times) / SCALE;
}

void HX711::tare(byte times) {
	double sum = read_average(times);
	set_offset(sum);
}

void HX711::set_scale(float scale) {
	SCALE = scale;
}

float HX711::get_scale() {
	return SCALE;
}

void HX711::set_offset(long offset) {
	OFFSET = offset;
}

long HX711::get_offset() {
	return OFFSET;
}

void HX711::power_down() {
	HAL_GPIO_WritePin(PDSCK_GPIOx, PDSCK_GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PDSCK_GPIOx, PDSCK_GPIO_Pin, GPIO_PIN_SET);
}

void HX711::power_up() {
	HAL_GPIO_WritePin(PDSCK_GPIOx, PDSCK_GPIO_Pin, GPIO_PIN_RESET);
}
