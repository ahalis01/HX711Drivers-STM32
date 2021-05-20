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
} HX711_TypeDef;

/***********************************************************************
 * sets microsecond delays
 ************************************************************************/
void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		; // wait for the counter to reach the microsecond input in the parameter
}

/***********************************************************************
 * shifts in the data in accordance to the data sheet
 ************************************************************************/
uint8_t shiftInData(uint8_t bitOrder, HX711_TypeDef instance) {
	uint8_t value = 0;
	uint8_t i;
	HAL_TIM_Base_Start(&htim1);

	//clock must be low before collecting data, DOUT is rising edge sensitive
	HAL_GPIO_WritePin(instance.PDSCK_GPIOx, instance.PDSCK_GPIO_Pin,
			GPIO_PIN_RESET);

	for (i = 0; i < 8; ++i) {
		HAL_GPIO_WritePin(instance.PDSCK_GPIOx, instance.PDSCK_GPIO_Pin,
				GPIO_PIN_SET);
		delay_us(1);

		if (bitOrder) {
			value |= HAL_GPIO_ReadPin(instance.DOUT_GPIOx,
					instance.DOUT_GPIO_Pin) << i;
		} else {
			value |= HAL_GPIO_ReadPin(instance.DOUT_GPIOx,
					instance.DOUT_GPIO_Pin) << (7 - i);
		}
		HAL_GPIO_WritePin(instance.PDSCK_GPIOx, instance.PDSCK_GPIO_Pin,
				GPIO_PIN_RESET);
		delay_us(1);
	}
	return value;
}

/***********************************************************************
 * loads and returns a single load cell reading
 ************************************************************************/
long read(HX711_TypeDef instance) {

	// Wait for the chip to become ready.
	wait_ready(0, instance);

	// Define structures for reading data into.
	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the hx711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read()

	__disable_irq();

	// Pulse the clock pin 24 times to read the data.
	data[2] = shiftInData(0, instance); //zero for the MSB first
	data[1] = shiftInData(0, instance);
	data[0] = shiftInData(0, instance);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < instance.GAIN; i++) {
		HAL_GPIO_WritePin(instance.PDSCK_GPIOx, instance.PDSCK_GPIO_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(instance.PDSCK_GPIOx, instance.PDSCK_GPIO_Pin,
				GPIO_PIN_RESET);
	}

	__enable_irq();

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ((unsigned long) filler << 24 | (unsigned long) data[2] << 16
			| (unsigned long) data[1] << 8 | (unsigned long) data[0]);

	return (long) value;
}

/***********************************************************************
 * returns a weighted average of the readings from the load cell
 ************************************************************************/
long read_average(uint8_t times, HX711_TypeDef instance) {
	long sum = 0;
	for (uint8_t i = 0; i < times; i++) {
		sum += read(instance);
	}
	return sum / times;
}

/***********************************************************************
 * checks if the hx711 is read
 ************************************************************************/
int is_ready(HX711_TypeDef instance) {
	return HAL_GPIO_ReadPin(instance.DOUT_GPIOx, instance.DOUT_GPIO_Pin) == 0;
}

/***********************************************************************
 * Waits until the hx711 is ready
 ************************************************************************/
void wait_ready(unsigned long delay_s, HX711_TypeDef instance) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the code until a load cell is connected.
	while (!is_ready(instance)) {
		HAL_Delay(delay_s);
	}
}

/***********************************************************************
* Sets the GAIN
************************************************************************/
void set_gain(byte gain, HX711_TypeDef instance) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
			instance.GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			instance.GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			instance.GAIN = 2;
			break;
	}

}

/***********************************************************************
* returns the average value with an adjustment for the tare
************************************************************************/
double HX711::get_value(byte times, HX711_TypeDef instance) {
	return read_average(times, instance) - instance.OFFSET;
}

/***********************************************************************
* returns the units
************************************************************************/
float HX711::get_units(byte times, HX711_TypeDef instance) {
	return get_value(times, instance) / instance.SCALE;
}

/***********************************************************************
* Tares, or zeros the scale
************************************************************************/
void HX711::tare(byte times, HX711_TypeDef instance) {
	double sum = read_average(times, instance);
	set_offset(sum, instance);
}

/***********************************************************************
* Sets the scale
************************************************************************/
void HX711::set_scale(float scale, HX711_TypeDef instance) {
	instance.SCALE = scale;
}

/***********************************************************************
* Sets the offset
************************************************************************/
void HX711::set_offset(long offset, HX711_TypeDef instance) {
	instance.OFFSET = offset;
}

//THE FUNCTIONS BELOW HAVE NOT BEEN FULLY TESTED

/***********************************************************************
 * Waits until the hx711 is ready, and tries "retries" number of times
 ************************************************************************/
int wait_ready_retry(int retries, unsigned long delay_ms,
		HX711_TypeDef instance) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	int count = 0;
	while (count < retries) {
		if (is_ready(instance)) {
			return 1;
		}
		HAL_Delay(delay_ms);
		count++;
	}
	return 0;
}

/***********************************************************************
 * Waits until the hx711 is ready, with a time limit
 ************************************************************************/
int wait_ready_timeout(unsigned long timeout, unsigned long delay_ms,
		HX711_TypeDef instance) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/hx711/pull/96
	unsigned long millisStarted = HAL_GetTick();
	while (HAL_GetTick() - millisStarted < timeout) {
		if (is_ready(instance)) {
			return 1;
		}
		HAL_Delay(delay_ms);
	}
	return 0;
}

/***********************************************************************
 * Powers down the hx711 using the PDSCK GPIO pin
 ************************************************************************/
void power_down(HX711_TypeDef instance) {
	HAL_GPIO_WritePin(instance.PDSCK_GPIOx, instance.PDSCK_GPIO_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(instance.PDSCK_GPIOx, instance.PDSCK_GPIO_Pin,
			GPIO_PIN_SET);
}

/***********************************************************************
 * Powers up the hx711 using the PDSCK GPIO pin
 ************************************************************************/
void power_up(HX711_TypeDef instance) {
	HAL_GPIO_WritePin(instance.PDSCK_GPIOx, instance.PDSCK_GPIO_Pin,
			GPIO_PIN_RESET);
}