# HX711Driver-STM32
This is a STM32 driver for the HX711 chip. It uses and is compatible with the STM32 HAL.
This library was designed using a HX711 breakout board. It has been tested with a STM32F4 Nucleo board.

To use the driver:
1. Include the header file.
2. Set an input GPIO pin, an output GPIO pin and a 1MHz (set to correct prescaler) timer using the CubeMx software.
3. Create a HX711_struct_TypeDef object, and use the functions as described in the header and source files.
