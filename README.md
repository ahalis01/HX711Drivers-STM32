<<<<<<< HEAD
# HX711Driver-STM32
This is a STM32 driver for the HX711 chip. It uses and is compatible with the STM32 HAL.
This library was designed using a HX711 breakout board. It has been tested with a STM32F4 Nucleo board.

To use the driver:
1. Include the header file.
2. Set an input GPIO pin, an output GPIO pin and a 1MHz (set to correct prescaler) timer using the CubeMx software.
3. Create a HX711_TypeDef object, and use the functions as described in the header and source files.
=======
# HX711-STM32library
This is a STM32 library for the HX711 breakout board. It is compatible with IDEs that use HAL, such as the STM32CubeIDE.

The c-version folder contains a stable version that has been tested and used in a project.
To use the c-version of the library with the STM32CubeIDE, the code must be copied and pasted into the main.c file.

The src folder contains a cpp version of the library that is incomplete. Using cpp in STM32CubeIDE is not ideal.

I am working on a seperate header file, and c file that can be compiled properly in the STM32CubeIDE. For now, the c-version can be used for quick and dirty solution for prototyping.
>>>>>>> 5a46a77838a83f071fd042a87cd7d29a1d3dfd20
