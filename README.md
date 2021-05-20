# HX711-STM32library
This is a STM32 library for the HX711 breakout board. It is compatible with IDEs that use HAL, such as the STM32CubeIDE.

The c-version folder contains a somewhat stable version that has been tested and used in a project.
To use the c-version of the library with the STM32CubeIDE, the code must be copied and pasted into the main.c file. This must be done for a proper build and compliation of the code.

The src folder contains a cpp version of the library that is incomplete. Using cpp in STM32CubeIDE is not ideal.

I am working on a seperate header file, and c file that can be compiled properly in the STM32CubeIDE. For now, the c-version can be used for quick and dirty solution for prototyping.
