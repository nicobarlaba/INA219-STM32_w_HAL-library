# INA219_STM32_F1
Here's a basic library for the INA219 by Texas, C written compatible with STM32. It uses HAL library to manage i2c communication.

Basically there are some "set_Config" which you can amply based on you necessities; there are 32V_1A/2A/25A 16V_400mA. I wrote this library starting from the Adafruit one so you might see some similarities on the name. Of course there's a lot which can be done better, but it's just a starting point :)

By default it uses HAL for STM32F1, but it is still full compatible with other STM32 just need to change the #include in INA219.h file to include HAL for your MCU

Feel free to contribute
