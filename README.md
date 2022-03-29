# IR Remote, RS-485 Modbus RTU controller

This is my codebase for a STM32 powered 38kHz IR remote that can 'learn' and then send out specfic signals for day, night and twilight lighting effects.
It makes use of the NEC protocol to learn and send commands.

RS-485 Modbus RTU can be used to automate sending these commands, using a Modbus RTU enabled CPU

## Learning NEC commands

This module uses TIM3 CHANNEL 1 in input capture mode to learn the commands to be sent. All capturing is done using the DMA, so essentially there is nothing
in the main `while()` loop.