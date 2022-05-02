# IR Remote, RS-485 Modbus RTU controller

This is my codebase for a STM32 powered 38kHz IR remote that can 'learn' and then send out specfic signals for day, night and twilight lighting effects for the Exo Terrasky light.
It makes use of the NEC protocol to learn and send commands.

RS-485 Modbus RTU can be used to automate sending these commands, using a Modbus RTU enabled CPU

## Modbus

### Register 40001

Write to this register to change the type of IR command that is sent out to the light.
- 0: 'Day' cycle
- 1: 'Night' cycle
- 2: 'Twilight' cycle

Typically twilight cycle can be called between sunset and sunrise. Modbus commands are debounced every 1 min. Other commands sent within that minute to this register are ignored. It is advised to often send this command to ensure your light functions well even thought sometimes commands get lost because of IR signal problems.

## Learning new light settings

Cycle through the different light settings with the cycle button to the setting you wish to change. Push the learn button. Now press the corresponding key (color or program) on the Terrasky remote control, pointing it to the back of this device. The command will be stored in memory and the device will send out immediately the new commands. Modbus is temporary disabled while in this learning mode.

The device will lost the settings if power is cut off