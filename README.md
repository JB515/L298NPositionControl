# L298NPositionControl
This (work in progress) Arduino library implements closed-loop position control of a DC motor, via the popular L298N driver chip. It makes use of Paul Stoffregen's excellent Encoder library, available here: https://github.com/PaulStoffregen/Encoder.

# Hardware
The library is designed to work with any DC motor with a quadrature encoder, as long as it is driven by an L298N driver module. Motors either sold with or connected to gears are also supported.

The driver/encoder can be connected to the Arduino on any pins, provided that the L298N . For best performance, the two encoder connections, encA and encB should be connected to interrupt pins. See [this page](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/) for details. If in doubt, the following wiring is used in the provided examples:

![Hardware schematic used in example programs](img/wiring.png)

# Code
The library provides a number of example programs to demonstrate various aspects of the functionality
