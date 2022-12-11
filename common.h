#ifndef _COMMON_HEADER
#define _COMMON_HEADER

// #define GPIO(port) ((struct gpio *) (0x40020000 + 0x400 * (bank)))

// use a `uint16` integer to represent a pin.
// a pin contains a port name and a pin number,
// as the port name ranges from A to G and the pin number ranges from 0 to 15,
// both less than 255.
// so it can form the structure `0xPPNN`.
#define Pin(port, number) ( (((port) - 'A') << 8) | (number))

#define get_pin_port(pin) ((pin) >> 8)
#define get_pin_number(pin) ((pin) & 0xff)


#endif