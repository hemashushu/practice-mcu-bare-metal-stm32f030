#ifndef _PIN_HEADER
#define _PIN_HEADER

#define BUILTIN_LED_PIN Pin('C', 13) // PC13
#define EXTERNAL_LED_PIN Pin('B', 9) // PB9

#define BUTTON_0_PIN Pin('B', 0) // PB0
#define BUTTON_1_PIN Pin('B', 1) // PB1

#define TX_PIN Pin('A', 9)
#define RX_PIN Pin('A', 10)

#define SCL_PIN Pin('B', 6) // PB6
#define SDA_PIN Pin('B', 7) // PB7

#define EEPROM_ADDR (0xA0) // 0xA0

#endif