

#ifndef __RF24_ARCH_CONFIG_H__
#define __RF24_ARCH_CONFIG_H__

#define rf24_max(a, b) (a>b?a:b)
#define rf24_min(a, b) (a<b?a:b)

#include <stddef.h>
#include <stdbool.h>
#include <util/delay.h>

#define MINIMAL
#define LOW  0
#define HIGH 1
#define OUTPUT 1
#define INPUT 0
#define CE_PORT PORTD
#define CE_PIN PIND3
#define CSN_PORT PORTD
#define CSN_PIN PIND4

#endif // __RF24_ARCH_CONFIG_H__

