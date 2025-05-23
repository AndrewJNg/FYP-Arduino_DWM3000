#pragma once

#ifndef MAIN_H_
#define MAIN_H_

//#include <avr/io.h> // all the standard AVR functions
#define __DELAY_BACKWARD_COMPATIBLE__ // this enables uint32 to be used in sleep functions

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>
#include <stdio.h>
#include <inttypes.h>
#include "dw3000_uart.h"
#include "dw3000_port.h"
#include "dw3000_device_api.h"
#include "dw3000_shared_functions.h"

#ifndef _BV
#define _BV(n) (1 << n)
#endif
#define __INLINE inline

#endif /* MAIN_H_ */
