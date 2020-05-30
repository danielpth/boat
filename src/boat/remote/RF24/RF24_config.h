
/*
 Copyright (C) 
    2011            J. Coliz <maniacbug@ymail.com>
    2015-2019 TMRh20
    2015            spaniakos <spaniakos@gmail.com>
    2015            nerdralph
    2015            zador-blood-stained
    2016            akatran
    2017-2019 Avamander <avamander@gmail.com>
    2019            IkpeohaGodson
                 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
*/

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__
/*** USER DEFINES:    ***/    
//#define FAILURE_HANDLING
//#define SERIAL_DEBUG
//#define MINIMAL
//#define SPI_UART    // Requires library from https://github.com/TMRh20/Sketches/tree/master/SPI_UART
//#define SOFTSPI     // Requires library from https://github.com/greiman/DigitalIO

#include "utility/includes.h"
    
/**********************/
#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

#endif // __RF24_CONFIG_H__
