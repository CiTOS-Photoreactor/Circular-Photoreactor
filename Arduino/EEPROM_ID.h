// Library to define the read and write ID functions for various Arduino modules.
// The functions are to be used with a SCPI interface.
// The identity is stored in the EEPROM starting at EEPROM_OFFSET (see main file, default is 0)
// Identity strings won't be saved if longer than 15 characters

#ifndef EEPROM_ID_h
#define EEPROM_ID_h

#include <Arduino.h>
#include <EEPROM.h>
#include <Vrekrer_scpi_parser.h>

void readID(SCPI_C commands, SCPI_P parameters, Stream &interface);
void writeID(SCPI_C commands, SCPI_P parameters, Stream &interface);

#endif
