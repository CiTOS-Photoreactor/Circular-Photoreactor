// More info in the EEPROM library can be found at https://www.arduino.cc/en/Reference/EEPROM
#include "EEPROM_ID.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <Vrekrer_scpi_parser.h>

#define EEPROM_OFFSET 0

// SCPI functions

void readID(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  byte len = EEPROM.read(EEPROM_OFFSET);
  char ID[len+1];
  for (int i = 0; i < len; i++)
  {
    ID[i] = EEPROM.read(EEPROM_OFFSET + 1 + i);
  }
  ID[len] = '\0';
  interface.print(ID);
  interface.print("\n");
}

void writeID(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  if (parameters.Size() != 1)
  {
    interface.print("Command accepts exactly one parameter.");
    interface.print('\n');
  }
  else if (String(parameters[0]).length() > 15)
  {
    interface.print("IDN string too long.");
    interface.print('\n');
  }
  else
  {
    byte len = String(parameters[0]).length();
    const String &New_ID = String(parameters[0]);
    EEPROM.update(EEPROM_OFFSET, len);
    for (int i = 0; i < len; i++)
    {
      EEPROM.update(EEPROM_OFFSET + 1 + i, New_ID[i]);
    }
    interface.print("IDN updated to: ");
    interface.print(New_ID);
    interface.print(".\n");
  }
}
