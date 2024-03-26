/* Sketch to control the Peltier photoreactor
   Designed to work on an Arduino Nano Every
*/

// Arduino constants, for proper vscode setup
#ifndef ARDUINO
#define ARDUINO 181
#include <Arduino.h>
#include <HardwareSerial.h>
#endif
// End of vscode-required constants

// Define version, for easier tracking and changing of pin assignments
#define PPR_VERSION 3

// Define pins
#if (PPR_VERSION == 3)
// Rev 3 for PCB version
#define ONE_WIRE_BUS 14 // Define pin for temperature sensor
#define PELT_COOLING 10 // to R_PWM
#define PELT_HEATING 9 // to L_PWM - needs to be a PWM-Pin
#define PELT_ENABLE  6
#define Button_UP 15
#define Button_DOWN 16
#elif (PPR_VERSION == 2)
// revised version to have all Peltier controls to PWM pins
#define ONE_WIRE_BUS 2 // Define pin for temperature sensor
#define PELT_COOLING 6 // to R_PWM
#define PELT_HEATING 9 // to L_PWM - needs to be a PWM-Pin
#define PELT_ENABLE  10 // to R_EN and L_EN
#define Button_UP 5
#define Button_DOWN 4
#else
// original assignment
// /!\ pin 8 is NOT a PWM pin, see Nano specs sheet (= pin D8)
#define ONE_WIRE_BUS 2 // Define pin for temperature sensor
#define PELT_COOLING 6
#define PELT_HEATING 8 //needs to be a PWM-Pin
#define PELT_ENABLE  10
#define Button_UP 5
#define Button_DOWN 4
#endif

// Libraries and configuration for SSD1306 display connected to I2C (SDA, SCL pins)
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

// Libraries and configuration for DS18B20 temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Temperature configuration and limits
#define MAX_TEMP 60
#define MIN_TEMP -10
#define START_HEATING_TEMP 0

#define MAX_HEAT_TIME 60*10      //in seconds
#define MAX_COOL_TIME 60*60     //in seconds

// Libraries for SCPI communication and device identification
#include "EEPROM_ID.h"
#include <Vrekrer_scpi_parser.h>
SCPI_Parser SCPI;

// Define variables
float tempAct = 0; // actual temperature
int tempSet = 20; // set temperature
float tempRipple = 0.5; // allowed temperature difference before system heats or cools
float TimeInState = 0; // used to check for errors if state lasts too long
int cycleDelay = 250; // in us
unsigned long prevTime = 0;
unsigned long currTime = 0;
enum e_State {
  working = 0, // System has reached target temp, Peltier are idle
  heating = 1, // System is heating
  cooling = 2, // System is cooling
  error = 3, // Error detected, no current to Peltier until cleared
  ready = 4 // Starting/waiting state until an instruction is received
} currentState;

void setup()
{
  // Define SCPI commands
  SCPI.RegisterCommand(F("HELP?"), &help);
  SCPI.RegisterCommand(F("*IDN?"), &readID);
  SCPI.RegisterCommand(F("*IDN"), &writeID);
  SCPI.RegisterCommand(F("DELAY"), &updateDelay);
  SCPI.RegisterCommand(F("CLEAR"), &clearError);
  SCPI.RegisterCommand(F("SUSPEND"), &suspend);
  SCPI.RegisterCommand(F("STAT?"), &sendStatus);
  SCPI.SetCommandTreeBase(F("TEMPerature"));
    SCPI.RegisterCommand(F(":ACT?"), &sendTempAct);
    SCPI.RegisterCommand(F(":SET?"), &sendTempSet);
    SCPI.RegisterCommand(F(":SET"), &updateTempSet);

  // Configure ports, sensor and pins
  Serial.begin(9600);
  sensors.begin();
  setupDisplay(); //Init OLED

  pinMode(Button_UP, INPUT_PULLUP);
  pinMode(Button_DOWN, INPUT_PULLUP);

  pinMode(PELT_ENABLE, OUTPUT);
  digitalWrite(PELT_ENABLE, LOW);

  pinMode(PELT_COOLING, OUTPUT);
  pinMode(PELT_HEATING, OUTPUT);
  digitalWrite(PELT_COOLING, LOW);
  digitalWrite(PELT_HEATING, LOW);

  currentState = ready;
  prevTime = millis();
}

void loop()
{
  // Check serial for instructions and process with SCPI parser
  SCPI.ProcessInput(Serial, "\n");

  // Check time so the main loop is only run every 250 us
  currTime = millis();
  if(currTime - prevTime > cycleDelay)
  {
    prevTime = currTime;
    
    //Increase Time in State, if not in error State, TimeInState is in seconds
    if (currentState != error)
    {
      TimeInState = TimeInState + cycleDelay/1000.0;
    }

    // Get temperature from sensor
    tempAct = getTemp();

    // Check if everthing is alright (Max heating time, Temperature within limits, ...)
    checkForError();

    // Check if a button is pressed
    getButtonInput();

    // Update between heating, cooling or working states
    // if not in error state or waiting for instruction ("ready" state)
    // Active heating only if tempSet is above START_HEATING_TEMP
    if (currentState != error && currentState != ready)
    {
      if (tempAct > tempSet + tempRipple)
      {
        currentState = cooling;
      }
      else if (tempAct <= tempSet - tempRipple && tempSet >= START_HEATING_TEMP)
      {
        currentState = heating;
      }
      else
      {
        currentState = working;
        TimeInState = 0;
      }
    }

    // Write heating/cooling state to driver
    switch (currentState)
    {
      case cooling:
        digitalWrite(PELT_HEATING, LOW);
        digitalWrite(PELT_COOLING, HIGH);
        digitalWrite(PELT_ENABLE, HIGH);
        break;
      case heating:
        digitalWrite(PELT_COOLING, LOW);
        analogWrite(PELT_HEATING, 255 / 100 * 70);
        digitalWrite(PELT_ENABLE, HIGH);
        break;
      default:
        digitalWrite(PELT_ENABLE, LOW);
        digitalWrite(PELT_COOLING, LOW);
        digitalWrite(PELT_HEATING, LOW);
        break;
    }

    // Write OLED
    updateDisplay();
  }
}

void setupDisplay()
{
  // Initialise OLED

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(500); // Pause for 0,5 seconds

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
}

void updateDisplay()
{
  //Clean up OLED display
  display.clearDisplay();
  display.setCursor(0, 0);

  //Write actual Temperature in the first row
  display.print(F("Tact: "));
  if (tempAct <= -10)
  {
    display.println(tempAct, 0);
  }
  else
  {
    display.println(tempAct, 1);
  }
  //Write Set Temperature in the second row
  display.print(F("Tset: "));
  display.println(tempSet);

  //Third row
  display.println(F(""));

  //Write Peltier state in last row
  if (currentState == heating)
  {
    display.println(F("heating"));
  }
  else if (currentState == cooling)
  {
    display.println(F("cooling"));
  }
  else if (currentState == working)
  {
    display.println(F("working"));
  }
  else if (currentState == ready)
  {
    display.println(F("ready"));
  }
  else
  {
    display.println(F("error"));
  }

  display.display();
}

float getTemp()
{
  //returns the temperature from one DS18B20 in DEG Celsius
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

//Check heating and cooling time
//also checks if the Temperature is within the limits
//turn off heating, if heating time is longer than MAX_HEAT_TIME
//turn off cooling, if cooling time is longer than MAX_COOL_TIME
//turn off peltier elements, if Temperature is not within the limits
void checkForError()
{
  if (currentState == heating || currentState == cooling)
  {
    if (currentState == heating && TimeInState >= MAX_HEAT_TIME)
    {
      currentState = error;
      TimeInState = 0;
    }
    else if (currentState == cooling && TimeInState >= MAX_COOL_TIME)
    {
      currentState = error;
      TimeInState = 0;
    }
  }

  // if TA is too high or too low, shutdown peltier elements
  if (tempAct > MAX_TEMP + 5 || tempAct < MIN_TEMP - 10)
  {
    if (currentState != ready)
    {
      currentState = error;
    }
  }
}

//Check if a Button is pressed
void getButtonInput()
{
  if (digitalRead(Button_UP) == LOW && tempSet < MAX_TEMP)
  {
    delay(100);
    if (digitalRead(Button_DOWN) == LOW)
    {
      TimeInState = 0;
      currentState = ready;
    }
    else
    {
      tempSet++;
      if (currentState == error || currentState == ready)
      {
        TimeInState = 0;
        currentState = working;
      }
    }
  }
  else if (digitalRead(Button_DOWN) == LOW && tempSet > MIN_TEMP)
  {
    delay(100);
    if (digitalRead(Button_UP) == LOW)
    {
      TimeInState = 0;
      currentState = ready;
    }
    else
    {
      tempSet--;
      if (currentState == error || currentState == ready)
      {
        TimeInState = 0;
        currentState = working;
      }
    }
  }
}

// SCPI functions
void help(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  interface.println(F("*IDN: read or update the device's identification string."));
  interface.println(F("TEMPerature: tree base."));
  interface.println(F(" TEMP:ACT: read the actual temperature as ACT:value in degree Celsius."));
  interface.println(F(" TEMP:SET: read or update the setting temperature as SET:value."));
  interface.println(F("CLEAR: clear error."));
  interface.println(F("SUSPEND: stop heating/cooling, put device in ready state."));
  interface.println(F("STAT?: returns current status as STAT:value."));
  interface.println(F("HELP?: return list of commands."));
}

void sendTempAct(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  interface.print("ACT:");
  interface.print(tempAct,2);
  interface.print(".\n");
}

void sendTempSet(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  interface.print("SET:");
  interface.print(tempSet);
  interface.print(".\n");
}

void sendStatus(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  interface.print("STAT:");
  if (currentState == heating)
  {
    interface.println(F("heating"));
  }
  else if (currentState == cooling)
  {
    interface.println(F("cooling"));
  }
  else if (currentState == working)
  {
    interface.println(F("working"));
  }
  else if (currentState == ready)
  {
    interface.println(F("ready"));
  }
  else
  {
    interface.println(F("error"));
  }
}

void updateTempSet(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  if (parameters.Size() != 1)
  {
    interface.print(F("Command accepts exactly one parameter."));
    interface.print('\n');
  }
  else
  {
    tempSet = String(parameters[0]).toInt();
    if (currentState == ready)
    {
      currentState = working;
    }
  }
}

void updateDelay(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  if (parameters.Size() != 1)
  {
    interface.print(F("Command accepts exactly one parameter."));
    interface.print('\n');
  }
  else
  {
    cycleDelay = String(parameters[0]).toInt();
  }
}

void clearError(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  currentState = working;
}

void suspend(SCPI_C commands, SCPI_P parameters, Stream &interface)
{
  currentState = ready;
}
