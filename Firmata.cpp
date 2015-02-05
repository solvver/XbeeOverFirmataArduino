/*
  Firmata.cpp - Firmata library v2.4.0 - 2014-12-21
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

//******************************************************************************
//* Includes
//******************************************************************************

#include "Firmata.h"
#include "HardwareSerial.h"
#include <XBee.h>

extern "C" {
#include <string.h>
#include <stdlib.h>
}

//******************************************************************************
//* Support Functions
//******************************************************************************

void FirmataClass::sendValueAsTwo7bitBytes(int value)
{
  FirmataStream->write(value & B01111111); // LSB
  FirmataStream->write(value >> 7 & B01111111); // MSB
}

void FirmataClass::sendValueAsTwo7bitBytesXbee(uint8_t* payload, int offset, int value) //*** OK
{
  payload[offset]=(value & B01111111); // LSB
  payload[offset+1]=(value >> 7 & B01111111); // MSB
}

void FirmataClass::startSysex(void)
{
  FirmataStream->write(START_SYSEX);
}

void FirmataClass::endSysex(void)
{
  FirmataStream->write(END_SYSEX);
}

//******************************************************************************
//* Constructors
//******************************************************************************

FirmataClass::FirmataClass()
{
  firmwareVersionCount = 0;
  firmwareVersionVector = 0;
  systemReset();
}

//******************************************************************************
//* Public Methods
//******************************************************************************

/* begin method with default serial bitrate */
/*void FirmataClass::begin(57600)
{
  begin(57600);
}*/

/* begin method for overriding default serial bitrate */
void FirmataClass::begin(void)  //***long speed
{
  //Serial.begin(speed);    //***
  //FirmataStream = &Serial; //***
  blinkVersion();
  flagStreaming=1;
  contPayloadSD=0;
  numPayloadSD=0;
  //printVersion();    //***
  //printFirmwareVersion();   //***
}

/* begin method for overriding default stream */
void FirmataClass::begin(Stream &s)
{
  //FirmataStream = &s;  //***
  // do not call blinkVersion() here because some hardware such as the
  // Ethernet shield use pin 13
  printVersion();
  printFirmwareVersion();
}

// output the protocol version message to the serial port
void FirmataClass::printVersion(void)
{
  addr64=XBeeAddress64(rx64Address.getMsb(), rx64Address.getLsb());
  //addr64= XBeeAddress64(0x0013A200, 0x406FB3AE);
  payload[0][0] = REPORT_VERSION;
  payload[0][1] = FIRMATA_MAJOR_VERSION;
  payload[0][2] = FIRMATA_MINOR_VERSION;
  tx64 = Tx64Request(addr64, payload[0], 3);
  xbee.send(tx64);

 // FirmataStream->write(REPORT_VERSION);
 // FirmataStream->write(FIRMATA_MAJOR_VERSION);
 // FirmataStream->write(FIRMATA_MINOR_VERSION);
}

void FirmataClass::blinkVersion(void)
{
  // flash the pin with the protocol version
  pinMode(VERSION_BLINK_PIN, OUTPUT);
  strobeBlinkPin(FIRMATA_MAJOR_VERSION, 40, 210);
  delay(250);
  strobeBlinkPin(FIRMATA_MINOR_VERSION, 40, 210);
  delay(125);
}

void FirmataClass::printFirmwareVersion(void)
{
  //Serial.println("*_*_*_printFirmwareVersion_*_*_*");
  payload[0][0] = START_SYSEX;
  payload[0][1] = REPORT_FIRMWARE;
  payload[0][2] = firmwareVersionVector[0];
  payload[0][3] = firmwareVersionVector[1];
  payload[0][4] = END_SYSEX;
  tx64 = Tx64Request(addr64, payload[0], 5);
  if (firmwareVersionCount)  // make sure that the name has been set before reporting
  {
   xbee.send(tx64);
  }
}
/*    startSysex();
  ^^    FirmataStream->write(REPORT_FIRMWARE);
  ||    FirmataStream->write(firmwareVersionVector[0]); // major version number
  ||    FirmataStream->write(firmwareVersionVector[1]); // minor version number
  ||    for (i = 2; i < firmwareVersionCount; ++i)
      {
        sendValueAsTwo7bitBytes(firmwareVersionVector[i]);
      }
      endSysex();*/

void FirmataClass::setFirmwareNameAndVersion(const char *name, byte major, byte minor)
{
  const char *firmwareName;
  const char *extension;

  // parse out ".cpp" and "applet/" that comes from using __FILE__
  extension = strstr(name, ".cpp");
  firmwareName = strrchr(name, '/');

  if (!firmwareName)
  {
    // windows
    firmwareName = strrchr(name, '\\');
  }
  if (!firmwareName)
  {
    // user passed firmware name
    firmwareName = name;
  }
  else
  {
    firmwareName ++;
  }

  if (!extension)
  {
    firmwareVersionCount = strlen(firmwareName) + 2;
  }
  else
  {
    firmwareVersionCount = extension - firmwareName + 2;
  }

  // in case anyone calls setFirmwareNameAndVersion more than once
  free(firmwareVersionVector);

  firmwareVersionVector = (byte *) malloc(firmwareVersionCount);
  firmwareVersionVector[firmwareVersionCount] = 0;
  firmwareVersionVector[0] = major;
  firmwareVersionVector[1] = minor;
  strncpy((char *)firmwareVersionVector + 2, firmwareName, firmwareVersionCount - 2);
}

//------------------------------------------------------------------------------
// Serial Receive Handling

/*int FirmataClass::available(void)
{
  return FirmataStream->available();
}*/


void FirmataClass::processSysexMessage(void)
{
  //Serial.println("==>ProcessSysexMessage<==");
  switch (storedInputData[0])  //first byte in buffer is command
  {
  case REPORT_FIRMWARE:
    //Serial.println("ProcessSysexMessage==>REPORTfIRMWARE");
    printFirmwareVersion();
    break;
  case STRING_DATA:
    if (currentStringCallback)
    {
      byte bufferLength = (sysexBytesRead - 1) / 2;
      byte i = 1;
      byte j = 0;
      while (j < bufferLength)
      {
        // The string length will only be at most half the size of the
        // stored input buffer so we can decode the string within the buffer.
        storedInputData[j] = storedInputData[i];
        i++;
        storedInputData[j] += (storedInputData[i] << 7);
        i++;
        j++;
      }
      // Make sure string is null terminated. This may be the case for data
      // coming from client libraries in languages that don't null terminate
      // strings.
      if (storedInputData[j - 1] != '\0')
      {
        storedInputData[j] = '\0';
      }
      (*currentStringCallback)((char *)&storedInputData[0]);
    }
    break;
  default:
      //Serial.println("processSysexMessage==> CASE:default");
    if (currentSysexCallback){
      //Serial.println("processSysexMessage==> currentSysexCallback");
      (*currentSysexCallback)(storedInputData[0], sysexBytesRead - 1, storedInputData + 1);
      }
  }
}

void FirmataClass::processInput(uint8_t inputData)
{
 // int inputData = FirmataStream->read(); // this is 'int' to handle -1 when no data
  int command;

  // TODO make sure it handles -1 properly

  if (parsingSysex)
  {
    if (inputData == END_SYSEX)
    {
      //stop sysex byte
      parsingSysex = false;
      //fire off handler function
      processSysexMessage();
    }
    else
    {
      //normal data byte - add to buffer
      storedInputData[sysexBytesRead] = inputData;
      sysexBytesRead++;
    }
  }
  else if ( (waitForData > 0) && (inputData < 128) )
  {
    waitForData--;
    storedInputData[waitForData] = inputData;
    if ( (waitForData == 0) && executeMultiByteCommand ) // got the whole message
    {
      switch (executeMultiByteCommand)
      {
      case ANALOG_MESSAGE:
        if (currentAnalogCallback)
        {
          (*currentAnalogCallback)(multiByteChannel,
                                   (storedInputData[0] << 7)
                                   + storedInputData[1]);
        }
        break;
      case DIGITAL_MESSAGE:
        if (currentDigitalCallback)
        {
          (*currentDigitalCallback)(multiByteChannel,
                                    (storedInputData[0] << 7)
                                    + storedInputData[1]);
        }
        break;
      case SET_PIN_MODE:
        if (currentPinModeCallback)
          (*currentPinModeCallback)(storedInputData[1], storedInputData[0]);
        break;
      case REPORT_ANALOG:
      //Serial.println("--------->PROCESS INPUT::rEPORT_ANALOG");
        if (currentReportAnalogCallback)
          //(*currentReportAnalogCallback)(multiByteChannel, storedInputData[0]);
          (*currentReportAnalogCallback)(storedInputData[1], storedInputData[0]); //***
        break;
      case REPORT_DIGITAL:
      //Serial.println("--------->PROCESS INPUT::REPORT_DIGITAL");
        if (currentReportDigitalCallback)
          (*currentReportDigitalCallback)(multiByteChannel, storedInputData[0]);
        break;
      }
      executeMultiByteCommand = 0;
    }
  }
  else
  {
    // remove channel info from command byte if less than 0xF0
    if (inputData < 0xF0)
    {
      command = inputData & 0xF0;
      multiByteChannel = inputData & 0x0F;
    }
    else
    {
      command = inputData;
      // commands in the 0xF* range don't use channel data
    }
    switch (command)
    {
    case ANALOG_MESSAGE:
    case DIGITAL_MESSAGE:
    case SET_PIN_MODE:
      waitForData = 2; // two data bytes needed
      executeMultiByteCommand = command;
      break;
    case REPORT_ANALOG: //***
      waitForData = 2; // one data byte needed
      executeMultiByteCommand = command;
      break;
    case REPORT_DIGITAL:
      waitForData = 1; // one data byte needed
      executeMultiByteCommand = command;
      break;
    case START_SYSEX:
      //Serial.println("We have a sysex message");
      parsingSysex = true;
      sysexBytesRead = 0;
      break;
    case SYSTEM_RESET:
         Serial.println("SYSTEM RESET");
      systemReset();
      break;
    case REPORT_VERSION:
      //Serial.println("REPORTVersion");
      Firmata.printVersion(); //***
      break;
    }
  }
}

//------------------------------------------------------------------------------
// Serial Send Handling

// send an analog message
void FirmataClass::sendAnalog(byte pin, int value)
{
  // pin can only be 0-15, so chop higher bits
  payload[0][0] = (ANALOG_MESSAGE | (pin & 0xF));
  sendValueAsTwo7bitBytesXbee(payload[0], 1, value);
  tx64 = Tx64Request(addr64, payload[0], 4);
  xbee.send(tx64);
  //FirmataStream->write(ANALOG_MESSAGE | (pin & 0xF));
  //sendValueAsTwo7bitBytes(value);
}

void FirmataClass::sendPayloadSD(void){
    Serial.println("##sendPaylooaadSD##");
   /* int runner=1;
    payloadSD[0][0] = (START_SYSEX);
    payloadSD[0][1] = (SAMPLES_PACKET);
    while(runner<contPayloadSD){
       // if (payloadSD[0][runner]==0x01 && payloadSD[0][runner+1]!=0x01 ) // we have a digital vaue
       if (payloadSD[0][runner++]==0x02){   // we have an analog value
           runner++;
           sendValueAsTwo7bitBytesXbee(payloadSD[0], runner, payloadSD[0][runner]);
       }
    }
    payloadSD[0][runner] = (END_SYSEX);
    //sendValueAsTwo7bitBytesXbee(payload[0], 1, value);
    /*Serial.println("##runner##          :");
    Serial.println(runner);
    for(int k=0;k<=runner;k++){
    Serial.print(k);
    Serial.print("  :");
    Serial.println(payloadSD[0][k]);
    }
    tx64 = Tx64Request(addr64, payloadSD[0], ++runner);
    xbee.send(tx64);
    contPayloadSD=2;*/
     for (byte k=0;k<=numPayloadSD;k++){
                    if (k==numPayloadSD) lengthPayload=contPayloadSD;
                    for(byte i=0;i<lengthPayload;i++){
                        payload[k][i]=payloadSD[0][i];
                    }
                }
 }

int FirmataClass::storeAnalog(byte pin, int value)
{
    Serial.println("storeAnalog---><++");
    Serial.println(hour());
    firmataFile = FirmataSD.open("firmata.txt", FILE_WRITE);
      if (firmataFile) {
             firmataFile.print("analogChanel  ");
             firmataFile.print(pin);
             firmataFile.print(" , ");
             firmataFile.println(value);
             firmataFile.close();         // close the file:
             return (true);
           } else {
            // if the file didn't open, print an error:
            Serial.println("error opening test analog.txt");
            return (false);
          }
}

int FirmataClass::sendFile(void){
    firmataFile = FirmataSD.open("firmata.txt", FILE_READ);
         if (firmataFile) {
           while (firmataFile.available()) {
          	Serial.write(firmataFile.read());
          }  firmataFile.close();
          return(1);
          }
            else {
        	// if the file didn't open, print an error:
          Serial.println("error opening test send.txt");
          return (0);
        }
}

// send a single digital pin in a digital message
void FirmataClass::sendDigital(byte pin, int value)
{
  /* TODO add single pin digital messages to the protocol, this needs to
   * track the last digital data sent so that it can be sure to change just
   * one bit in the packet.  This is complicated by the fact that the
   * numbering of the pins will probably differ on Arduino, Wiring, and
   * other boards.  The DIGITAL_MESSAGE sends 14 bits at a time, but it is
   * probably easier to send 8 bit ports for any board with more than 14
   * digital pins.
   */

  // TODO: the digital message should not be sent on the serial port every
  // time sendDigital() is called.  Instead, it should add it to an int
  // which will be sent on a schedule.  If a pin changes more than once
  // before the digital message is sent on the serial port, it should send a
  // digital message for each change.

  //    if(value == 0)
  //        sendDigitalPortPair();
}


// send 14-bits in a single digital message (protocol v1)
// send an 8-bit port in a single digital message (protocol v2)
void FirmataClass::sendDigitalPort(byte portNumber, int portData)
{
 payload[0][0] = DIGITAL_MESSAGE | (portNumber & 0xF);
 payload[0][1] = ((byte)portData % 128);
 payload[0][2] = (portData >> 7);
 tx64 = Tx64Request(addr64, payload[0], 3);
 xbee.send(tx64);
}

int FirmataClass::storeDigitalPort(byte portNumber, int portData){
    Serial.println("storeAnalog---><++");
    firmataFile = FirmataSD.open("firmata.txt", FILE_WRITE);
    if (firmataFile) {
           firmataFile.print("digitalChanel  ");
           firmataFile.print(portNumber);
           firmataFile.print(" , ");
           firmataFile.println(portData);
           firmataFile.close();         // close the file:
           return (true);
         } else {
          // if the file didn't open, print an error:
          Serial.println("error opening test digital.txt");
          return (false);
        }
}

int FirmataClass::storeSamplingPacket(){
    Serial.println("StoreSamplingPacket");
    byte lengthPayload=95;
    firmataFile = FirmataSD.open("firmata.txt", FILE_WRITE);
    if (firmataFile) {
        //SD storage
    } else {                    //split payloadSD in packet
        if (contPayloadSD>lengthPayload){
            numPayloadSD=(contPayloadSD/lengthPayload);
            contPayloadSD=(contPayloadSD%lengthPayload);
            for (byte k=0;k<=numPayloadSD;k++){
                if (k==numPayloadSD) lengthPayload=contPayloadSD;
                for(byte i=0;i<lengthPayload;i++){
                    payload[k][i]=payloadSD[0][i];
                }
            }
        }  //else do nothing and send PayloadSD
    }
    //contPayloadSD;  //se inicializa a 0
    //if(type==0x02){ //ANALOG
    //    payloadSD[numPayloadSD][1]=0;
    //}
}

 /*/*payload[0][0] = (ANALOG_MESSAGE | (pin & 0xF));
  sendValueAsTwo7bitBytesXbee(payload[0], 1, value);
  tx64 = Tx64Request(addr64, payload[0], 4);
  xbee.send(tx64);
 FirmataStream->write(DIGITAL_MESSAGE | (portNumber & 0xF));
  FirmataStream->write((byte)portData % 128); // Tx bits 0-6
  FirmataStream->write(portData >> 7);  // Tx bits 7-13
*/

void FirmataClass::sendSysex(byte command, byte bytec, byte *bytev)
{
  byte i;
  startSysex();
  FirmataStream->write(command);
  for (i = 0; i < bytec; i++)
  {
    sendValueAsTwo7bitBytes(bytev[i]);
  }
  endSysex();
}

void FirmataClass::sendString(byte command, const char *string)
{
  sendSysex(command, strlen(string), (byte *)string);
}


// send a string as the protocol string type
void FirmataClass::sendString(const char *string)
{
  sendString(STRING_DATA, string);
}

// expose the write method
void FirmataClass::write(byte c)
{
  FirmataStream->write(c);
}


// Internal Actions/////////////////////////////////////////////////////////////

// generic callbacks
void FirmataClass::attach(byte command, callbackFunction newFunction)
{
  switch (command)
  {
  case ANALOG_MESSAGE: currentAnalogCallback = newFunction; break;
  case DIGITAL_MESSAGE: currentDigitalCallback = newFunction; break;
  case REPORT_ANALOG: currentReportAnalogCallback = newFunction; break;
  case REPORT_DIGITAL: currentReportDigitalCallback = newFunction; break;
  case SET_PIN_MODE: currentPinModeCallback = newFunction; break;
  }
}

void FirmataClass::attach(byte command, systemResetCallbackFunction newFunction)
{
    Serial.print("Attach system reset");
  switch (command)
  {
  case SYSTEM_RESET: currentSystemResetCallback = newFunction; break;
  }
}

void FirmataClass::attach(byte command, stringCallbackFunction newFunction)
{
  switch (command)
  {
  case STRING_DATA: currentStringCallback = newFunction; break;
  }
}

void FirmataClass::attach(byte command, sysexCallbackFunction newFunction)
{
  currentSysexCallback = newFunction;
}

void FirmataClass::detach(byte command)
{
  switch (command)
  {
  case SYSTEM_RESET: currentSystemResetCallback = NULL; break;
  case STRING_DATA: currentStringCallback = NULL; break;
  case START_SYSEX: currentSysexCallback = NULL; break;
  default:
    attach(command, (callbackFunction)NULL);
  }
}

// sysex callbacks
/*
 * this is too complicated for analogReceive, but maybe for Sysex?
 void FirmataClass::attachSysex(sysexFunction newFunction)
 {
 byte i;
 byte tmpCount = analogReceiveFunctionCount;
 analogReceiveFunction* tmpArray = analogReceiveFunctionArray;
 analogReceiveFunctionCount++;
 analogReceiveFunctionArray = (analogReceiveFunction*) calloc(analogReceiveFunctionCount, sizeof(analogReceiveFunction));
 for(i = 0; i < tmpCount; i++) {
 analogReceiveFunctionArray[i] = tmpArray[i];
 }
 analogReceiveFunctionArray[tmpCount] = newFunction;
 free(tmpArray);
 }
*/

//******************************************************************************
//* Private Methods
//******************************************************************************



// resets the system state upon a SYSTEM_RESET message from the host software
void FirmataClass::systemReset(void)
{
  Serial.println("systemReset");
  byte i;

  waitForData = 0; // this flag says the next serial input will be data
  executeMultiByteCommand = 0; // execute this after getting multi-byte data
  multiByteChannel = 0; // channel data for multiByteCommands

  for (i = 0; i < MAX_DATA_BYTES; i++)
  {
    storedInputData[i] = 0;
  }

  parsingSysex = false;
  sysexBytesRead = 0;

  if (currentSystemResetCallback)
    (*currentSystemResetCallback)();
}



// =============================================================================
// used for flashing the pin for the version number
void FirmataClass::strobeBlinkPin(int count, int onInterval, int offInterval)
{
  byte i;
  pinMode(VERSION_BLINK_PIN, OUTPUT);
  for (i = 0; i < count; i++)
  {
    delay(offInterval);
    digitalWrite(VERSION_BLINK_PIN, HIGH);
    delay(onInterval);
    digitalWrite(VERSION_BLINK_PIN, LOW);
  }
}


// make one instance for the user to use
FirmataClass Firmata;
