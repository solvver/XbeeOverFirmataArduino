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
  numPayloads=0;
  samplesCount=0;
  numberChannels=0;
  lengthPayload=90;
  numPayloadsCounter=0;
  samplePacketInitialiced=0;
  for (byte k=3;k>0;k--){
    firstSample[k]=true;
    for (byte i=0;i<=TOTAL_PINS;i++){
          contSamplesStored[k][i]=1;
        }
  }
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
    Serial.println("ProcessSysexMessage==>REPORTfIRMWARE");
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
      Serial.println("REPORTVersion");
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
  tx64 = Tx64Request(addr64, payload[0], 3);
  xbee.send(tx64);
  //FirmataStream->write(ANALOG_MESSAGE | (pin & 0xF));
  //sendValueAsTwo7bitBytes(value);
}

void FirmataClass::sendSamplingPacket(void){
    //numPayloads=((numberChannels*samplesCount*2+8)%lengthPayload);
    /*byte contPayload=0;
    byte contSamplesPayload=0;
    uint16_t totalSamplesStored=0;

    payload[0][0]=START_SYSEX;
    payload[0][1]=SAMPLES_PACKET;
    payload[0][2]=samplesCount;         //!!!!!!!!!!!!hay que enviar el número de paquetes para parsear en Node.js, hazlo aquí
    payload[0][3]=(year()&0x0F);
    payload[0][4]=month();
    payload[0][5]=day();
    payload[0][6]=hour();
    payload[0][7]=minute();
    payload[0][8]=second();
    totalSamplesStored=9;

    for (byte typesCounter=3;typesCounter>0;typesCounter--){
        for (byte channelsCounter=0;channelsCounter<contChannels[typesCounter];channelsCounter++){
            for (byte samplesCounter=0;samplesCounter<contSamplesStored[typesCounter][channelsCounter];samplesCounter++){
                    if (samplesCounter==0) {                                        //first byte contains pin info
                          Serial.println("first sample");
                          totalSamplesStored+=contSamplesStored[typesCounter][channelsCounter];
                          switch (typesCounter){
                              case 1:                                                     //digital channels
                              payload[contPayload][samplesCount++]=DIGITAL_MESSAGE | (samplesPacket[typesCounter][channelsCounter][samplesCounter] & 0x0F);  //DIGITAL_MESSAGE | (portNumber & 0xF)
                              break;
                              case 2:                                                     //analog channel
                              payload[contPayload][samplesCount++]=(ANALOG_MESSAGE | (samplesPacket[typesCounter][channelsCounter][samplesCounter] & 0xF));
                              break;
                          }
                      }
                      payload[contPayload][samplesCount++]=samplesPacket[typesCounter][channelsCounter][samplesCounter];
                      if (samplesCount==100 || samplesCount==totalSamplesStored){
                        Serial.println("last sample for one packet");
                        payload[contPayload][samplesCount]=END_SYSEX;
                        contPayload++;
                      }
            }
        }
    }
     for (byte typesCounter=3;typesCounter>0;typesCounter--){
            for (byte channelsCounter=0;channelsCounter<contChannels[typesCounter];channelsCounter++){
                free (samplesPacket[typesCounter][channelsCounter]);
            }
        }
    for(byte contPayloadToSend=0;contPayloadToSend<=contPayload;contPayloadToSend++){
        if (contPayloadToSend==contPayload) lengthPayload=totalSamplesStored-100*contPayload;
        else lengthPayload=100;
            tx64 = Tx64Request(addr64, payload[contPayloadToSend], lengthPayload);
            xbee.send(tx64);
    }
*/
}
     //Serial.println("##sPRINTING--GayLorzas##");
     /*for (byte k=0;k<=numPayloads;k++){
             // Serial.print("Numero");
             // Serial.println(k);
          if(k==numPayloads) {
              //lengthPayload=(((samplesCount*numberChannels)%(30*numPayloads))*3)+9;
              lengthPayload=(((totalSamples)%(30*numPayloads))*3)+9;
          } else {
              lengthPayload=99;
          }
         // Serial.print("lengthPayload  :");
         // Serial.println(lengthPayload);
          tx64 = Tx64Request(addr64, payload[k], lengthPayload);
          xbee.send(tx64);
         // Serial.println("");
      }
    numPayloadsCounter=0;
    contPayload=0;*/

/*in FirmataClass::storeAnalog(byte pin, int value)
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
}*/

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
/*
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
}*/

/*int FirmataClass::storeSamplingPacket(byte pin, int value, byte type){
    // Serial.println("storeSamplingPacket  :");
    //Firmata.sendString("arduino storing");
    if(numPayloadsCounter==numPayloads){
        lengthPayload=(((totalSamples)%(30*numPayloads))*3)+9;
    } else {
        lengthPayload=99;
    }
    if (contPayload<8){
        payload[numPayloadsCounter][contPayload++]=START_SYSEX;
        payload[numPayloadsCounter][contPayload++]=SAMPLES_PACKET;
        payload[numPayloadsCounter][contPayload++]=(year()&0x0F);
        payload[numPayloadsCounter][contPayload++]=month();
        payload[numPayloadsCounter][contPayload++]=day();
        payload[numPayloadsCounter][contPayload++]=hour();
        payload[numPayloadsCounter][contPayload++]=minute();
        payload[numPayloadsCounter][contPayload++]=second();
    }
    if(type==0x01){
        payload[numPayloadsCounter][contPayload++]=(DIGITAL_MESSAGE | (pin & 0xF));
        payload[numPayloadsCounter][contPayload++]=((byte)value % 128);
        payload[numPayloadsCounter][contPayload++]=(value >> 7);
    }
    if(type==0x02){
        payload[numPayloadsCounter][contPayload++]=(ANALOG_MESSAGE | (pin & 0xF));
        sendValueAsTwo7bitBytesXbee(payload[numPayloadsCounter], contPayload, value);
        contPayload+=2;
    }
    if(contPayload==(lengthPayload-1)){
       payload[numPayloadsCounter][contPayload]=END_SYSEX;
       contPayload=0;
       numPayloadsCounter++;
    }
}*/

int FirmataClass::storeSamplingPacket(uint8_t pin, int value, byte type){
    /*bool channelStoredBefore=false;
    if (firstSample[type]==true){
        firstSample[type]=false;
        contChannels[type]=1;
        if (!samplePacketInitialiced){
                   // samplesPacket=(uint8_t***)calloc(((numberChannels*samplesCount*2)+(numberChannels)), sizeof(uint8_t)); //reservar memoria para 2 bytes por muestra y una de número canal
                    Serial.print("sizeof(samplesPacket)  :");
                    Serial.println(sizeof(samplesPacket));
                    samplePacketInitialiced=true;
                    }
        samplesPacket[type][0][0]=pin;
    } else {
        for (byte k=contChannels[type];k>0;k--){
          if (pin==samplesPacket[type][k-1][0]) channelStoredBefore=true;
        }
        if (!channelStoredBefore){
          samplesPacket[type][contChannels[type]][0]=pin;
          contChannels[type]++;
        }
    }
    if (type==1){
        for (byte channelNumber=0;channelNumber<contChannels[type];channelNumber++){
            if (pin==samplesPacket[1][channelNumber][0]){
                samplesPacket[1][channelNumber][contSamplesStored[1][channelNumber]++]=((uint8_t)value % 128);
                samplesPacket[1][channelNumber][contSamplesStored[1][channelNumber]++]=(value >> 7);
            }
        }
    } else if (type==2) {
          for (byte channelNumber=0;channelNumber<contChannels[type];channelNumber++){
                    if (pin==samplesPacket[2][channelNumber][0]){
                       // samplesPacket[2][channelNumber][contSamplesStored[2][channelNumber]++]=((uint8_t)value % 128);
                       // samplesPacket[2][channelNumber][contSamplesStored[2][channelNumber]++]=(value >> 7);
                       sendValueAsTwo7bitBytesXbee(samplesPacket[2][channelNumber], contSamplesStored[2][channelNumber], value);
                       contSamplesStored[2][channelNumber]+=2;
                    }
                }
    }*/
}


void FirmataClass::sendSysex(byte command, byte bytec, byte *bytev)
{
  byte i;
  byte k=0;
  payload[9][0] =START_SYSEX;
  payload[9][1] = command;
  for (i = 0; i < bytec; i++)
  {
   payload[9][i+2] = bytev[i];
   k=i;
  }
  payload[9][k+3]=END_SYSEX;
  tx64 = Tx64Request(addr64, payload[9], (bytec+3));
  xbee.send(tx64);
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

void FirmataClass::sendInt(uint8_t uint8_t)
{
  sendSysex(INT_DATA, 1, (byte *)uint8_t);
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
