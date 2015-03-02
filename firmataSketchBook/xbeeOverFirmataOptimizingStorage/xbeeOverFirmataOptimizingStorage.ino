/*
 * Firmata is a generic protocol for communicating with microcontrollers
 * from software on a host computer. It is intended to work with
 * any host computer software package.
 *
 * To download a host software package, please clink on the following link
 * to open the download page in your default browser.
 *
 * http://firmata.org/wiki/Download
 */

/*
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2014 Jeff Hoefs.  All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
  formatted using the GNU C formatting and indenting
*/

/*
 * TODO: use Program Control to load stored profiles from EEPROM
 */
#include <SD.h>
#include <Servo.h>
#include <Wire.h>
#include <Firmata.h>
#include <XBee.h>
#include <Time.h>
//initialize class xbee and variables ***
/*XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();*/

uint8_t option = 0;
uint8_t data[100];

// move the following defines to Firmata.h?
#define I2C_WRITE B00000000
#define I2C_READ B00001000
#define I2C_READ_CONTINUOUSLY B00010000
#define I2C_STOP_READING B00011000
#define I2C_READ_WRITE_MODE_MASK B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000

#define MAX_QUERIES 8
#define MINIMUM_SAMPLING_INTERVAL 10
#define MINIMUM_DELIVERY_INTERVAL 10

#define REGISTER_NOT_SPECIFIED -1

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned long previousMillis2;      // same as currentMillis. Used with deliveryInterval
unsigned int samplingInterval = 19;          // how often to run the main loop (in ms)
unsigned long deliveryInterval = 0;           

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
};

/* for i2c read continuous more */
i2c_device_info query[MAX_QUERIES];

byte i2cRxData[32];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
unsigned int i2cReadDelayTime = 0;  // default delay time between i2c read request and Wire.requestFrom()

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void readAndReportData(byte address, int theRegister, byte numBytes) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
#if ARDUINO >= 100
    Wire.write((byte)theRegister);
#else
    Wire.send((byte)theRegister);
#endif
    Wire.endTransmission();
    // do not set a value of 0
    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (numBytes < Wire.available()) {
    Firmata.sendString("I2C Read Error: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    Firmata.sendString("I2C Read Error: Too few bytes received");
  }

  i2cRxData[0] = address;
  i2cRxData[1] = theRegister;

  for (int i = 0; i < numBytes && Wire.available(); i++) {
#if ARDUINO >= 100
    i2cRxData[2 + i] = Wire.read();
#else
    i2cRxData[2 + i] = Wire.receive();
#endif
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend) 
{  
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent 
    if (forceSend==true){
      if (previousPINs[portNumber] != portValue){
        Firmata.sendDigitalPort(portNumber, portValue);
        previousPINs[portNumber] = portValue;
      }
    } else {  
      Firmata.storeSamplingPacket(portNumber, portValue, 0x01);
    }    
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  int forceSend;
  if (Firmata.flagStreaming==1) forceSend=true;
  else forceSend=false;
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), forceSend);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), forceSend);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), forceSend);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), forceSend);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), forceSend);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), forceSend);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), forceSend);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), forceSend);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), forceSend);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), forceSend);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), forceSend);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), forceSend);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), forceSend);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), forceSend);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), forceSend);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), forceSend);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (pinConfig[pin] == I2C && isI2CEnabled && mode != I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_ANALOG(pin)) {
   // Serial.println("setPinModeCallback IS_PIN_ANALOG");
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting 0=>NO reporting
   // Serial.print("Reporting???");
    //Serial.println (mode);
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) {
      //Serial.println("Configuring input digital pin");
        if (!(portConfigInputs[pin / 8] & (1 << (pin & 7)))) Firmata.numberChannels++;
      portConfigInputs[pin / 8] |= (1 << (pin & 7));      
       // Firmata.totalSamples=(Firmata.numberChannels*Firmata.samplesCount);
        Serial.print("Firmata.number digital Channels  ");
        Serial.println(Firmata.numberChannels);
        Serial.print("Firmata.samplesCount  ");
        Serial.println(Firmata.samplesCount);
        Firmata.samplePacketInitialiced[1]=false;
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch (mode) {
    case ANALOG:
      // Serial.print("setPinModeCallback  CASE ANALOG");
       pin=CHANNEL_TO_PIN(pin);
      if (IS_PIN_ANALOG(pin)) {
        //Serial.print("setPinModeCallback IS_PIN_ANALOG CASE ANALOG");
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        }
        pinConfig[pin] = ANALOG;
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        //Serial.print("setPinModeCallback IS_PIN_DIGITAL CASE INPUT");
        pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        pinConfig[pin] = INPUT;
         //Serial.print("pinConfig???");
        //Serial.println (pinConfig[5]);
      }
      break;
    case OUTPUT:
      //Serial.println("Set digital output");
      //Serial.print(pin);
      if (IS_PIN_DIGITAL(pin)) {
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        pinConfig[pin] = OUTPUT;
      }
      break;
    case PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        pinConfig[pin] = PWM;
      }
      break;
    case I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        pinConfig[pin] = I2C;
      }
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (pinConfig[pin]) {
      case PWM:
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), value);
        pinState[pin] = value;
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport &~ (1 << analogPin);
    } else {
     // Serial.println("configuring pin to reportAnalog ");
      if (!(analogInputsToReport & (1 << analogPin))) Firmata.numberChannels++;
      //Firmata.totalSamples=(Firmata.numberChannels*Firmata.samplesCount);
     Serial.print("Firmata analog channels number");
      Serial.println(Firmata.numberChannels);
      Serial.print("Firmata.samplesCount  ");
      Serial.println(Firmata.samplesCount);
      //Firmata.sendString("reportAnalogCallback");
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // Send pin value immediately. This is helpful when connected via
      // ethernet, wi-fi or bluetooth so pin states can be known upon
      // reconnecting.
      Firmata.sendAnalog(analogPin, analogRead(analogPin));
      Firmata.samplePacketInitialiced[2]=false;
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  /*Serial.println("01010101-REPORTdigitalCALLBACK---");
  Serial.println(port);
  Serial.println(value);*/
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, uint32_t *argv)
{
  //Serial.print("sysexCallback==>");
  //Serial.print("command:");
  //Serial.print(command);
  //Serial.print("argc");
  //Serial.print(argc);
  //Serial.print("argv");
  //Serial.println(*argv);
 /* byte mode;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;*/
  int cont=0;
  int lengthPayload[10];
  int numPayload=0;

  switch (command) {
      case SET_TIME:
      setTime((argv[3]),(argv[4]),(argv[5]),(argv[2]),(argv[1]),(argv[0]));
      break;
    /*case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing not supported");
        return;
      }
      else {
        slaveAddress = argv[0];
      }
      switch (mode) {
        case I2C_WRITE:
          Wire.beginTransmission(slaveAddress);
          for (byte i = 2; i < argc; i += 2) {
            data = argv[i] + (argv[i + 1] << 7);
#if ARDUINO >= 100
            Wire.write(data);
#else
            Wire.send(data);
#endif
          }
          Wire.endTransmission();
          delayMicroseconds(70);
          break;
        case I2C_READ:
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          readAndReportData(slaveAddress, (int)slaveRegister, data);
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= MAX_QUERIES) {
            // too many queries, just ignore
            Firmata.sendString("too many queries");
            break;
          }
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = (int)REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = slaveRegister;
          query[queryIndex].bytes = data;
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          // if read continuous mode is enabled for only 1 i2c device, disable
          // read continuous reporting for that device
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            // if read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove it's data from
            // the array, shifiting other array data to fill the space
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr == slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }
            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].reg;
                query[i].bytes = query[i + 1].bytes;
              }
            }
            queryIndex--;
          }
          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));
      if (delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }
      if (!isI2CEnabled) {
        enableI2CPins();
      }
      break;*/
    case SAMPLING_INTERVAL:  //OK
      Serial.print("sampling interval  ");
      if (argc > 1) {
          samplingInterval = (argv[0] + (argv[1] << 8)); //***-7=>+8
          
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else if (argc==1){
        samplingInterval = (argv[0]);
      }
      Serial.println(samplingInterval);
     break;
    case DELIVERY_INTERVAL:  //***
    Serial.print("DELIVERY_INTERVAL");
      if (argc > 1) {
        if (argc==4){
          deliveryInterval = ((argv[0]<< 24) + (argv[1]<<16) + (argv[2]<<8) + (argv[3]));
        } else if (argc==3) {
          deliveryInterval = ((argv[0]<<16) + (argv[1]<<8) + (argv[2]));
        } else {
          deliveryInterval = ((argv[0]<< 8) + (argv[1]));
        }
        previousMillis2=deliveryInterval;
        Serial.print(deliveryInterval);
        if (deliveryInterval < MINIMUM_DELIVERY_INTERVAL) {
          deliveryInterval = MINIMUM_DELIVERY_INTERVAL;
        }
        /*if (deliveryInterval==65535) {
          Firmata.flagStreaming=1;
        }
        else {*/
          Firmata.flagStreaming=0;
          Firmata.samplesCount=(deliveryInterval/samplingInterval);
          Serial.print("   ");
          Serial.println(Firmata.samplesCount);
        //}
      }
      break;
    /*case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;*/
      case CAPABILITY_QUERY:  //OK
      Firmata.payload[0][cont++]=START_SYSEX;
      Firmata.payload[0][cont++]=CAPABILITY_RESPONSE;
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        //Serial.println(pin);
        if (cont>=90) {
          Firmata.payload[numPayload][cont++]=END_SYSEX;
          lengthPayload[numPayload]=cont;     //save length payload to make the request
          numPayload++;
          cont=0;
          Firmata.payload[numPayload][cont++]=START_SYSEX;
          Firmata.payload[numPayload][cont++]=CAPABILITY_RESPONSE;  
        }
        if (IS_PIN_DIGITAL(pin)) {
          //Serial.println("pin___Digital");
          Firmata.payload[numPayload][cont++]=((byte)INPUT);
          Firmata.payload[numPayload][cont++]=(1);
          Firmata.payload[numPayload][cont++]=((byte)OUTPUT);
          Firmata.payload[numPayload][cont++]=(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          //Serial.println("pin___Analog");
          Firmata.payload[numPayload][cont++]=(ANALOG);
          Firmata.payload[numPayload][cont++]=(10);
        }
        if (IS_PIN_PWM(pin)) {
          //Serial.println("pin___PWM");
          Firmata.payload[numPayload][cont++]=(PWM);
          Firmata.payload[numPayload][cont++]=(8);
        }
        if (IS_PIN_DIGITAL(pin)) {
          //Serial.println("pin___servo");
          Firmata.payload[numPayload][cont++]=(SERVO);
          Firmata.payload[numPayload][cont++]=(14);
        }
        if (IS_PIN_I2C(pin)) {
          //Serial.println("pin___i2c");
          Firmata.payload[numPayload][cont++]=(I2C);
          Firmata.payload[numPayload][cont++]=(1);  // to do: determine appropriate value
        }
        Firmata.payload[numPayload][cont++]=(127);
      }
      Firmata.payload[numPayload][cont++]=END_REPORT;
      Firmata.payload[numPayload][cont]=END_SYSEX;
      Firmata.addr64= XBeeAddress64(0x0013A200, 0x406FB3AE);
      for (int contPayloadToSend=0; contPayloadToSend<(numPayload+1);contPayloadToSend++){
      //Serial.print("sending");  
      if(contPayloadToSend<numPayload) Firmata.tx64 = Tx64Request(Firmata.addr64, Firmata.payload[contPayloadToSend], lengthPayload[contPayloadToSend]);
      if(contPayloadToSend==numPayload) Firmata.tx64 = Tx64Request(Firmata.addr64, Firmata.payload[contPayloadToSend], (cont+1));
      Firmata.xbee.send(Firmata.tx64);
      }
      break;
    case PIN_STATE_QUERY:/*
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write((byte)pinConfig[pin]);
          Firmata.write((byte)pinState[pin] & 0x7F);
          if (pinState[pin] & 0xFF80) Firmata.write((byte)(pinState[pin] >> 7) & 0x7F);
          if (pinState[pin] & 0xC000) Firmata.write((byte)(pinState[pin] >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;*/
    case ANALOG_MAPPING_QUERY:  // OK
      Firmata.payload[0][cont++]=START_SYSEX;
      Firmata.payload[0][cont++]=ANALOG_MAPPING_RESPONSE;
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (cont>=90) {
          Firmata.payload[numPayload][cont++]=END_SYSEX;
          lengthPayload[numPayload]=cont;     //save length payload to make the request
          numPayload++;
          cont=0;
          Firmata.payload[numPayload][cont++]=START_SYSEX;
          Firmata.payload[numPayload][cont++]=ANALOG_MAPPING_RESPONSE;  
        }
       Firmata.payload[numPayload][cont++]=(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.payload[numPayload][cont]=END_SYSEX;
      Firmata.addr64= XBeeAddress64(0x0013A200, 0x406FB3AE);
      for (int contPayloadToSend=0; contPayloadToSend<(numPayload+1);contPayloadToSend++){
      if(contPayloadToSend<numPayload) Firmata.tx64 = Tx64Request(Firmata.addr64, Firmata.payload[contPayloadToSend], lengthPayload[contPayloadToSend]);
      if(contPayloadToSend==numPayload) Firmata.tx64 = Tx64Request(Firmata.addr64, Firmata.payload[contPayloadToSend], (cont+1));
      Firmata.xbee.send(Firmata.tx64);
      }
      break;
  }
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, I2C);
    }
  }

  isI2CEnabled = true;

  // is there enough time before the first I2C request to call this here?
  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
}

/*==============================================================================
 * SETUP()
 *============================================================================*/

void systemResetCallback()
{
  Serial.println("SystemResetCallback");
  // initialize a defalt state
  currentMillis=0;        // store the current value from millis()
  previousMillis=0;       // for comparison with currentMillis
  previousMillis2=0;      // same as currentMillis. Used with deliveryInterval
  samplingInterval = 19;          // how often to run the main loop (in ms)
  deliveryInterval = 0;

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;      // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }
  /*for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, ANALOG);
    } else {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }
  }*/
  
  // by default, do not report any analog inputs
  analogInputsToReport = 0;
  //reset Firmata's variables
  for (byte typesCounter=3;typesCounter>0;typesCounter--){
           for (byte channelsCounter=0;channelsCounter<Firmata.contChannels[typesCounter];channelsCounter++){
                free((uint8_t*)Firmata.samplesPacket[typesCounter][channelsCounter]);
            }
            free((uint8_t**)Firmata.samplesPacket[typesCounter]);
        }
     free((uint8_t***)Firmata.samplesPacket);
  Firmata.begin();
}

void setup()
{
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);
  
  Serial2.begin(57600); //***
  Firmata.xbee.begin(Serial2);  //***
  Serial.begin(57600);  //***
  Serial.println("Firmata is going to START");
  Firmata.begin();
 /* pinMode(22, OUTPUT);
  pinMode(53, OUTPUT);
  if (!Firmata.FirmataSD.begin(53)) {
    Serial.println("initialization failed!");
    return;
  } else Serial.println("initialization done.");*/
 // systemResetCallback();  // reset to default config
 Serial.println("SETUP is DONE");
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop()
{
  byte pin, analogPin, portValue;
  int contDataRx;
  
  currentMillis = millis();
  
  if ((currentMillis - previousMillis2) > deliveryInterval && Firmata.flagStreaming==0 && Firmata.readyToSend==true){ //enviar paquetes almacenados mientras tanto
    previousMillis2+=deliveryInterval;
    /*if(Firmata.sendFile());
    else Firmata.sendPayloadSD();*/
   // Firmata.sendString("arduino sending");
     Firmata.sendSamplingPacket();
  }
    
  Firmata.xbee.readPacket();
  if (Firmata.xbee.getResponse().isError()){
      Serial.print("response error code");
      Serial.println(Firmata.xbee.getResponse().getErrorCode());
      Firmata.sendErrorTx(Firmata.xbee.getResponse().getErrorCode());
  }


  /* DIGITALREAD - as fast as possible, check for changes and output them to the
   * FTDI buffer using Serial.print()  */
  if (Firmata.flagStreaming)  checkDigitalInputs();

  /* SERIALREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
    
   if (Firmata.xbee.getResponse().isAvailable()){
          //Serial.println("#######_AVAILABLE_########");
          if (Firmata.xbee.getResponse().getApiId() ==TX_STATUS_RESPONSE ) {
            Firmata.xbee.getResponse().getTxStatusResponse(Firmata.TxStatus);
            //if(Firmata.TxStatus.getStatus()==0) Serial.println("Tx OK");
        }
          if (Firmata.xbee.getResponse().getApiId() == RX_16_RESPONSE) {
                /*xbee.getResponse().getRx16Response(rx16);
        	option = rx16.getOption();*/
        	//data = rx16.getData(0);
        } 
        if (Firmata.xbee.getResponse().getApiId() == RX_64_RESPONSE) {
                Firmata.xbee.getResponse().getRx64Response(Firmata.rx64); //comprobar que se pasan bien las cosas
                Firmata.rx64Address=Firmata.rx64.getRemoteAddress64();
                option = Firmata.rx64.getOption();
                
                for(contDataRx=0;contDataRx<(((Firmata.xbee.getResponse().getMsbLength()<<8)+Firmata.xbee.getResponse().getLsbLength())-11);contDataRx++){
                //Serial.print("Data to process:  ");
                //Serial.println(Firmata.rx64.getData(contDataRx), HEX);
                Firmata.processInput(Firmata.rx64.getData(contDataRx));
              }  
        }
   }
  
  /* SEND FTDI WRITE BUFFER - make sure that thesendAnalogsendAnalog FTDI buffer doesn't go over
   * 60 bytes. use a timer to sending an event character every 4 ms to
   * trigger the buffer to dump. */
  if ((currentMillis - previousMillis) > samplingInterval) {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {      
      if (Firmata.flagStreaming==0 && (portConfigInputs[pin / 8] & (1 << (pin & 7)))) {
          portValue = (readPort((pin/8), portConfigInputs[pin/8])) & portConfigInputs[pin / 8];
          Firmata.storeSamplingPacket(pin, portValue, 0x01);
      }     
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          if(Firmata.flagStreaming==1){
            Firmata.sendAnalog(analogPin, analogRead(analogPin));
            }  else {  //store error
              Firmata.storeSamplingPacket(analogPin, analogRead(analogPin), 2);
            }
          }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes);
      }
    }
  }
}
