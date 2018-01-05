/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * Contribution by a-lurker and Anticimex,
 * Contribution by Norbert Truchsess <norbert.truchsess@t-online.de>
 * Contribution by Tomas Hozza <thozza@gmail.com>
 *
 *
 * DESCRIPTION
 * The EthernetGateway sends data received from sensors to the ethernet link.
 * The gateway also accepts input on ethernet interface, which is then sent out to the radio network.
 *
 * The GW code is designed for Arduino 328p / 16MHz.  ATmega168 does not have enough memory to run this program.
 *
 * LED purposes:
 * - To use the feature, uncomment MY_DEFAULT_xxx_LED_PIN in the sketch below
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/ethernet_gateway for wiring instructions.
 *
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable gateway ethernet module type
#define MY_GATEWAY_W5100

// W5100 Ethernet module SPI enable (optional if using a shield/module that manages SPI_EN signal)
//#define MY_W5100_SPI_EN 4

// Enable Soft SPI for NRF radio (note different radio wiring is required)
// The W5100 ethernet module seems to have a hard time co-operate with
// radio on the same spi bus.
//#if !defined(MY_W5100_SPI_EN) && !defined(ARDUINO_ARCH_SAMD)
//#define MY_SOFTSPI
//#define MY_SOFT_SPI_SCK_PIN 14
//#define MY_SOFT_SPI_MISO_PIN 16
//#define MY_SOFT_SPI_MOSI_PIN 15
//#endif

// When W5100 is connected we have to move CE/CSN pins for NRF radio
//#ifndef MY_RF24_CE_PIN
//#define MY_RF24_CE_PIN 5
//#endif
//#ifndef MY_RF24_CS_PIN
//#define MY_RF24_CS_PIN 6
//#endif

// Enable to UDP
//#define MY_USE_UDP

#define MY_IP_ADDRESS 192,168,10,204   // If this is disabled, DHCP is used to retrieve address
// Renewal period if using DHCP
//#define MY_IP_RENEWAL_INTERVAL 60000
// The port to keep open on node server mode / or port to contact in client mode
#define MY_PORT 5003

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 10, 51

// The MAC address can be anything you want but should be unique on your network.
// Newer boards have a MAC address printed on the underside of the PCB, which you can (optionally) use.
// Note that most of the Ardunio examples use  "DEAD BEEF FEED" for the MAC address.
#define MY_MAC_ADDRESS 0xDE, 0xAD, 0xBE, 0xAA, 0xAA, 0x01

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
#define MY_INCLUSION_MODE_BUTTON_PIN  23

// Set blinking period
//#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Flash leds on rx/tx/err
// Uncomment to override default HW configurations
//#define WITH_LEDS_BLINKING_INVERSE
#define MY_DEFAULT_ERR_LED_PIN 20  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  21  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  22  // Transmit led pin


#if defined(MY_USE_UDP)
#include <EthernetUdp.h>
#endif
#include <Ethernet.h>
#include <MySensors.h>

//#define RELAY_PIN 7  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
//#define NUMBER_OF_RELAYS 3 // Total number of attached relays
#define RELAY_ON HIGH  // GPIO value to write to turn on attached relay
#define RELAY_OFF LOW // GPIO value to write to turn off attached relay

struct Device
{
  int nChildId;
  uint8_t nPin;
  uint8_t pinmode;
  mysensor_sensor sensorType;
  bool bRetainState;
};

const bool RETAINSTATE = true;

const uint8_t INPUT_00 = A0;
const uint8_t INPUT_01 = A1;
const uint8_t OUTPUT_02 = 2;
const uint8_t OUTPUT_03 = 3;
const uint8_t OUTPUT_05 = 5;
const uint8_t OUTPUT_06 = 6;
const uint8_t OUTPUT_07 = 7;
const uint8_t OUTPUT_08 = 8;
const uint8_t OUTPUT_A5 = A5;

const Device AR_DEVICES[] = {{ 1, INPUT_00, INPUT_PULLUP, S_BINARY, false }
                            ,{ 2, INPUT_01, INPUT_PULLUP, S_BINARY, false }
                            ,{ 3, OUTPUT_02 , OUTPUT, S_BINARY, !RETAINSTATE }
                            ,{ 4, OUTPUT_03 , OUTPUT, S_DIMMER, !RETAINSTATE }    // --> Change to PWM later
                            ,{ 5, OUTPUT_05 , OUTPUT, S_DIMMER, !RETAINSTATE }    // --> Change to PWM later
                            ,{ 6, OUTPUT_06 , OUTPUT, S_DIMMER, !RETAINSTATE }    // --> Change to PWM later
                            ,{ 7, OUTPUT_07 , OUTPUT, S_BINARY, !RETAINSTATE }
                            ,{ 8, OUTPUT_08 , OUTPUT, S_BINARY, !RETAINSTATE }
                            ,{ 9, OUTPUT_A5 , OUTPUT, S_BINARY, !RETAINSTATE }
                            };

// This will execute before MySensors starts up
void before()
{
  //Setup devices
  for (int i = 0; i < ARRAY_SIZE(AR_DEVICES); i++)
  {
    // Set pinmode
    pinMode(AR_DEVICES[i].nPin, AR_DEVICES[i].pinmode);
    
    // Set relay to last known state (using eeprom storage). Set to off if no retain.
    if (AR_DEVICES[i].pinmode == OUTPUT)
    {
      bool bOutputState = AR_DEVICES[i].bRetainState ? loadState(AR_DEVICES[i].nChildId) : false;
      digitalWrite(AR_DEVICES[i].nPin, bOutputState ? RELAY_ON : RELAY_OFF);
    }
  }
}

// Setup() is executed AFTER mysensors has been initialised.
void setup()
{

}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Relay", "1.0");

  // Present inputs
  for (int i = 0; i < ARRAY_SIZE(AR_DEVICES); i++)
  {
    present(AR_DEVICES[i].nChildId, AR_DEVICES[i].sensorType);
  }
}

void receive(const MyMessage &message)
{
    switch (message.type){
    case V_STATUS:
      // Change relay state
      digitalWrite(AR_DEVICES[message.sensor].nPin, message.getBool() ? RELAY_ON : RELAY_OFF);

      // Store state in eeprom if retaining is enabled
      if (AR_DEVICES[message.sensor].bRetainState)
      {
        saveState(message.sensor, message.getBool());
      }

      Serial.print("Incoming change for sensor:");
      break;
    case V_PERCENTAGE:
      // --> ToDo          
      Serial.println("--> Work to do: no status altered.");
      Serial.print("Incoming percentage change for sensor:");
      break;
    default:
      break;      
    }

    // Write some debug info    
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getInt());
}

//int SwitchState_Last[ARRAY_SIZE(AR_DEVICES)];

void loop()
{  
  //for (int i = 0; i < ARRAY_SIZE(AR_DEVICES); i++)
  //{
  //  if (AR_DEVICES[i].pinmode == INPUT_PULLUP)
  //  {
  //    bool bCurrentInputState = !digitalRead(AR_DEVICES[i].nPin);
  //    
  //    if (SwitchState_Last[i] != bCurrentInputState)
  //    {
  //      //send(MyMessage(AR_DEVICES[i].nChildId, V_STATUS).setSensor(sensorId).setDestination(nodeId).set(true));
  //      
  //      MyMessage msg(AR_DEVICES[i].nChildId, V_STATUS);
  //      send(msg.setDestination(2).set(bCurrentInputState));

  //    }
  //    
  //    SwitchState_Last[i] = bCurrentInputState;      
  //  }
  //}
    
}
