#include <Arduino.h>
#include <EEPROM.h>
#include <VirtualWire.h>
#include <EasyTransferVirtualWire.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "sha1.h"

// Constants
const unsigned int AWAKE_SLEEP_INTERVAL_COUNT = 4;
const int LED_POWER_PIN = 13;
const int TRANSMIT_DATA_PIN = 7;
const int TRANSMIT_POWER_PIN = 8;
const int PIR_DATA_PIN = 2;
const int PIR_POWER_PIN = 3;
const unsigned char* SECRET = "QHcGpCh?mAzQ7vCW#4SZnZ5-2-r%2kfL";
const unsigned int SECRET_LENGTH = 32;
const byte COMMAND_TYPE = 5;
const float PACKET_DATA = 1;

// Device ID
unsigned int uniqueDeviceId = 0;

// Current packet number
unsigned int packetNumber = 0;

// EasyTransfer lib object
EasyTransferVirtualWire ET;

// Init PIR state variables
volatile int pirSensorState = 0;

// On signal level change (0 -> 1 or 1 -> 0)
void onWakeUp() {
  pirSensorState = (digitalRead(pirDataPin) == HIGH);
}

// Packet structure
struct SEND_DATA_STRUCTURE {
  unsigned int sourceId; //4 bytes
  unsigned int packetNumber; //4 bytes
  byte commandType; // 1 byte
  float data; // 4 bytes
  byte hmac0; // 1 byte
  byte hmac2; // 1 byte
  byte hmac3; // 1 byte
  byte hmac4; // 1 byte
  byte hmac5; // 1 byte
  byte hmac6; // 1 byte
  byte hmac8; // 1 byte
  byte hmac11; // 1 byte
  byte hmac13; // 1 byte
  byte hmac15; // 1 byte
  byte hmac17; // 1 byte
  byte hmac18; // 1 byte
  byte hmac19; // 1 byte
};

// Current packet
SEND_DATA_STRUCTURE packet;

// Write EEPROM API
void EEPROMWriteInt(int address, unsigned int value)
{
	byte lowByte = ((value >> 0) & 0xFF);
	byte highByte = ((value >> 8) & 0xFF);

	EEPROM.write(address, lowByte);
	EEPROM.write(address + 1, highByte);
}

// Read EEPROM API
unsigned int EEPROMReadInt(int address)
{
	byte lowByte = EEPROM.read(address);
	byte highByte = EEPROM.read(address + 1);

	return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

// Returns unique unsigned int Device ID
int getDeviceId()
{
	int uniqueDeviceId = 0;

	Serial.print("Getting Device ID...");
	uniqueDeviceId = EEPROMReadInt(0);

	if (uniqueDeviceId < 10000 || uniqueDeviceId > 60000) {
		Serial.print("N/A, updating... ");
		randomSeed(analogRead(0));
		uniqueDeviceId = random(10000, 60000);
		EEPROMWriteInt(0, uniqueDeviceId);
	}

	Serial.println(uniqueDeviceId);

	return uniqueDeviceId;
}

// Sends pir alarm
void sendData()
{
  unsigned char *hash;

  // Calculating hash
  Sha1.initHmac(SECRET, SECRET_LENGTH);
  Sha1.print(uniqueDeviceId);
  Sha1.print(packetNumber);
  Sha1.print(COMMAND_TYPE);
  Sha1.print(PACKET_DATA);

  hash = Sha1.resultHmac();

  // Constructing packet
  packet.sourceId = uniqueDeviceId;
  packet.packetNumber = packetNumber;
  packet.commandType = COMMAND_TYPE;
  packet.data = PACKET_DATA;

  packet.hmac0 = hash[0];
  packet.hmac2 = hash[2];
  packet.hmac3 = hash[3];
  packet.hmac4 = hash[4];
  packet.hmac5 = hash[5];
  packet.hmac6 = hash[6];
  packet.hmac8 = hash[8];
  packet.hmac11 = hash[11];
  packet.hmac13 = hash[13];
  packet.hmac15 = hash[15];
  packet.hmac17 = hash[17];
  packet.hmac18 = hash[18];
  packet.hmac19 = hash[19];

  Serial.println(packet.packetNumber);
  Serial.println(packet.data);
  Serial.println();

  ET.sendData();
}

// Enables sleep mode
void sleep()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ADCSRA &= ~(1 << 7);
  sleep_enable();
  sleep_bod_disable();

  attachInterrupt(0, onWakeUp, CHANGE);

  sleep_mode();
  sleep_disable();
  detachInterrupt(PIR_DATA_PIN);
}

// Initialization part
void setup()
{
  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(PIR_POWER_PIN, OUTPUT);
  pinMode(TRANSMIT_DATA_PIN, OUTPUT);
  pinMode(TRANSMIT_POWER_PIN, OUTPUT);
  pinMode(PIR_DATA_PIN, INPUT);
  digitalWrite(PIR_DATA_PIN, HIGH);

  digitalWrite(PIR_POWER_PIN, HIGH);

  // Delay execution of sketch for a min, to allow PIR sensor get stabilized
  for (int i = 1; i <= 120; i++) {
    digitalWrite(LED_POWER_PIN, HIGH);
    delay(100);
    digitalWrite(LED_POWER_PIN, LOW);
    delay(100);
   }

  Serial.begin(9600); // Debugging only

  ET.begin(details(packet));
  vw_set_tx_pin(TRANSMIT_DATA_PIN);
  vw_setup(2000); // 2k bits ber second

  randomSeed(analogRead(0));

  uniqueDeviceId = getDeviceId();
}

void loop()
{
  interrupts();

  if (pirSensorState == 0) {
    digitalWrite(LED_POWER_PIN, LOW);
  }
  else {
   digitalWrite(LED_POWER_PIN, HIGH);
   digitalWrite(TRANSMIT_POWER_PIN, HIGH);
   delay(500);

   sendData();
   delay(50);
   sendData();
   delay(50);
   sendData();

   digitalWrite(TRANSMIT_POWER_PIN, LOW);

   // Increasing current packet number
   packetNumber++;
  }

  // Sleep again
  sleep();
}
