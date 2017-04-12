#include <Wire.h>
#include <NeoPixelBus.h>

const byte FDC = 0x2A; // FDC adress either Ox2A or 0X2B
const byte CHMSB[] = {0x00, 0x02, 0x04, 0x06}; // DATA channels with conversion result and status
const byte CHLSB[] = {0x01, 0x03, 0x05, 0x07}; // DATA_LSB channels must be read after DATA channel

long refMesure; // reference value

//Build the complete conversion result from the specific channel
long readChannel(uint8_t no) {
  unsigned long val = 0;
  word c = 0;
  word d = 0;
  c = readValue(FDC, CHMSB[no]);
  d = readValue(FDC, CHLSB[no]);
  val = c;
  val <<= 16;
  val += d;
  return (long)val;
}


//Read bytes from register channel specified
word readValue (int FDC, int reg) {
  byte a = 0;
  byte b = 0;
  word value = 0;
  Wire.beginTransmission(FDC);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(FDC, 2);
  while (Wire.available())
  {
    a = Wire.read();
    b = Wire.read();
  }
  value = a;
  value <<= 8;
  value += b;
  return value;
}

//Configuring the FDC2214
void writeConfig(int FDC, byte reg, byte MSB, byte LSB) {
  Wire.beginTransmission(FDC);
  Wire.write(reg);
  Wire.write(MSB);
  Wire.write(LSB);
  Wire.endTransmission();
}

void Configuration() {
	writeConfig(FDC, 0x08, 0xFF, 0xFF);//RCOUNT_CH0
  writeConfig(FDC, 0x10, 0x04, 0x00);//SETTLECOUNT_CH0
  writeConfig(FDC, 0x14, 0x10, 0x01);//CLOCK_DIVIDERS_CH0
  writeConfig(FDC, 0x19, 0x00, 0x01);//ERROR_CONFIG
  writeConfig(FDC, 0x1A, 0x1E, 0x01);//CONFIG
  writeConfig(FDC, 0x1B, 0xC2, 0x0C);//MUX_CONFIG
  writeConfig(FDC, 0x1E, 0x8C, 0x40);//DRIVE_CURRENT_CH0	
}

void setup() {

	pinMode(A0, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output
	pinMode(D6, OUTPUT); // ShutDown
	pinMode(D7, OUTPUT); // ADDR
  Serial.begin(9600); // fait sortir un écran pour l'affichage
	digitalWrite(D6, LOW); // LOW --> Ox2A
	digitalWrite(D7, LOW); // exit ShutDown
 /*
  * D1 <= SCL
  * D2 <= SDA
  * D6 <= SD
  * D7 <= ADDR
  * INTB <= not used
  */

	Wire.begin();
	Configuration();

	refMesure = readChannel(0); // initialize reference value

 Serial.println(refMesure);
}

void loop() {
	long mesure = readChannel(0);
  Serial.println(mesure);
	if (mesure > refMesure) { 
		digitalWrite(LED_BUILTIN, HIGH);
	} else {
		digitalWrite(LED_BUILTIN, LOW);
	}
 delay(2000);
}
