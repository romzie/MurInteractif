#include <Wire.h>
#include <NeoPixelBus.h>

#define LED_NO 2
#define CHAN_NB 4

const byte FDC = 0x2A; // FDC adress either Ox2A or 0X2B
const byte CHMSB[] = {0x00, 0x02, 0x04, 0x06}; // DATA channels with conversion result and status
const byte CHLSB[] = {0x01, 0x03, 0x05, 0x07}; // DATA_LSB channels must be read after DATA channel

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(LED_NO);
uint8_t neoRED[LED_NO];
uint8_t neoGREEN[LED_NO];
uint8_t neoBLUE[LED_NO];
uint8_t neoBRIGHT[LED_NO]; // 0 = black // 255 = white

long refMeasure[CHAN_NB]; // references values
long measure[CHAN_NB]; // dynamic values
long oldMeasure[CHAN_NB]; // previous dynamic values
long referencingRate = 200; // number of samples to initialize references values
long samplingRate = 100; // number of samples to do a measure
long errorMargin[CHAN_NB]; // equals dispersion of references values during sampling divided by 4

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

//Update actual values
void updateMeasure() {
  long measure;
  for (int i = 0; i < CHAN_NB; i++) {
    oldMeasure[i] = measure[i];
    for (int i = 0; i < samplingRate; i++) {
      measure[i] = (measure[i]*i + readChannel(i))/(i+1);
    }
  }
}

//Initializing of references values
void initializeReference() {
  long average = 0;
  long value;
  long max;
  for (int c = 0; c < CHAN_NB; c++) {
    max = 0;
    for (int i = 0; i < referencingRate; i++) {
      value = readChannel(c);
      average = average + value;
      if (value > max) {
        max = value;
      }
    }
    average = average/referencingRate;
    errorMargin[c] = (max - average)/2;
    refMeasure[c] = average + errorMargin[c];
  }
}

void updateLEDs() {
  RgbColor color;
  color.R = 255;
  color.G = 0;
  color.B = 0;

  // strip.SetPixelColor(0, color);
  // strip.SetPixelColor(1, color);

  // sensors slider
  long diff[CHAN_NB];
  long sum = 0;

  for (int i = 0; i < CHAN_NB; i++) {
    diff[i] = measure[i] - refMeasure[i];
    sum = sum + diff[i];
  }
  if (diff[0] > 0 || diff[1] > 0) { // presence detected
    if (diff[1] <= 0) { // not on 1 at all
      neoBRIGHT[0] = 255;
      neoBRIGHT[1] = 0;
      strip.SetPixelColor(1,color);
      color.R = 0;
      color.G = 255;
      color.B = 0;
      strip.SetPixelColor(0, color);
    } else if (diff[0] <= 0) { // not on 0
      neoBRIGHT[0] = 0;
      neoBRIGHT[1] = 255;
      strip.SetPixelColor(0,color);
      color.R = 0;
      color.G = 255;
      color.B = 0;
      strip.SetPixelColor(1, color);
    } else {
      for (int i = 0; i < CHAN_NB; i++) {
        neoBRIGHT[i] = diff[i]/sum; // x is sensor x
        strip.SetPixelColor(0, color);
        strip.SetPixelColor(1, color);
      }
    }
  }
  strip.Show();
}

void updateSlider() {
  updateLEDs();
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
  writeConfig(FDC, 0x09, 0xFF, 0xFF);//RCOUNT_CH1
  writeConfig(FDC, 0x0A, 0xFF, 0xFF);//RCOUNT_CH2
  writeConfig(FDC, 0x0B, 0xFF, 0xFF);//RCOUNT_CH3
  writeConfig(FDC, 0x10, 0x04, 0x00);//SETTLECOUNT_CH0
  writeConfig(FDC, 0x11, 0x04, 0x00);//SETTLECOUNT_CH1
  writeConfig(FDC, 0x12, 0x04, 0x00);//SETTLECOUNT_CH2
  writeConfig(FDC, 0x13, 0x04, 0x00);//SETTLECOUNT_CH3
  writeConfig(FDC, 0x14, 0x10, 0x01);//CLOCK_DIVIDERS_CH0
  writeConfig(FDC, 0x15, 0x10, 0x01);//CLOCK_DIVIDERS_CH1
  writeConfig(FDC, 0x16, 0x10, 0x01);//CLOCK_DIVIDERS_CH2
  writeConfig(FDC, 0x17, 0x10, 0x01);//CLOCK_DIVIDERS_CH3
  writeConfig(FDC, 0x19, 0x00, 0x01);//ERROR_CONFIG
  writeConfig(FDC, 0x1A, 0x1E, 0x01);//CONFIG
  writeConfig(FDC, 0x1B, 0xC2, 0x0C);//MUX_CONFIG
  writeConfig(FDC, 0x1E, 0x8C, 0x40);//DRIVE_CURRENT_CH0
  writeConfig(FDC, 0x1F, 0x8C, 0x40);//DRIVE_CURRENT_CH1
  writeConfig(FDC, 0x20, 0x88, 0x00);//DRIVE_CURRENT_CH2
  writeConfig(FDC, 0x21, 0x88, 0x00);//DRIVE_CURRENT_CH3
}

void setup() {
  strip.Begin();
  strip.Show();

  Serial.begin(9600); //écran de debug

  pinMode(A0,INPUT);
	pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output
	pinMode(D6, OUTPUT); // ADDR
	pinMode(D7, OUTPUT); // ShutDown

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
  initializeReference();
}

void loop() {
  updateMeasure();
  updateSlider();
  //strip.Show();
  delay(500);
}
