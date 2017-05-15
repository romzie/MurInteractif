#include <Wire.h>
#include <NeoPixelBus.h>

#define LED_NO 14
#define CHAN_NB 4


RgbColor green;
RgbColor red;
RgbColor white;
HsbColor blackHSB;

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
long errorMargin[CHAN_NB]; // margin to detect a presence
long diff[CHAN_NB]; // for detecting presence


int mode = 0; /* current state of the prototype :
                * 0 off mode
                * 1 color selection mode
                * 2 brightness selection mode
                * 3 slider mode
                */
HsbColor mainColor; // selected color
HsbColor sliderColor; // slider color
HsbColor standbyColor; // selected color for standby
bool brightUp = true; // boolean for the standby mode
float brightPulse = 0.1; // intervalle de modification de la luminosité du standby


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
  for (int i = 0; i < CHAN_NB; i++) {
    oldMeasure[i] = measure[i];
    measure[i] = 0;
    for (int j = 0; j < samplingRate; j++) {
      measure[i] = measure[i] + readChannel(i) / samplingRate;
    }
  }
}

//Initializing of references values
void initializeReference() {
  long average = 0;
  long value;
  long min;
  delay(2000);
  for (int c = 0; c < CHAN_NB; c++) {
    min = 0x7FFFFFFF;
    average = 0;
    for (int i = 0; i < referencingRate; i++) {
      value = readChannel(c);
      average = average + value / referencingRate;
      if (value < min) {
        min = value;
      }
      delay(5);
    }
    //average = (long)average/referencingRate;
    errorMargin[c] = (average - min) * 45 / 100; //% error margin
    refMeasure[c] = min + errorMargin[c]; 
  }
}

// color initialisation
void initColors() {
  green.R = 0;
  green.G = 255;
  green.B = 0;

  red.R = 255;
  red.G = 0;
  red.B = 0;

  white.R = 255;
  white.G = 255;
  white.B = 255;

  blackHSB.H = 0;
  blackHSB.S = 0;
  blackHSB.B = 0;
}

void initMainColor() {
  mainColor.H = 0.5;
  mainColor.S = 1;
  mainColor.B = 0.5;
  sliderColor = mainColor;
  standbyColor = mainColor;
}

// update diff array
void checkPresence() {
  for (int i = 0; i < CHAN_NB; i++) {
    diff[i] = measure[i] - refMeasure[i];
    if (diff[i] > 0) {
      diff[i] = 0;
    }
  }
  
  Serial.print("Mode : ");
  Serial.print(mode);
  Serial.print("  |  CH0  ");
  Serial.print(diff[0]);
  Serial.print("     ");
  Serial.print(refMeasure[0]);
  Serial.print("     ");
  Serial.print(measure[0]);


  Serial.print("  |  CH1  ");
  Serial.print(diff[1]);
  Serial.print("     ");
  Serial.print(refMeasure[1]);
  Serial.print("     ");
  Serial.print(measure[1]);

  Serial.print("  |  CH2  ");
  Serial.print(diff[2]);
  Serial.print("     ");
  Serial.print(refMeasure[2]);
  Serial.print("     ");
  Serial.print(measure[2]);

  Serial.print("  |  CH3  ");
  Serial.print(diff[3]);
  Serial.print("     ");
  Serial.print(refMeasure[3]);
  Serial.print("     ");
  Serial.println(measure[3]);
}

void updateSlider() {
  if (diff[0] < 0 || diff[1] < 0) { // presence detected
    if (diff[1] >= 0) { // not on 1 at all
      gradient(0, 13, red, green);
    } else if (diff[0] >= 0) { // not on 0
      gradient(0, 13, green, red);
    } else {
      int numled = (int)(14 * diff[0] / (diff[0]+diff[1]) - 1);
      gradient(0, numled, red, green);
      gradient(numled, 13, green, red);
    }
  }
}

int numG = 0;

// circle slider with 3 sensors
void updateCircleSlider() {
  if (diff[0] < 0 || diff[1] < 0 || diff[2] < 0) { // presence detected

    switch (mode) {
          case 1: //color
            if (diff[1] > diff[2]) { // using sensor proportion to locate the user on the circle
              sliderColor.H = (360*diff[0] + 120*diff[1] + 240*diff[2])/((diff[0] + diff[1] + diff[2])*360.0);
            } else {
              sliderColor.H = (120*diff[1] + 240*diff[2])/((diff[0] + diff[1] + diff[2])*360.0);
            }
            Serial.print("COLOR.H : ");
            Serial.println(sliderColor.H);
            gradient(0,13,sliderColor, sliderColor);
            break;
          case 2: //bright
            float tmp;
            if (diff[1] > diff[2]) { 
              tmp = (360*diff[0] + 120*diff[1] + 240*diff[2])/((diff[0] + diff[1] + diff[2])*360.0);
            } else {
              tmp = (120*diff[1] + 240*diff[2])/((diff[0] + diff[1] + diff[2])*360.0);
            }
            if (tmp > 0.5) {
              sliderColor.B = 1 - tmp;
            } else {
              sliderColor.B = tmp;
            }
            Serial.print("COLOR.B : ");
            Serial.println(sliderColor.B); 
            gradient(0,13,sliderColor, sliderColor);
            break;
          case 3: //slider
            
            if (diff[1] > diff[2]) { 
              tmp = (360*diff[2] + 120*diff[1] + 240*diff[0])/((diff[0] + diff[1] + diff[2])*360.0);
            } else {
              tmp = (120*diff[1] + 240*diff[0])/((diff[0] + diff[1] + diff[2])*360.0);
            }
            tmp = (int(tmp*13)+6)%14;
            Serial.print("NO LED : ");
            Serial.println(tmp);
            HSBgradientNoir(tmp, sliderColor);
            
            /*
            numG = (numG +1) % 14;
            HSBgradientNoir(numG, sliderColor);
            */         
            break;
          default: break;
    }

    
  }
}


// button with sensor number no
void updateButton(int no) {
  if (diff[no] < 0) { // presence detected
    mode = (mode + 1)%4;
    mainColor = sliderColor;
    if (mode == 0) {
      standbyColor = mainColor; //reset standby mode;
    }
    if (mode == 1 || mode == 2) {
        sliderColor = mainColor;
    }
    delay(1500);
  }
}

void HSBgradientNoir(int top, HsbColor color) {
  HsbColor Ncolor;
  Ncolor.H = color.H;
  Ncolor.S = color.S;
  for (int i = 0; i <= 7; i = i+1 % 14){
    Ncolor.B = color.B*(7-i)/7;
    strip.SetPixelColor((top+i)%14,Ncolor);
    strip.SetPixelColor((14+top-i)%14,Ncolor);
  }
}


void HSBgradient(int first, int last, HsbColor colorFirst, HsbColor colorLast) {
  int diffLUM = colorLast.B - colorFirst.B;
  int noLED;

  HsbColor newColor;
  if (colorFirst.S == 0) {
    newColor.H = colorLast.H;
    newColor.S = colorLast.S;
  } else {
    newColor.H = colorFirst.H;
    newColor.S = colorFirst.S;
  }
  for (int i = 0; i < last - first + 1; i++) {
    newColor.B = colorFirst.B + diffLUM * i / (last - first);
    noLED = (i + first)%14;
    strip.SetPixelColor(noLED, newColor);
  }

}

void gradient(int first, int last, RgbColor colorFirst, RgbColor colorLast) {
  int diffRED = colorLast.R - colorFirst.R;
  int diffGRE = colorLast.G - colorFirst.G;
  int diffBLU = colorLast.B - colorFirst.B;
  int noLED;

  for (int i = 0; i < last - first + 1; i++) {
    RgbColor newColor;
    newColor.R = colorFirst.R + diffRED * i / (last - first);
    newColor.G = colorFirst.G + diffGRE * i / (last - first);
    newColor.B = colorFirst.B + diffBLU * i / (last - first);
    noLED = (i + first)%14;
    strip.SetPixelColor(noLED, newColor);
  }
}

void standby() {
  if (brightUp == true) {
    standbyColor.B = standbyColor.B + brightPulse;
  } else {
    standbyColor.B = standbyColor.B - brightPulse;
  }
  if (standbyColor.B >= 1 - 9*brightPulse/10) {
    brightUp = false;
  } else if (standbyColor.B <= 9*brightPulse/10) {
    brightUp = true;
  }
  Serial.print("LUMINOSITE : ");
  Serial.println(standbyColor.B);
  gradient(0, LED_NO, standbyColor, standbyColor);
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
  Serial.begin(9600); //écran de debug
  strip.Begin();

  pinMode(A0, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output
  pinMode(D6, OUTPUT); // ADDR
  pinMode(D7, OUTPUT); // ShutDown

  digitalWrite(D6, LOW); // LOW --> Ox2A
  digitalWrite(D7, LOW); // exit ShutDown

  /*
     D1 <= SCL
     D2 <= SDA
     D6 <= SD
     D7 <= ADDR
     INTB <= not used

     DATA LED <= RX
  */

  Wire.begin();
  Configuration();
  initializeReference();
  
  initColors();
  initMainColor();

  gradient(13, 0, mainColor, mainColor);

  strip.Show();
  delay(2000);
}

void loop() {
  updateMeasure();
  checkPresence();
  
  //updateSlider();
  updateButton(3);
  if (mode != 0) {
    updateCircleSlider();
  } else {
    standby();
  }
 
  strip.Show();
  delay(10);
}
