#include <Wire.h>
#include <NeoPixelBus.h>
//#include <ESP8266WiFi.h>
//#include <WiFiClient.h>
//#include <ESP8266WebServer.h>
//#include <WebSocketsServer.h>
//#include <FS.h>

#define CMIN -33000 // conservative minimum DI value
#define CMAX -2300 // conservative max DI value
#define LED_NO 92
#define MINSPREAD 5.0 // how many LEDs away of detected position the light should be cut off
#define LED_SHIFT -30 // number of LEDs to shift the position/angle
#define WAIT_BEFORE_FADE 1000 // ms

const byte FDC = 0x2A;// FDC address either 0x2A or 0x2B;
const byte CHMSB[] = {0x00, 0x02, 0x04, 0x06};
const byte CHLSB[] = {0x01, 0x03, 0x05, 0x07};
const uint8_t g_nice_hue[] = {0, 64, 96, 128, 160, 192, 255};

uint8_t g_t_hue = 0; // target hue: index of g_nice_hue. target hue = g_nice_hue[g_t_hue]
RgbColor g_t_c(0, 0, 0); // target color
uint8_t g_c_hue = 0; // current hue [0, 255]
uint8_t g_led_hue[LED_NO], g_led_l[LED_NO];
unsigned long g_led_t[LED_NO]; // last update of each led

// mesured values
long cmesure[4];
long di[4]; // derivative integral of cmesure

bool g_new_data = false;
// features
float g_r = -1.0; // distance to center (radius) [0,1] or -1 for unknown
uint8_t g_pos = 0; // angular position [0,255]
uint16_t g_dist = 0; // hand distance [0,255], used for setting intensity, 0: far, 255: close

unsigned long g_prev_time = millis(); // previous loop time
unsigned long g_d = g_prev_time; // last dimming action
unsigned long g_lock_time = g_prev_time;
bool g_locked = false;

// ------------ wifi setup ------------
const char *ssid = "SystemiModerni";
const char *password = "EricRomeo";

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(LED_NO);
//ESP8266WebServer server(80);
//WebSocketsServer webSocket = WebSocketsServer(8000);

//void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
//  switch (type) {
//    case WStype_DISCONNECTED:
//      Serial.printf("[%u] Disconnected!\n", num);
//      break;
//    case WStype_CONNECTED: {
//        IPAddress ip = webSocket.remoteIP(num);
//        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
//        //webSocket.sendTXT(num, "Connected");
//      } break;
//    case WStype_TEXT: {
//        Serial.printf("[%u] get Text: %s\n", num, payload);
//        g_dist = atoi((char*)payload);
//        char* space = strchr((char*)payload, ' ');
//        if (space == 0) Serial.println("damn");
//        float new_r = atof(space++);
//        space = strchr(space, ' ');
//        if (space == 0) Serial.println("damn2");
//        float pos = atof(space + 1);
//        g_pos = (uint8_t)constrain((pos + 3.141593) * 255.0 / 6.283186, 0, 255);
//        long new_time = millis();
//        g_dr = 1000 * (new_r - g_r) / (new_time - g_prev_time);
//        g_prev_time = new_time;
//        g_r = new_r;
//        g_new_data = true;
//        //        Serial.print(g_dist);
//        //        Serial.print("\t r: ");
//        //        Serial.print(g_r);
//        //        Serial.print("\t dr: ");
//        //        Serial.print(g_dr);
//        //        Serial.print("\t");
//        //        Serial.println(g_pos);
//      }
//  }
//}


//format bytes
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}


String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  return "text/plain";
}


bool handleFileRead(String path) {
  //  Serial.println("handleFileRead: " + path);
  //  if (path.endsWith("/")) path += "index.html";
  //  String contentType = getContentType(path);
  //  if (SPIFFS.exists(path)) {
  //    Serial.println("streaming " + contentType);
  //    File file = SPIFFS.open(path, "r");
  //    server.streamFile(file, contentType);
  //    file.close();
  //    return true;
  //  }
  //  return false;
}


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


//IIR Filter
long average(long meassure, long avg_1, int n) {
  return (long)(((avg_1 * n) - avg_1 + meassure) / n);
}


void monitorCapa() {
  long mesure[4], dist[4];
  for (int i = 0; i < 4; ++i) {
    mesure[i] = readChannel(i);
    //mesure[i] = average(readChannel(i),cmesure[i], 8);
    long d = di[i] + (mesure[i] - cmesure[i]);
    di[i] = average(d , di[i], 8);
    cmesure[i] = mesure[i]; // keep as t-1
    dist[i] = (CMAX - di[i]) * 1000 / (CMAX - CMIN); // scale to [0,1000]
    dist[i] = constrain(dist[i], 0, 1000);
  }
  getFeatures(dist);
}


void getFeatures(long* dist) { // input: per-electrode distance [0,1000], output: global hand dist, R, angular position
  float s = (float)(dist[1] + dist[2] + dist[3]);
  g_dist = (uint16_t)constrain((s + dist[0]) / 4, 0, 255); // global hand distance [0,255]
  //uint16_t ndist = (uint16_t)constrain(s + dist[0], 0, 1000); // global hand distance [0,1000]
  //g_dist = average(ndist, g_dist, 16);
  if (g_dist <= 0 ) {
    g_r = -1.0;
    return; // don't modify the rest
  }
  float radius = s / (s + dist[0]); // [0,1]
  if ( radius > 0.75 ) {
    g_pos = getAngle(dist);
    // set to 1
    if ( g_r == -1.0 ) g_r = 1.0;
    if ( g_r != 1.0 ) {
      g_t_hue = (g_t_hue + 1) % 7; // change target color
      g_r = 1.0;
    }
  }
  if ( radius < 0.4 ) {
    // set to 0
    if ( g_r == -1.0 ) g_r = 0.0;
    if ( g_r != 0.0 ) {
      g_t_hue = (g_t_hue + 1) % 7; // change target color
      g_r = 0.0;
    }
  }
}


uint8_t getAngle(long* dist) {
  float cm; // barycenter
  if ( dist[2] < dist[1] && dist[2] < dist[3]) {
    cm = (float)(2 * dist[2] + 3 * dist[3] + 4 * dist[1]) / (float)(dist[1] + dist[2] + dist[3]);
    if (cm > 3.0) cm = cm - 3.0;
  } else {
    cm = (float)(dist[1] + 2 * dist[2] + 3 * dist[3]) / (float)(dist[1] + dist[2] + dist[3]);
  }
  int16_t scm = (int16_t)((255.0 - cm * 255.0) / 3.0 - 255 * LED_SHIFT / (float)LED_NO);
  if (scm < 1) scm = scm + 255;
  return (uint8_t)scm; // [1,255]
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

bool updateLeds() {
  unsigned long now = millis();
  if (now - g_d < 30) {
    return false;
  }
  g_d = now;
  bool res = pushToTargetHue();
  if (g_dist == 0) {
    for (uint8_t i = 0; i < LED_NO; ++i) {
      if ( dimm(i, now) ) res = true;
    }
    return res;
  }
  if (g_r <= 0.0 && g_dist > 0) {
    for (uint8_t i = 0; i < LED_NO; ++i) {
      g_led_t[i] = now;
      if ( lighten(i, now) ) res = true;
    }
    return res;
  }
  uint8_t spread = MINSPREAD + (uint8_t)(3 * g_dist * MINSPREAD / 255); // [MINSPREAD + r_spread, 4*MINSPREAD]
  //uint8_t spread = MINSPREAD + (uint8_t)(3 * g_dist * MINSPREAD / 1000) + r_spread; // [MINSPREAD + r_spread, 4*MINSPREAD + r_spread]
  uint16_t pos = (uint16_t)g_pos * LED_NO / 255;
  uint8_t low = pos - spread;
  uint8_t high = pos + spread;
  if ( pos + spread >= LED_NO ) {
    high = pos + spread - LED_NO;
  }
  if ( pos < spread ) {
    low = LED_NO - spread + pos;
  }
  // dimm all outside of spread
  for (uint8_t i = 0; i < LED_NO; ++i) {
    if (i < pos - spread || i > pos + spread) {
      if ( dimm(i, now) ) res = true;
    }
  }
  // update inside spread
  for (int i = -spread; i <= spread; ++i) {
    int cpos = pos + i;
    float a = (float)(spread - abs(pos - cpos)) / (float)spread; // [0,1.0]
    if (a < 0.0) continue;
    if (cpos >= LED_NO) cpos = cpos - LED_NO;
    if (cpos < 0) cpos = cpos + LED_NO;
    if ( lighten(cpos, now, a) ) res = true;
  }
  return res;
}


bool pushToTargetHue() {
  bool res = false;
  for (uint8_t i = 0; i < LED_NO; ++i) {
    if ( g_led_hue[i] != g_nice_hue[g_t_hue] ) {
      if ((g_led_hue[i] < 10 && g_nice_hue[g_t_hue] == 255) || (g_nice_hue[g_t_hue] == 0 && g_led_hue[i] > 245 )) {
        g_led_hue[i] = g_nice_hue[g_t_hue];
      } else {
        g_led_hue[i] = average(g_nice_hue[g_t_hue], g_led_hue[i], 8);
      }
      res = true;
    }
  }
  return res;
}


RgbColor getColor(uint8_t i, uint8_t l) {
  uint16_t r, g, b;
  if (i < 64) { // redish
    r = 252;
    g = 252 - i * 4;
    b = g;
  }
  if (i >= 64 && i < 128) {
    r = 252 - (i - 64) * 4;
    g = (i - 64) * 4;
    b = 0;
  }
  if (i >= 128 && i < 192) {
    r = 0;
    g = 252 - (i - 128) * 4;
    b = (i - 128) * 4;
  }
  if (i >= 192) {
    r = (i - 192) * 4;
    g = r;
    b = 252;
  }
  // correct intensity
  RgbColor res;
  res.R = (uint8_t)(r * (uint16_t)l / 255);
  res.G = (uint8_t)(g * (uint16_t)l / 255);
  res.B = (uint8_t)(b * (uint16_t)l / 255);
  return res;
}


bool dimm(uint16_t index, unsigned long now) {
  if (now - g_led_t[index] < WAIT_BEFORE_FADE) {
    return false;
  }
  if ( g_led_l[index] > 0 ) {
    //--g_led_l[index];
    g_led_l[index] = average(0, g_led_l[index], 32);
    return true;
  }
  return false;
}


bool lighten(uint16_t index, unsigned long now) {
  if ( g_led_l[index] >= g_dist ) {
    return false;
  }
  g_led_l[index] = average( g_dist, g_led_l[index], 4);
  return true;
}


bool lighten(uint16_t index, unsigned long now, float coef) {
  uint8_t nl = (uint8_t)((float)g_dist * coef);
  if ( g_led_l[index] >= nl ) {
    return false;
  }
  g_led_l[index] = average( nl, g_led_l[index], 4 );
  return true;
}


void showColors() {
  for (uint8_t i = 0; i < LED_NO; ++i) {
    RgbColor c = getColor(g_led_hue[i], g_led_l[i]);
    strip.SetPixelColor(i, c);
  }
  strip.Show();
}


void monitorPiezo() {
  int val = analogRead(A0);
  if (val > 20) { // 10
    if (millis() - g_lock_time > 1000) { // disable state switching for a while
      g_locked = !g_locked;
      g_lock_time = millis();
      Serial.print("lock change ");
      Serial.print(g_locked);
      Serial.print(" ");
      Serial.print(val);
      Serial.print("\n");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  strip.Begin();
  strip.Show();

  pinMode(A0, INPUT);
  pinMode(D6, OUTPUT); // ADDR --> LOW 0x2A
  pinMode(D7, OUTPUT); // SD (shutdown)
  //pinMode(D8, INPUT); // INTB --> interrupt on change
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  Wire.begin();
  Configuration();

  unsigned long n = millis();
  for (int i = 0; i < LED_NO; ++i) {
    //strip.SetPixelColor(i, g_base_color);
    g_led_t[i] = n;
    g_led_hue[i] = 0;
    g_led_l[i] = 0;
  }

  for (int i = 0; i < 4; ++i) {
    cmesure[i] = readChannel(i);
    di[i] = 0;
  }
  //  SPIFFS.begin();
  //  {
  //    Dir dir = SPIFFS.openDir("/");
  //    while (dir.next()) {
  //      String fileName = dir.fileName();
  //      size_t fileSize = dir.fileSize();
  //      Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
  //    }
  //    Serial.printf("\n");
  //  }
  //
  //  WiFi.softAP(ssid, password);
  //  IPAddress apip = WiFi.softAPIP();
  //  Serial.println("xxxxxxxxxxxxxxxx");
  //  Serial.println(apip);
  //
  //  server.onNotFound([]() {
  //    if (!handleFileRead(server.uri()))
  //      server.send(404, "text/plain", "FileNotFound");
  //  });
  //  server.begin();
  //
  //  webSocket.onEvent(webSocketEvent);
  //  webSocket.begin();
}


void loop() {
  //  server.handleClient(); yield();
  //  webSocket.loop(); yield();
  monitorPiezo(); // yield();
  if (!g_new_data && !g_locked) {
    monitorCapa(); yield();

    if (updateLeds()) {
      showColors(); // strip.Show();
      g_new_data = false;
    }
  }
}
