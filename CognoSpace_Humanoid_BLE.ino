/*
  =====================================================================
   CognoSpace Humanoid Robot  -  BLE firmware  (pairs with index.html)
  =====================================================================
   Board   : ESP32 Dev Module, Arduino-ESP32 core 3.x
   Libs    : ESP32Servo, LiquidCrystal_I2C, Adafruit GFX, Adafruit SSD1306
   If the sketch is too big: Tools > Partition Scheme > "Huge APP (3MB No OTA)"

   The web page (index.html) connects with Web Bluetooth using the
   Nordic UART Service. Every command is one text line ending in '\n'.

   App -> Robot
     T|l1|l2|l3|l4     speech page for the LCD (shown while the app speaks it)
     TE                speech finished -> back to the Ready screen
     P|text            keep this text on the LCD (word wrapped)
     PC                clear the kept text
     A,<name>          action WITH LCD banner : wave clap dance greet nod
                       shake think grab give reset
     M,<name>          motion only (LCD untouched, used while speaking) :
                       all of the above + hello bye thanks yes no minidance
                       demo_head demo_r1 demo_r2 demo_l1 demo_l2 demo_eyes demo_buzz
     SV,<servo>,<pos>  head | lh1 | lh2 | rh1 | rh2 , 0..180
     D,<F|B|L|R|S>     drive (re-send every 200 ms, auto stop after 800 ms)
     E,<0..10>         OLED eye expression (0 = auto animate)
     MU,<1..5>         play a buzzer song
     GS,<1..3>         start LCD game (easy / medium / hard)
     J                 game jump          GQ   quit game
     X                 STOP everything    Q    ask for current state

   Robot -> App (notifications, max 20 bytes)
     S,<action>   E,<emoji>   G,<score>   GO,<score>
     V1,<head>,<lh1>,<lh2>    V2,<rh1>,<rh2>
  =====================================================================
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================
//  BLE (Nordic UART Service)
// =====================================================
#define BLE_NAME         "CognoSpace_Humanoid"
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // app writes here
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // robot notifies here

BLEServer         *bleServer = nullptr;
BLECharacteristic *txChar    = nullptr;
volatile bool deviceConnected = false;
volatile bool evConnected     = false;
volatile bool evDisconnected  = false;

// =====================================================
//  PIN CONFIG  (unchanged from the WiFi version)
// =====================================================
#define OLED_SDA_PIN  18
#define OLED_SCL_PIN  19
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDRESS 0x3C

TwoWire OLEDWire = TwoWire(1);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &OLEDWire, OLED_RESET);
LiquidCrystal_I2C lcd(0x27, 20, 4);

Servo headServo;
Servo leftHandServo1;
Servo leftHandServo2;
Servo rightHandServo1;
Servo rightHandServo2;

const int IN1 = 13;
const int IN2 = 14;
const int ENA = 27;
const int IN3 = 26;
const int IN4 = 4;
const int ENB = 23;

#define HEAD_PIN        32
#define LEFT_HAND1_PIN  33
#define LEFT_HAND2_PIN  25
#define RIGHT_HAND1_PIN 15
#define RIGHT_HAND2_PIN 12
#define BUZZER_PIN      16

const int freq              = 5000;
const int resolution        = 8;
const int defaultMotorSpeed = 200;

// =====================================================
//  STATE
// =====================================================
int headPos       = 90;
int leftHand1Pos  = 90;
int leftHand2Pos  = 180;
int rightHand1Pos = 90;
int rightHand2Pos = 180;

String driveState    = "STOPPED";
String currentAction = "Ready";
String lastSentAction = "";
int    lastSentEmoji  = -1;

bool   persistentDisplay = false;
String persistentLine[4] = {"", "", "", ""};
bool   speechOnLcd = false;          // LCD currently shows a speech page

// drive (written by the BLE task, applied in loop)
volatile char     driveCmd   = 'S';
volatile uint32_t driveStamp = 0;
char              appliedDrive = 'S';
const unsigned long motorTimeout = 800;

// OLED eye
volatile int emojiMode = 0;
unsigned long lastEyeTick     = 0;
unsigned long eyeTickInterval = 80;
int autoAnimStep = 0;

// command queue (BLE task -> loop) and motion queue (loop -> motion task)
#define CMD_LEN 128
struct MotionJob { char name[16]; bool lcd; };
QueueHandle_t cmdQueue    = nullptr;
QueueHandle_t motionQueue = nullptr;
volatile bool motionBusy      = false;
volatile bool abortMotion     = false;
volatile bool motionDoneFlag  = false;
volatile bool restoreLcdFlag  = false;
volatile bool keepPose        = false;

// game
volatile bool gameRunning   = false;
volatile bool jumpRequested = false;
bool gameOver     = false;
int  gameScore    = 0;
int  gameSpeed    = 80;

#define TERRAIN_WIDTH 19
static char terrainUpper[TERRAIN_WIDTH + 1];
static char terrainLower[TERRAIN_WIDTH + 1];

byte heroPos            = 1;
byte newTerrainType     = 0;
byte newTerrainDuration = 1;

unsigned long lastGameTick    = 0;
unsigned long gameOverTime    = 0;
bool          gameOverPending = false;

#define SPRITE_RUN1            1
#define SPRITE_RUN2            2
#define SPRITE_JUMP            3
#define SPRITE_JUMP_UPPER      '.'
#define SPRITE_JUMP_LOWER      4
#define SPRITE_TERRAIN_SOLID   5
#define SPRITE_TERRAIN_SOLID_R 6
#define SPRITE_TERRAIN_SOLID_L 7
#define SPRITE_TERRAIN_EMPTY   ' '

#define HERO_POSITION_RUN_LOWER_1  1
#define HERO_POSITION_RUN_LOWER_2  2
#define HERO_POSITION_JUMP_1       3
#define HERO_POSITION_JUMP_2       4
#define HERO_POSITION_JUMP_3       5
#define HERO_POSITION_JUMP_4       6
#define HERO_POSITION_JUMP_5       7
#define HERO_POSITION_JUMP_6       8
#define HERO_POSITION_JUMP_7       9
#define HERO_POSITION_JUMP_8      10

// =====================================================
//  BLE SEND / RECEIVE
// =====================================================
// Call only from loop().
void bleSend(const String &msg) {
  if (!deviceConnected || txChar == nullptr) return;
  String out = msg;
  if (out.length() > 20) out = out.substring(0, 20);
  txChar->setValue((uint8_t *)out.c_str(), out.length());
  txChar->notify();
}

// Runs in the Bluetooth task: keep it tiny, never touch LCD / servos here.
void routeLine(const char *line) {
  if (line[0] == 'D' && line[1] == ',') {            // drive: latest value wins
    driveCmd   = line[2];
    driveStamp = millis();
    return;
  }
  if (line[0] == 'J' && line[1] == 0) {              // game jump
    if (gameRunning) jumpRequested = true;
    return;
  }
  if (line[0] == 'X' && line[1] == 0) {              // emergency stop: act at once
    if (motionBusy) abortMotion = true;
    driveCmd = 'S';
  }
  char item[CMD_LEN];
  strncpy(item, line, CMD_LEN - 1);
  item[CMD_LEN - 1] = 0;
  if (cmdQueue != nullptr) xQueueSend(cmdQueue, item, 0);
}

class RxCallbacks : public BLECharacteristicCallbacks {
  char buf[CMD_LEN];
  int  len      = 0;
  bool overflow = false;

  void onWrite(BLECharacteristic *c) {
    String v = String(c->getValue().c_str());
    for (unsigned int i = 0; i < v.length(); i++) {
      char ch = v[i];
      if (ch == '\r') continue;
      if (ch == '\n') {
        buf[len] = 0;
        if (len > 0 && !overflow) routeLine(buf);
        len = 0;
        overflow = false;
      } else if (len < CMD_LEN - 1) {
        buf[len++] = ch;
      } else {
        overflow = true;
      }
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s)    { deviceConnected = true;  evConnected = true; }
  void onDisconnect(BLEServer *s) { deviceConnected = false; evDisconnected = true; }
};

void setupBLE() {
  BLEDevice::init(BLE_NAME);
  BLEDevice::setMTU(247);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *svc = bleServer->createService(NUS_SERVICE_UUID);

  txChar = svc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic *rxChar = svc->createCharacteristic(
      NUS_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCallbacks());

  svc->start();

  // Service UUID in the advertisement, full name in the scan response
  // (both together do not fit in one 31-byte packet).
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setCompleteServices(BLEUUID(NUS_SERVICE_UUID));
  BLEAdvertisementData scanData;
  scanData.setName(BLE_NAME);
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  BLEDevice::startAdvertising();
}

// =====================================================
//  OLED EYE DRAWING
// =====================================================
void drawEye(int offsetX, int offsetY, int pupil, int iris, String mood, int blinkH) {
  oled.clearDisplay();
  int cx = 64, cy = 32, maxR = 30;

  oled.fillCircle(cx, cy, maxR, SSD1306_WHITE);
  oled.drawCircle(cx, cy, maxR + 1, SSD1306_WHITE);

  int px = constrain(cx + offsetX, cx - 14, cx + 14);
  int py = constrain(cy + offsetY, cy - 14, cy + 14);

  oled.fillCircle(px, py, iris,  SSD1306_BLACK);
  oled.fillCircle(px, py, pupil, SSD1306_BLACK);

  oled.fillCircle(px - pupil / 2 - 1, py - pupil / 2 - 1, 3, SSD1306_WHITE);
  oled.fillCircle(px - pupil / 2,     py - pupil / 2,     1, SSD1306_WHITE);

  if (blinkH > 0) {
    oled.fillRect(cx - 35, cy - 35, 70, blinkH, SSD1306_BLACK);
    oled.fillRect(cx - 35, cy + 35 - blinkH, 70, blinkH, SSD1306_BLACK);
  }

  if (mood == "angry") {
    oled.fillTriangle(cx - 35, cy - 35, cx + 35, cy - 35, cx, cy + 8, SSD1306_BLACK);
    oled.drawLine(cx - 28, cy - 28, cx - 10, cy - 6, SSD1306_WHITE);
    oled.drawLine(cx + 28, cy - 28, cx + 10, cy - 6, SSD1306_WHITE);
  }
  else if (mood == "sad") {
    oled.fillTriangle(cx - 35, cy - 35, cx + 35, cy - 35, cx, cy - 5, SSD1306_BLACK);
  }
  else if (mood == "suspicious") {
    oled.fillRect(cx - 35, cy - 35, 70, 18, SSD1306_BLACK);
    oled.fillRect(cx - 35, cy + 14, 70, 24, SSD1306_BLACK);
    oled.drawLine(cx - 28, cy - 17, cx + 28, cy - 17, SSD1306_WHITE);
  }
  else if (mood == "happy") {
    oled.fillRect(cx - 35, cy + 13, 70, 25, SSD1306_BLACK);
    oled.drawLine(cx - 15, cy + 11, cx + 15, cy + 11, SSD1306_WHITE);
  }
  else if (mood == "sleep") {
    oled.fillRect(cx - 35, cy - 2, 70, 38, SSD1306_BLACK);
    for (int i = -18; i <= 18; i += 7)
      oled.drawLine(cx + i, cy, cx + i, cy + 5, SSD1306_WHITE);
  }
  else if (mood == "love") {
    int hx = px, hy = py - 2;
    oled.fillCircle(hx - 3, hy, 3, SSD1306_WHITE);
    oled.fillCircle(hx + 3, hy, 3, SSD1306_WHITE);
    oled.fillTriangle(hx - 6, hy + 1, hx + 6, hy + 1, hx, hy + 7, SSD1306_WHITE);
    oled.fillCircle(hx - 3, hy, 2, SSD1306_BLACK);
    oled.fillCircle(hx + 3, hy, 2, SSD1306_BLACK);
  }
  else if (mood == "wink") {
    oled.fillRect(cx - 35, cy - 35, 35, 70, SSD1306_BLACK);
    oled.drawLine(cx - 30, cy, cx - 2, cy, SSD1306_WHITE);
  }

  oled.display();
}

// Called only from loop(). Other tasks just change emojiMode.
void tickEye() {
  if (millis() - lastEyeTick < eyeTickInterval) return;
  lastEyeTick = millis();

  int mode = emojiMode;
  if (mode == 1)  { drawEye(0, -4, 10, 16, "happy",      0); return; }
  if (mode == 2)  { drawEye(0, 10,  8, 16, "angry",      0); return; }
  if (mode == 3)  { drawEye(0,  0,  5, 20, "surprised",  0); return; }
  if (mode == 4)  { drawEye(0,  5, 10, 16, "sad",        0); return; }
  if (mode == 5)  { drawEye(6,  0, 10, 16, "wink",       0); return; }
  if (mode == 6)  { drawEye(0,  0, 10, 14, "love",       0); return; }
  if (mode == 7)  { drawEye(12, -12, 10, 16, "neutral",  0); return; }
  if (mode == 8)  { drawEye(0,  0,  8, 16, "sleep",      0); return; }
  if (mode == 9)  {
    float a = (autoAnimStep % 36) * 10.0 * 3.14159 / 180.0;
    int dx = (int)(14 * cos(a));
    int dy = (int)(14 * sin(a));
    drawEye(dx, dy, 10, 16, "neutral", 0);
    autoAnimStep++;
    return;
  }
  if (mode == 10) { drawEye(16, 4,  8, 16, "suspicious", 0); return; }

  // Auto-animate idle
  int s = autoAnimStep;
  if (s >= 120) { autoAnimStep = 0; s = 0; }
  if      (s < 30)  { int tx = map(s, 0, 29, 0, -18);   drawEye(tx, 0, 10, 16, "neutral", 0); }
  else if (s < 36)  { int bh = (s < 33) ? (s - 30) * 12 : (35 - s) * 12; drawEye(-18, 0, 10, 16, "neutral", bh); }
  else if (s < 66)  { int tx = map(s, 36, 65, -18, 18); drawEye(tx, 0, 10, 16, "neutral", 0); }
  else if (s < 72)  { int bh = (s < 69) ? (s - 66) * 12 : (71 - s) * 12; drawEye(18, 0, 10, 16, "neutral", bh); }
  else if (s < 90)  { int tx = map(s, 72, 89, 18, 10); int ty = map(s, 72, 89, 0, -12); drawEye(tx, ty, 10, 16, "neutral", 0); }
  else              { int ps = 10 + (int)(2 * sin((s - 90) * 3.14159 / 15.0)); drawEye(0, 0, ps, 16, "neutral", 0); }
  autoAnimStep++;
}

// =====================================================
//  ABORTABLE DELAY + BUZZER
// =====================================================
// Used by the motion task. Returns false as soon as STOP is requested.
bool mdelay(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (abortMotion) return false;
    uint32_t left = ms - (millis() - t0);
    delay(left > 5 ? 5 : (left ? left : 1));
  }
  return !abortMotion;
}
#define MD(ms) do { if (!mdelay(ms)) return; } while (0)

bool buzzTone(int frequency, int duration) {
  if (frequency <= 0) return mdelay(duration);
  long period  = 1000000L / frequency;
  long total   = (long)duration * 1000L;
  long elapsed = 0;
  while (elapsed < total) {
    if (abortMotion) { digitalWrite(BUZZER_PIN, LOW); return false; }
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(period / 2);
    elapsed += period;
  }
  return true;
}
bool buzzBeep(int f, int d) { if (!buzzTone(f, d)) return false; return mdelay(30); }

#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_AS4 466
#define NOTE_DS4 311
#define NOTE_FS4 370
#define NOTE_GS4 415
#define NOTE_CS5 554
#define NOTE_DS5 622
#define NOTE_FS5 740
#define NOTE_GS5 831
#define NOTE_AS5 932

// =====================================================
//  SONGS
// =====================================================
void playNotes(const int *melody, const int *dur, int count) {
  for (int i = 0; i < count; i++) if (!buzzBeep(melody[i], dur[i])) return;
}
void playPacman() {
  static const int melody[] = { NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C5, NOTE_B4, NOTE_A4,
                                NOTE_G4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_F4,
                                NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_A4,
                                NOTE_G4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_B4 };
  static const int dur[] = { 200, 200, 200, 100, 200, 200, 300, 200, 200, 200, 100, 200, 200, 300,
                             150, 150, 150, 150, 300, 150, 150, 150, 150, 400 };
  playNotes(melody, dur, 24);
}
void playMario() {
  static const int melody[] = { NOTE_E5, NOTE_E5, 0, NOTE_E5, 0, NOTE_C5, NOTE_E5, 0,
                                NOTE_G5, 0, 0, NOTE_G4, 0, 0, NOTE_C5, 0, NOTE_G4, 0, NOTE_E4, 0,
                                NOTE_A4, 0, NOTE_B4, 0, NOTE_AS4, NOTE_A4, 0,
                                NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, 0, NOTE_F5, NOTE_G5,
                                0, NOTE_E5, 0, NOTE_C5, NOTE_D5, NOTE_B4 };
  static const int dur[] = { 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250,
                             150, 150, 150, 150, 150, 150, 125, 125, 125, 125, 125, 125, 125,
                             83, 83, 83, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125 };
  playNotes(melody, dur, 40);
}
void playGOT() {
  static const int melody[] = { NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4,
                                NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4,
                                NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_AS4, NOTE_A4,
                                NOTE_F4, NOTE_AS4, NOTE_A4, NOTE_G4 };
  static const int dur[] = { 400, 150, 175, 100, 600, 150, 175, 100, 600, 150,
                             175, 100, 300, 150, 600, 150, 175, 100, 800 };
  playNotes(melody, dur, 19);
}
void playSubway() {
  static const int melody[] = { NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_G5,
                                NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_C5, NOTE_E5,
                                NOTE_G5, NOTE_A5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_E5,
                                NOTE_G5, NOTE_E5, NOTE_C5, NOTE_D5, NOTE_E5 };
  static const int dur[] = { 150, 150, 150, 150, 150, 300, 150, 150, 150, 150, 150, 150,
                             150, 300, 150, 150, 150, 150, 300, 150, 150, 150, 400 };
  playNotes(melody, dur, 23);
}
void playTokyoDrift() {
  static const int melody[] = { NOTE_A4, NOTE_A4, NOTE_A4, NOTE_G4, NOTE_A4,
                                NOTE_A4, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_G4,
                                NOTE_G4, NOTE_G4, NOTE_F4, NOTE_G4,
                                NOTE_G4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_F4,
                                NOTE_A4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_A4 };
  static const int dur[] = { 150, 150, 150, 150, 300, 150, 150, 150, 150, 300,
                             150, 150, 150, 300, 150, 150, 150, 150, 300,
                             150, 150, 150, 150, 500 };
  playNotes(melody, dur, 24);
}

void buzzJump()     { buzzTone(800, 50);  buzzTone(1000, 50); }
void buzzGameOver() { buzzTone(400, 200); buzzTone(300, 200); buzzTone(200, 400); }
void buzzPoint()    { buzzTone(1200, 30); }

// =====================================================
//  LCD HELPERS   (LCD is only ever touched from loop())
// =====================================================
void lcdPrintCentered(int row, String s) {
  if ((int)s.length() > 20) s = s.substring(0, 20);
  int pad = (20 - s.length()) / 2;
  String out = "";
  for (int i = 0; i < pad; i++) out += " ";
  out += s;
  while ((int)out.length() < 20) out += " ";
  lcd.setCursor(0, row);
  lcd.print(out);
}
void showPersistentDisplay() {
  if (!persistentDisplay) return;
  for (int i = 0; i < 4; i++) lcdPrintCentered(i, persistentLine[i]);
}
void displayWrappedText(String text) {
  String lines[4] = {"", "", "", ""};
  int lineIdx = 0;
  String word = "";
  text.trim();
  for (int i = 0; i <= (int)text.length(); i++) {
    char c = (i < (int)text.length()) ? text.charAt(i) : ' ';
    if (c == ' ' || c == '\n') {
      if (word.length() == 0) continue;
      if (lineIdx >= 4) break;
      String candidate = (lines[lineIdx].length() == 0) ? word : lines[lineIdx] + " " + word;
      if ((int)candidate.length() <= 20) { lines[lineIdx] = candidate; }
      else { lineIdx++; if (lineIdx >= 4) break; lines[lineIdx] = word; }
      word = "";
    } else { word += c; }
  }
  persistentDisplay = true;
  for (int i = 0; i < 4; i++) persistentLine[i] = lines[i];
  showPersistentDisplay();
}
void showReadyScreen() {
  speechOnLcd = false;
  if (persistentDisplay) { showPersistentDisplay(); return; }
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcdPrintCentered(1, "CognoSpace Robot");
  lcdPrintCentered(2, "Status: READY");
  lcd.setCursor(0, 3); lcd.print("====================");
}
void showWaitingScreen() {
  lcd.clear();
  lcdPrintCentered(0, "CognoSpace Robot");
  lcdPrintCentered(1, "Bluetooth: waiting");
  lcdPrintCentered(2, BLE_NAME);
  lcdPrintCentered(3, "Open app > Connect");
}
void updateActionDisplay(String title, String line2, String line3) {
  persistentDisplay = false;
  lcd.clear();
  lcdPrintCentered(0, title);
  lcd.setCursor(0, 1); lcd.print("====================");
  lcdPrintCentered(2, line2);
  lcdPrintCentered(3, line3);
}
// "line1|line2|line3|line4" sent by the app at the instant it starts speaking.
void showSpeechPage(const char *payload) {
  String lines[4];
  int n = 0;
  String cur = "";
  for (const char *p = payload; ; p++) {
    if (*p == '|' || *p == 0) {
      if (n < 4) lines[n++] = cur;
      cur = "";
      if (*p == 0) break;
    } else cur += *p;
  }
  while (n > 0 && lines[n - 1].length() == 0) n--;
  int start = (4 - n) / 2;
  for (int r = 0; r < 4; r++) {
    int idx = r - start;
    lcdPrintCentered(r, (idx >= 0 && idx < n) ? lines[idx] : String(""));
  }
  speechOnLcd = true;
}

// =====================================================
//  MOTOR / SERVO HELPERS
// =====================================================
void applyMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  ledcWrite(ENA, abs(leftSpeed)); }
  else if (leftSpeed < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); ledcWrite(ENA, abs(leftSpeed)); }
  else                    { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  ledcWrite(ENA, 0); }
  if (rightSpeed > 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  ledcWrite(ENB, abs(rightSpeed)); }
  else if (rightSpeed < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); ledcWrite(ENB, abs(rightSpeed)); }
  else                     { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  ledcWrite(ENB, 0); }
}
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
}
void resetAllServos() {
  headServo.write(90); leftHandServo1.write(90); leftHandServo2.write(180);
  rightHandServo1.write(90); rightHandServo2.write(180);
  headPos = 90; leftHand1Pos = 90; leftHand2Pos = 180;
  rightHand1Pos = 90; rightHand2Pos = 180;
}
bool sweepServo(Servo &s, int from, int to, int sd) {
  if (from < to) { for (int p = from; p <= to; p += 3) { s.write(p); if (!mdelay(sd)) return false; } }
  else           { for (int p = from; p >= to; p -= 3) { s.write(p); if (!mdelay(sd)) return false; } }
  return true;
}

// =====================================================
//  MOTIONS  (run in the motion task, never touch the LCD)
// =====================================================
void nodHead() {
  for (int i = 0; i < 2; i++) { headServo.write(70); MD(300); headServo.write(110); MD(300); }
  headServo.write(90);
}
void shakeHead() {
  for (int i = 0; i < 2; i++) { headServo.write(60); MD(350); headServo.write(120); MD(350); }
  headServo.write(90);
}
void quickWave() {
  rightHandServo1.write(90);
  for (int i = 0; i < 3; i++) { rightHandServo2.write(135); MD(250); rightHandServo2.write(180); MD(250); }
}
void slowWave() {
  rightHandServo1.write(90);
  for (int i = 0; i < 4; i++) { rightHandServo2.write(135); MD(400); rightHandServo2.write(180); MD(400); }
}
void miniDance() {
  emojiMode = 1;
  for (int i = 0; i < 4; i++) {
    leftHandServo1.write(90); leftHandServo2.write(0);
    rightHandServo1.write(90); rightHandServo2.write(180);
    headServo.write(60); applyMotorSpeeds(150, -150); MD(400);
    leftHandServo1.write(90); leftHandServo2.write(180);
    rightHandServo1.write(90); rightHandServo2.write(0);
    headServo.write(120); applyMotorSpeeds(-150, 150); MD(400);
  }
}
void mGreet() {
  emojiMode = 1;
  MD(800);
  for (int step = 0; step <= 90; step += 3) {
    rightHandServo1.write(90 - step);
    leftHandServo1.write(90 + step);
    MD(15);
  }
  MD(500);
  for (int i = 0; i < 4; i++) {
    rightHandServo2.write(135); leftHandServo2.write(135); MD(300);
    rightHandServo2.write(180); leftHandServo2.write(180); MD(300);
  }
  for (int step = 0; step <= 90; step += 3) {
    rightHandServo1.write(step);
    leftHandServo1.write(180 - step);
    MD(15);
  }
}
void mClap() {
  emojiMode = 1;
  rightHandServo2.write(135); leftHandServo2.write(135);
  MD(500);
  for (int step = 0; step <= 90; step += 3) {
    rightHandServo1.write(90 - step);
    leftHandServo1.write(90 + step);
    MD(15);
  }
  MD(200);
  for (int i = 0; i < 8; i++) {
    rightHandServo2.write(180); leftHandServo2.write(180);
    if (!buzzTone(800, 80)) return;
    MD(120);
    rightHandServo2.write(135); leftHandServo2.write(135);
    MD(120);
  }
}
void mWave() {
  emojiMode = 1;
  rightHandServo1.write(90);
  for (int i = 0; i < 6; i++) {
    rightHandServo2.write(135); MD(400);
    rightHandServo2.write(180); MD(400);
  }
}
void mDance() {
  for (int i = 0; i < 5; i++) {
    leftHandServo1.write(90); leftHandServo2.write(0);
    rightHandServo1.write(90); rightHandServo2.write(180);
    headServo.write(45); emojiMode = 1;
    applyMotorSpeeds(180, -180); MD(600);
    leftHandServo1.write(90); leftHandServo2.write(180);
    rightHandServo1.write(90); rightHandServo2.write(0);
    headServo.write(135); emojiMode = 9;
    applyMotorSpeeds(-180, 180); MD(600);
    leftHandServo1.write(90); leftHandServo2.write(90);
    rightHandServo1.write(90); rightHandServo2.write(90);
    headServo.write(90);
    applyMotorSpeeds(200, 200); MD(400);
    applyMotorSpeeds(-200, -200); MD(400);
  }
}
void mNod() {
  emojiMode = 1;
  for (int i = 0; i < 5; i++) { headServo.write(70); MD(400); headServo.write(110); MD(400); }
  headServo.write(90);
}
void mShake() {
  emojiMode = 2;
  for (int i = 0; i < 5; i++) { headServo.write(60); MD(400); headServo.write(120); MD(400); }
  headServo.write(90);
}
void mThink() {
  emojiMode = 7;
  headServo.write(70); MD(500);
  rightHandServo1.write(45); rightHandServo2.write(45); MD(2000);
  headServo.write(110); MD(500); headServo.write(90); MD(500);
}
void mGrab() {
  emojiMode = 10;
  rightHandServo1.write(180); rightHand1Pos = 180;
  keepPose = true;
  MD(1000);
}
void mGive() {
  emojiMode = 1;
  keepPose = true;
  for (int pos = 180; pos >= 0; pos -= 5) { rightHandServo1.write(pos); MD(30); }
  rightHand1Pos = 0;
  MD(1000);
}
void mHello()  { emojiMode = 1; quickWave(); if (abortMotion) return; rightHandServo2.write(180); nodHead(); }
void mThanks() { emojiMode = 1; nodHead(); if (abortMotion) return; MD(300); nodHead(); }

// Component demos used by the "tell me about your parts" tour
void demoHead() { headServo.write(0); MD(600); headServo.write(180); MD(600); headServo.write(90); }
void demoR1()   { if (!sweepServo(rightHandServo1, 0, 180, 12)) return; sweepServo(rightHandServo1, 180, 90, 12); }
void demoR2()   { sweepServo(rightHandServo2, 0, 180, 12); }
void demoL1()   { if (!sweepServo(leftHandServo1, 0, 180, 12)) return; sweepServo(leftHandServo1, 180, 90, 12); }
void demoL2()   { sweepServo(leftHandServo2, 0, 180, 12); }
void demoEyes() { emojiMode = 3; MD(1500); emojiMode = 1; MD(1500); emojiMode = 8; MD(1500); }
void demoBuzz() {
  if (!buzzBeep(NOTE_C5, 200)) return;
  if (!buzzBeep(NOTE_E5, 200)) return;
  if (!buzzBeep(NOTE_G5, 200)) return;
  buzzBeep(NOTE_C5, 400);
}
void mTokyo() {
  emojiMode = 9;
  applyMotorSpeeds(150, -150); MD(400);
  applyMotorSpeeds(-150, 150); MD(400);
  stopMotors();
  playTokyoDrift();
}

void runMotion(const char *n) {
  if      (!strcmp(n, "wave"))      mWave();
  else if (!strcmp(n, "clap"))      mClap();
  else if (!strcmp(n, "dance"))     mDance();
  else if (!strcmp(n, "greet"))     mGreet();
  else if (!strcmp(n, "nod"))       mNod();
  else if (!strcmp(n, "shake"))     mShake();
  else if (!strcmp(n, "think"))     mThink();
  else if (!strcmp(n, "grab"))      mGrab();
  else if (!strcmp(n, "give"))      mGive();
  else if (!strcmp(n, "reset"))     { mdelay(800); }
  else if (!strcmp(n, "hello"))     mHello();
  else if (!strcmp(n, "bye"))       { emojiMode = 1; slowWave(); }
  else if (!strcmp(n, "thanks"))    mThanks();
  else if (!strcmp(n, "yes"))       { emojiMode = 1; nodHead(); }
  else if (!strcmp(n, "no"))        shakeHead();
  else if (!strcmp(n, "minidance")) miniDance();
  else if (!strcmp(n, "demo_head")) demoHead();
  else if (!strcmp(n, "demo_r1"))   demoR1();
  else if (!strcmp(n, "demo_r2"))   demoR2();
  else if (!strcmp(n, "demo_l1"))   demoL1();
  else if (!strcmp(n, "demo_l2"))   demoL2();
  else if (!strcmp(n, "demo_eyes")) demoEyes();
  else if (!strcmp(n, "demo_buzz")) demoBuzz();
  else if (!strcmp(n, "song1"))     { emojiMode = 1;  playPacman(); }
  else if (!strcmp(n, "song2"))     { emojiMode = 9;  playMario(); }
  else if (!strcmp(n, "song3"))     { emojiMode = 10; playGOT(); }
  else if (!strcmp(n, "song4"))     { emojiMode = 3;  playSubway(); }
  else if (!strcmp(n, "song5"))     mTokyo();
}

// Runs gestures, dances and songs so the LCD / Bluetooth stay responsive.
void motionTask(void *arg) {
  MotionJob job;
  for (;;) {
    if (xQueueReceive(motionQueue, &job, portMAX_DELAY) != pdTRUE) continue;
    motionBusy = true;
    keepPose   = false;
    runMotion(job.name);

    bool aborted = abortMotion;
    stopMotors();
    if (!keepPose || aborted) resetAllServos();
    emojiMode = 0;
    if (aborted) xQueueReset(motionQueue);
    if (uxQueueMessagesWaiting(motionQueue) == 0) {
      if (job.lcd) restoreLcdFlag = true;
      motionDoneFlag = true;
      abortMotion    = false;
      motionBusy     = false;
    }
  }
}

// Stop whatever the motion task is doing and wait until it is idle.
void abortAndWait() {
  if (motionBusy) {
    abortMotion = true;
    uint32_t t0 = millis();
    while (motionBusy && millis() - t0 < 2500) delay(5);
  }
  motionDoneFlag = false;
  restoreLcdFlag = false;
}

// =====================================================
//  ACTION START (loop side: LCD banner + hand over to motion task)
// =====================================================
String actionLabel(const String &n) {
  if (n == "wave")  return "Waving";
  if (n == "clap")  return "Clapping";
  if (n == "dance" || n == "minidance") return "Dancing";
  if (n == "greet" || n == "hello")     return "Greeting";
  if (n == "nod"   || n == "yes" || n == "thanks") return "Nodding";
  if (n == "shake" || n == "no")        return "Shaking";
  if (n == "think") return "Thinking";
  if (n == "grab")  return "Grabbing";
  if (n == "give")  return "Giving";
  if (n == "reset") return "Resetting";
  if (n == "bye")   return "Waving";
  if (n.startsWith("song")) return "Playing Music";
  if (n.startsWith("demo")) return "Explaining";
  return "Moving";
}

void showActionBanner(const String &n) {
  persistentDisplay = false;
  speechOnLcd = false;
  if (n == "wave") {
    lcd.clear();
    lcdPrintCentered(0, "Welcome to");
    lcdPrintCentered(1, "CognoSpace");
    lcdPrintCentered(2, "Skill Composite");
    lcdPrintCentered(3, "Lab!");
  }
  else if (n == "clap")  updateActionDisplay("CLAP ACTION", "Clapping for you!", "Great job!");
  else if (n == "dance") updateActionDisplay("DANCE MODE", "Watch me groove!", "CognoSpace!");
  else if (n == "greet") updateActionDisplay("GREETING", "Welcome to", "CognoSpace!");
  else if (n == "nod")   updateActionDisplay("NODDING YES", "Yes, I agree!", "Understood!");
  else if (n == "shake") updateActionDisplay("SHAKING NO", "No, I disagree", "Not right!");
  else if (n == "think") updateActionDisplay("THINKING", "Hmm... Let me", "think...");
  else if (n == "grab")  updateActionDisplay("GRAB ACTION", "Grabbing object", "Hold tight!");
  else if (n == "give")  updateActionDisplay("GIVE ACTION", "Releasing object", "Here you go!");
  else if (n == "reset") updateActionDisplay("RESET ALL", "Resetting...", "Please wait");
}

void showSongBanner(int song) {
  persistentDisplay = false;
  speechOnLcd = false;
  lcd.clear();
  lcdPrintCentered(0, "Now Playing:");
  switch (song) {
    case 1: lcdPrintCentered(1, "Pacman");          lcdPrintCentered(2, "Airtel Tone");      lcdPrintCentered(3, "Enjoy the beat!");  break;
    case 2: lcdPrintCentered(1, "Mario Theme");     lcdPrintCentered(2, "Docomo Tone");      lcdPrintCentered(3, "Its a me, Mario!"); break;
    case 3: lcdPrintCentered(1, "Game of Thrones"); lcdPrintCentered(2, "Winter is Coming"); lcdPrintCentered(3, "Brace yourself!");  break;
    case 4: lcdPrintCentered(1, "Subway Surfers");  lcdPrintCentered(2, "Keep Running!");    lcdPrintCentered(3, "Dodge the train!"); break;
    case 5: lcdPrintCentered(1, "Tokyo Drift");     lcdPrintCentered(2, "Fast & Furious");   lcdPrintCentered(3, "Drift time!");      break;
  }
}

void startMotion(const String &name, bool withLcd) {
  if (gameRunning || gameOver) return;
  MotionJob job;
  name.toCharArray(job.name, sizeof(job.name));
  job.lcd = withLcd;

  stopMotors();
  appliedDrive = 'S';
  driveCmd     = 'S';
  driveState   = "STOPPED";

  motionBusy    = true;
  currentAction = actionLabel(name);
  if (xQueueSend(motionQueue, &job, 0) != pdTRUE) {
    Serial.println("Motion queue full, dropped: " + name);
  }
}

// =====================================================
//  GAME  (runs in loop, same as the WiFi version)
// =====================================================
void initGameGraphics() {
  static byte graphics[] = {
    B01100, B01100, B00000, B01110, B11100, B01100, B11010, B10011,
    B01100, B01100, B00000, B01100, B01100, B01100, B01100, B01110,
    B01100, B01100, B00000, B11110, B01101, B11111, B10000, B00000,
    B11110, B01101, B11111, B10000, B00000, B00000, B00000, B00000,
    B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111,
    B00011, B00011, B00011, B00011, B00011, B00011, B00011, B00011,
    B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000,
  };
  for (int i = 0; i < 7; ++i) lcd.createChar(i + 1, &graphics[i * 8]);
  for (int i = 0; i < TERRAIN_WIDTH; i++) {
    terrainUpper[i] = SPRITE_TERRAIN_EMPTY;
    terrainLower[i] = SPRITE_TERRAIN_EMPTY;
  }
  terrainUpper[TERRAIN_WIDTH] = '\0';
  terrainLower[TERRAIN_WIDTH] = '\0';
}
void advanceTerrain(char *terrain, char newTile) {
  for (int i = 0; i < TERRAIN_WIDTH - 1; i++) terrain[i] = terrain[i + 1];
  terrain[TERRAIN_WIDTH - 1] = newTile;
}
bool checkCollision() {
  if (heroPos >= HERO_POSITION_RUN_LOWER_1 && heroPos <= HERO_POSITION_RUN_LOWER_2) {
    if (terrainLower[1] == SPRITE_TERRAIN_SOLID ||
        terrainLower[1] == SPRITE_TERRAIN_SOLID_R ||
        terrainLower[1] == SPRITE_TERRAIN_SOLID_L) return true;
  }
  return false;
}
void drawGame() {
  String scoreStr = "S:" + String(gameScore);
  while ((int)scoreStr.length() < 6) scoreStr = " " + scoreStr;
  scoreStr = scoreStr.substring(scoreStr.length() - 6);
  lcd.setCursor(0, 0);
  for (int i = 0; i < 14; i++) {
    char c = terrainUpper[i];
    if (c == SPRITE_TERRAIN_SOLID) lcd.write(SPRITE_TERRAIN_SOLID);
    else lcd.print(' ');
  }
  lcd.print(scoreStr);
  lcd.setCursor(0, 1);
  if (heroPos >= HERO_POSITION_JUMP_1 && heroPos <= HERO_POSITION_JUMP_8) {
    for (int i = 0; i < TERRAIN_WIDTH; i++) {
      if (i == 1) { if (heroPos <= HERO_POSITION_JUMP_4) lcd.write(SPRITE_JUMP); else lcd.write(SPRITE_JUMP_UPPER); }
      else { char c = terrainUpper[i]; if (c == SPRITE_TERRAIN_SOLID) lcd.write(SPRITE_TERRAIN_SOLID); else lcd.print(' '); }
    }
    lcd.setCursor(0, 2);
    for (int i = 0; i < TERRAIN_WIDTH; i++) {
      if (i == 1) { if (heroPos >= HERO_POSITION_JUMP_5) lcd.write(SPRITE_JUMP_LOWER); else lcd.print(' '); }
      else { char c = terrainLower[i]; if (c == SPRITE_TERRAIN_SOLID) lcd.write(SPRITE_TERRAIN_SOLID); else lcd.print(' '); }
    }
  } else {
    for (int i = 0; i < TERRAIN_WIDTH; i++) {
      char c = terrainUpper[i]; if (c == SPRITE_TERRAIN_SOLID) lcd.write(SPRITE_TERRAIN_SOLID); else lcd.print(' ');
    }
    lcd.setCursor(0, 2);
    for (int i = 0; i < TERRAIN_WIDTH; i++) {
      if (i == 1) lcd.write((heroPos == HERO_POSITION_RUN_LOWER_1) ? SPRITE_RUN1 : SPRITE_RUN2);
      else { char c = terrainLower[i]; if (c == SPRITE_TERRAIN_SOLID) lcd.write(SPRITE_TERRAIN_SOLID); else lcd.print(' '); }
    }
  }
  lcd.setCursor(0, 3); lcd.print("====================");
}
void gameTick() {
  char newLower = (newTerrainType == 1) ? SPRITE_TERRAIN_SOLID : SPRITE_TERRAIN_EMPTY;
  advanceTerrain(terrainLower, newLower);
  advanceTerrain(terrainUpper, SPRITE_TERRAIN_EMPTY);
  if (--newTerrainDuration == 0) {
    if (newTerrainType == 0) { newTerrainType = 1; newTerrainDuration = random(2, 5); }
    else                     { newTerrainType = 0; newTerrainDuration = random(6, 16); }
  }
  if (jumpRequested && (heroPos == HERO_POSITION_RUN_LOWER_1 || heroPos == HERO_POSITION_RUN_LOWER_2)) {
    heroPos = HERO_POSITION_JUMP_1; jumpRequested = false; buzzJump(); emojiMode = 3;
  }
  if      (heroPos >= HERO_POSITION_JUMP_1 && heroPos < HERO_POSITION_JUMP_8) heroPos++;
  else if (heroPos == HERO_POSITION_JUMP_8) { heroPos = HERO_POSITION_RUN_LOWER_1; emojiMode = 0; }
  else heroPos = (heroPos == HERO_POSITION_RUN_LOWER_1) ? HERO_POSITION_RUN_LOWER_2 : HERO_POSITION_RUN_LOWER_1;
  gameScore++;
  if (gameScore % 10 == 0) buzzPoint();
  if (gameScore % 5 == 0)  bleSend("G," + String(gameScore));
  drawGame();
  if (checkCollision()) {
    gameRunning = false; gameOver = true; gameOverTime = millis();
    emojiMode = 2; buzzGameOver();
    bleSend("GO," + String(gameScore));
    lcd.clear();
    lcdPrintCentered(0, "*** GAME OVER ***");
    lcdPrintCentered(1, "====================");
    lcdPrintCentered(2, "Score: " + String(gameScore));
    lcdPrintCentered(3, "Resetting in 7s...");
  }
}
void startGame(int difficulty) {
  gameOver = false; gameOverPending = false; gameScore = 0; jumpRequested = false;
  heroPos = HERO_POSITION_RUN_LOWER_1; newTerrainType = 0; newTerrainDuration = 5;
  if      (difficulty == 1) gameSpeed = 100;
  else if (difficulty == 2) gameSpeed = 70;
  else                      gameSpeed = 45;
  initGameGraphics();
  currentAction = "Game Running"; persistentDisplay = false; speechOnLcd = false; emojiMode = 0;
  bleSend("S,Game Running"); lastSentAction = currentAction;
  lcd.clear(); lcdPrintCentered(0, "GET READY!");
  lcdPrintCentered(1, "3..."); buzzTone(600, 150); delay(700);
  lcdPrintCentered(1, "2..."); buzzTone(700, 150); delay(700);
  lcdPrintCentered(1, "1..."); buzzTone(800, 150); delay(700);
  lcdPrintCentered(1, "GO!"); emojiMode = 1; buzzTone(1000, 200); delay(500);
  lcd.clear();
  gameRunning = true;
  lastGameTick = millis();
}
void endGame() {
  gameRunning = false; gameOver = false; gameOverPending = false; jumpRequested = false;
  currentAction = "Ready"; emojiMode = 0;
  persistentDisplay = false;
}

// =====================================================
//  COMMAND HANDLER  (loop side)
// =====================================================
void sendState() {
  bleSend("S," + currentAction);
  bleSend("V1," + String(headPos) + "," + String(leftHand1Pos) + "," + String(leftHand2Pos));
  bleSend("V2," + String(rightHand1Pos) + "," + String(rightHand2Pos));
  bleSend("E," + String((int)emojiMode));
  lastSentAction = currentAction;
  lastSentEmoji  = emojiMode;
}

void stopEverything() {
  abortAndWait();
  stopMotors();
  appliedDrive = 'S'; driveCmd = 'S'; driveState = "STOPPED";
  if (gameRunning || gameOver) endGame();
  emojiMode = 0;
  currentAction = "Ready";
  showReadyScreen();
}

void handleCommand(char *line) {
  // ---- speech pages ----
  if (line[0] == 'T' && line[1] == '|') {
    if (!gameRunning && !gameOver) showSpeechPage(line + 2);
    return;
  }
  if (!strcmp(line, "TE")) {
    if (speechOnLcd && !gameRunning && !gameOver) showReadyScreen();
    return;
  }
  if (line[0] == 'P' && line[1] == '|') {
    if (!gameRunning && !gameOver) { speechOnLcd = false; displayWrappedText(String(line + 2)); }
    return;
  }
  if (!strcmp(line, "PC")) {
    persistentDisplay = false;
    if (!gameRunning && !gameOver) showReadyScreen();
    return;
  }
  if (!strcmp(line, "X")) { stopEverything(); return; }
  if (!strcmp(line, "Q")) { sendState(); return; }
  if (!strcmp(line, "GQ")) {
    if (gameRunning || gameOver) { endGame(); resetAllServos(); stopMotors(); showReadyScreen(); }
    return;
  }

  String s(line);

  if (s.startsWith("A,")) {                       // action with LCD banner
    String name = s.substring(2);
    if (gameRunning || gameOver) return;
    abortAndWait();                               // a new button press replaces the running action
    if (name == "reset") emojiMode = 0;
    showActionBanner(name);
    startMotion(name, true);
    return;
  }
  if (s.startsWith("M,")) {                       // motion only (while speaking)
    startMotion(s.substring(2), false);
    return;
  }
  if (s.startsWith("SV,")) {
    int c2 = s.indexOf(',', 3);
    if (c2 < 0) return;
    String sn = s.substring(3, c2);
    int pos = constrain(s.substring(c2 + 1).toInt(), 0, 180);
    if      (sn == "head") { headPos = pos;       headServo.write(pos); }
    else if (sn == "lh1")  { leftHand1Pos = pos;  leftHandServo1.write(pos); }
    else if (sn == "lh2")  { leftHand2Pos = pos;  leftHandServo2.write(pos); }
    else if (sn == "rh1")  { rightHand1Pos = pos; rightHandServo1.write(pos); }
    else if (sn == "rh2")  { rightHand2Pos = pos; rightHandServo2.write(pos); }
    return;
  }
  if (s.startsWith("E,")) {
    emojiMode = constrain(s.substring(2).toInt(), 0, 10);
    autoAnimStep = 0;
    return;
  }
  if (s.startsWith("MU,")) {
    int song = s.substring(3).toInt();
    if (song < 1 || song > 5 || gameRunning || gameOver) return;
    abortAndWait();
    showSongBanner(song);
    startMotion("song" + String(song), true);
    return;
  }
  if (s.startsWith("GS,")) {
    int diff = s.substring(3).toInt();
    if (diff < 1 || diff > 3) diff = 2;
    abortAndWait();
    stopMotors(); appliedDrive = 'S'; driveCmd = 'S'; driveState = "STOPPED";
    startGame(diff);
    return;
  }
}

// =====================================================
//  LOOP SERVICES
// =====================================================
void serviceDrive() {
  if (motionBusy || gameRunning || gameOver) return;   // motion task owns the motors
  char c = driveCmd;
  if (millis() - driveStamp > motorTimeout) c = 'S';   // lost link / finger lifted
  if (c == appliedDrive) return;
  appliedDrive = c;
  switch (c) {
    case 'F': applyMotorSpeeds(defaultMotorSpeed, defaultMotorSpeed);   driveState = "FORWARD";  break;
    case 'B': applyMotorSpeeds(-defaultMotorSpeed, -defaultMotorSpeed); driveState = "BACKWARD"; break;
    case 'L': applyMotorSpeeds(-defaultMotorSpeed, defaultMotorSpeed);  driveState = "LEFT";     break;
    case 'R': applyMotorSpeeds(defaultMotorSpeed, -defaultMotorSpeed);  driveState = "RIGHT";    break;
    default:  stopMotors(); appliedDrive = 'S';                         driveState = "STOPPED";  break;
  }
}

void serviceConnection() {
  if (evConnected) {
    evConnected = false;
    Serial.println("BLE connected");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("====================");
    lcdPrintCentered(1, "Bluetooth");
    lcdPrintCentered(2, "Connected!");
    lcd.setCursor(0, 3); lcd.print("====================");
    emojiMode = 1;
    buzzTone(1000, 70); buzzTone(1400, 90);
    delay(1200);
    emojiMode = 0;
    persistentDisplay = false;
    currentAction = "Ready";
    lastSentAction = "";
    lastSentEmoji  = -1;
    showReadyScreen();
  }
  if (evDisconnected) {
    evDisconnected = false;
    Serial.println("BLE disconnected");
    abortAndWait();
    stopMotors();
    appliedDrive = 'S'; driveCmd = 'S'; driveState = "STOPPED";
    if (gameRunning || gameOver) endGame();
    resetAllServos();
    emojiMode = 0;
    persistentDisplay = false;
    speechOnLcd = false;
    currentAction = "Ready";
    showWaitingScreen();
    delay(300);
    BLEDevice::startAdvertising();
  }
}

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcdPrintCentered(1, "CognoSpace");
  lcdPrintCentered(2, "Humanoid Robot");
  lcd.setCursor(0, 3); lcd.print("====================");

  OLEDWire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED init failed!");
  } else {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(20, 28);
    oled.print("CognoSpace!");
    oled.display();
    Serial.println("OLED OK");
  }

  headServo.attach(HEAD_PIN);
  leftHandServo1.attach(LEFT_HAND1_PIN);
  leftHandServo2.attach(LEFT_HAND2_PIN);
  rightHandServo1.attach(RIGHT_HAND1_PIN);
  rightHandServo2.attach(RIGHT_HAND2_PIN);
  resetAllServos();

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);
  stopMotors();

  cmdQueue    = xQueueCreate(16, CMD_LEN);
  motionQueue = xQueueCreate(4, sizeof(MotionJob));
  // Priority 2 on core 1: buzzer timing is not disturbed by loop().
  xTaskCreatePinnedToCore(motionTask, "motion", 6144, nullptr, 2, nullptr, 1);

  delay(2500);                       // splash
  setupBLE();
  Serial.println("BLE advertising as " BLE_NAME);

  showWaitingScreen();
  drawEye(0, 0, 10, 16, "neutral", 0);
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  serviceConnection();

  char line[CMD_LEN];
  while (xQueueReceive(cmdQueue, line, 0) == pdTRUE) handleCommand(line);

  if (!motionBusy && abortMotion) abortMotion = false;

  if (motionDoneFlag) {
    motionDoneFlag = false;
    if (!gameRunning && !gameOver) {
      currentAction = "Ready";
      if (restoreLcdFlag && !speechOnLcd && deviceConnected) showReadyScreen();
    }
    restoreLcdFlag = false;
  }

  serviceDrive();

  if (gameRunning && !gameOver) {
    if (millis() - lastGameTick >= (unsigned long)gameSpeed) {
      lastGameTick = millis();
      gameTick();
    }
  }
  if (gameOver && !gameOverPending) {
    gameOverPending = true;
    gameOverTime = millis();
  }
  if (gameOver && gameOverPending && (millis() - gameOverTime >= 7000)) {
    endGame();
    resetAllServos(); stopMotors();
    showReadyScreen();
  }

  tickEye();

  if (currentAction != lastSentAction) {
    lastSentAction = currentAction;
    bleSend("S," + currentAction);
  }
  if ((int)emojiMode != lastSentEmoji) {
    lastSentEmoji = emojiMode;
    bleSend("E," + String(lastSentEmoji));
  }

  delay(2);
}
