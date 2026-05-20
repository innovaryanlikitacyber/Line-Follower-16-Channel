
#include <Arduino.h>

#define DEBUG_SENSOR 1   // 1 = tampilkan sensor, 0 = mati

// ===================================================
// KONFIGURASI PIN MUX
// ===================================================
const int s0Pin = 4;
const int s1Pin = 2;
const int s2Pin = 18;
const int s3Pin = 5;
const int analogZPin = 32;

// ===================================================
// PIN PUSH BUTTON
// ===================================================
#define BTN_START 35   // tekan sekali = jalan
#define BTN_STOP  19   // tekan sekali = berhenti

bool robotRun = false;
int lastStartState = HIGH;
int lastStopState  = HIGH;
unsigned long lastDebounce = 0;
const int debounceDelay = 40;

// ===================================================
// TB6612 MOTOR DRIVER
// ===================================================
#include <SparkFun_TB6612.h>

#define PWMA 33
#define PWMB 13
#define STBY 27
#define AIN1 26
#define AIN2 25
#define BIN1 14
#define BIN2 12

Motor motorL(AIN1, AIN2, PWMA, 1, STBY); // KIRI
Motor motorR(BIN1, BIN2, PWMB, 1, STBY); // KANAN

// ===================================================
int sensor[16];
int THRESHOLD = 45;

// ===================================================
// PID
// ===================================================
float Kp = 4.5;
float Ki = 0.001;
float Kd = 30;

float error, lastError, integral;
int baseSpeed = 255;
float I_MAX = 200;

// ===================================================
// AUTO LINE COLOR
// ===================================================
bool lineIsDark = true;   // true = garis hitam, false = garis putih

// ===================================================
void motorStop() {
  motorL.drive(0);
  motorR.drive(0);
}

// ===================================================
// START / STOP HANDLER (IDENTIK PROGRAM AWAL)
// ===================================================
void updateButtons() {
  int startRead = digitalRead(BTN_START);
  int stopRead  = digitalRead(BTN_STOP);

  if (millis() - lastDebounce > debounceDelay) {

    if (startRead == LOW && lastStartState == HIGH)
      robotRun = true;
    lastStartState = startRead;

    if (stopRead == LOW && lastStopState == HIGH) {
      robotRun = false;
      motorStop();
    }
    lastStopState = stopRead;

    lastDebounce = millis();
  }
}

// ===================================================
void selectChannel(int ch) {
  digitalWrite(s0Pin, ch & 1);
  digitalWrite(s1Pin, ch & 2);
  digitalWrite(s2Pin, ch & 4);
  digitalWrite(s3Pin, ch & 8);
}

// ===================================================
int readFiltered() {
  int a = analogRead(analogZPin);
  int b = analogRead(analogZPin);
  int c = analogRead(analogZPin);
  return max(min(a, b), min(max(a, b), c)); // median
}

// ===================================================
// AUTO DETECT WARNA GARIS
// ===================================================
void detectLineColor() {
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    selectChannel(i);
    delayMicroseconds(5);
    sum += readFiltered();
  }
  long avg = sum / 16;

  // asumsi background dominan saat start
  lineIsDark = avg > THRESHOLD;

  Serial.print("TRACK MODE: ");
  Serial.println(lineIsDark ? "DARK LINE" : "WHITE LINE");
}

// ===================================================
void readSensors() {
  for (int i = 0; i < 16; i++) {
    selectChannel(i);
    delayMicroseconds(5);

    int adc = readFiltered();
    sensor[i] = lineIsDark ? (adc > THRESHOLD) : (adc < THRESHOLD);

#if DEBUG_SENSOR
    Serial.print(sensor[i]);
    Serial.print(" ");
#endif
  }

#if DEBUG_SENSOR
  Serial.println();
#endif
}

// ===================================================
float getLineError() {
  long sum = 0, cnt = 0;
  for (int i = 0; i < 16; i++) {
    if (sensor[i]) {
      sum += map(i, 0, 15, -100, 100);
      cnt++;
    }
  }
  if (cnt == 0) return lastError;
  return (float)sum / cnt;
}

// ===================================================
// NODE DETECTION
// ===================================================
bool nodeLeft()  { return sensor[0]  && sensor[1]  && sensor[2]; }
bool nodeRight() { return sensor[13] && sensor[14] && sensor[15]; }
bool nodeStraight() { return sensor[7] || sensor[8]; }

bool allWhite() {
  for (int i = 0; i < 16; i++)
    if (sensor[i]) return false;
  return true;
}

// ===================================================
// SMART TURN 90° (PAKAI SENSOR)
// ===================================================
void turnLeft90() {
  motorL.drive(-240);
  motorR.drive(240);
  while (true) {
    readSensors();
    if (sensor[7] || sensor[8]) break;
  }
}

void turnRight90() {
  motorL.drive(240);
  motorR.drive(-240);
  while (true) {
    readSensors();
    if (sensor[7] || sensor[8]) break;
  }
}

// ===================================================
void setup() {
  pinMode(s0Pin, OUTPUT);
  pinMode(s1Pin, OUTPUT);
  pinMode(s2Pin, OUTPUT);
  pinMode(s3Pin, OUTPUT);
  pinMode(analogZPin, INPUT);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  Serial.begin(115200);
  delay(500);

  detectLineColor();   // AUTO DETECT TRACK
  motorStop();
}

// ===================================================
void loop() {

  updateButtons();
  if (!robotRun) {
    motorStop();
    return;
  }

  readSensors();

  // ===== NODE HANDLING =====
  if (nodeLeft() && !nodeStraight()) {
    turnLeft90();
    return;
  }

  if (nodeRight() && !nodeStraight()) {
    turnRight90();
    return;
  }

  // ===== RECOVERY =====
  if (allWhite()) {
    motorL.drive(255);
    motorR.drive(-255);
    return;
  }

  // ===== PID =====
  error = getLineError();
  integral += error;
  integral = constrain(integral, -I_MAX, I_MAX);

  float pid = Kp * error
            + Ki * integral
            + Kd * (error - lastError);

  lastError = error;

  motorL.drive(constrain(baseSpeed + pid, -255, 255));
  motorR.drive(constrain(baseSpeed - pid, -255, 255));
}
