// ESP32 B: TB6612 motors, ultrasonic auto-brake, MQ2 alcohol sensor, UART link to ESP A

// ---- Pins ----
#define PWMA 26
#define AIN1 13
#define AIN2 14
#define PWMB 25
#define BIN1 12
#define BIN2 27
#define STBY 33
#define TRIG 5
#define ECHO 18
#define MQ2_PIN 34
#define RX2 16
#define TX2 17

// ---- Tuning ----
#define BRAKE_CM     30        // auto-brake distance
#define LINK_TIMEOUT 500       // ms without a command from A -> stop
#define WARMUP_MS    25000UL   // MQ2 heater warm-up
#define BASE_MS      3000UL    // clean-air baseline measurement after warm-up
#define ALC_DELTA    250       // rise above baseline = alcohol (tune this)

#define PWM_FREQ 20000
#define PWM_BITS 8
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_WRITE(pin, v) ledcWrite(pin, v)
#else
  #define PWM_ATTACH(pin) (ledcSetup((pin) == PWMA ? 0 : 1, PWM_FREQ, PWM_BITS), ledcAttachPin(pin, (pin) == PWMA ? 0 : 1))
  #define PWM_WRITE(pin, v) ledcWrite((pin) == PWMA ? 0 : 1, v)
#endif

int cmdPwm = 0;
unsigned long lastCmd = 0;
String rxLine;

int distCm = 999;
int nearCnt = 0;
bool brake = false;

int gasVal = 0, gasBase = 0, gasThr = 0;
bool gasReady = false, alc = false;
unsigned long bootMs, baseSum = 0, baseCnt = 0;

unsigned long tUS = 0, tGas = 0, tTx = 0, tDbg = 0;

// pwm > 0: forward. pwm == 0: coast, or short-brake when hard = true
void driveMotors(int pwm, bool hard) {
  if (pwm > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    PWM_WRITE(PWMA, pwm); PWM_WRITE(PWMB, pwm);
  } else if (hard) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
    PWM_WRITE(PWMA, 255); PWM_WRITE(PWMB, 255);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    PWM_WRITE(PWMA, 0); PWM_WRITE(PWMB, 0);
  }
}

int readDistanceCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000);   // about 4 m max
  if (us == 0) return 999;                // nothing in range
  return (int)(us / 58);
}

void updateGas() {
  long s = 0;
  for (int i = 0; i < 8; i++) s += analogRead(MQ2_PIN);
  gasVal = s / 8;

  unsigned long t = millis() - bootMs;
  if (t < WARMUP_MS) { gasReady = false; return; }
  if (t < WARMUP_MS + BASE_MS) {            // measure clean-air baseline
    baseSum += gasVal; baseCnt++;
    gasReady = false;
    return;
  }
  if (!gasReady) {                          // finish baseline once
    gasBase = baseCnt ? baseSum / baseCnt : gasVal;
    gasThr = gasBase + ALC_DELTA;
    gasReady = true;
  }
  alc = gasVal > gasThr;
}

void readFromA() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      if (rxLine.startsWith("S,")) {
        cmdPwm = constrain(rxLine.substring(2).toInt(), 0, 255);
        lastCmd = millis();
      }
      rxLine = "";
    } else if (c != '\r' && rxLine.length() < 30) {
      rxLine += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  PWM_ATTACH(PWMA); PWM_ATTACH(PWMB);
  driveMotors(0, false);
  digitalWrite(STBY, HIGH);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);

  bootMs = millis();
  Serial.println("ESP B ready. MQ2 warming up...");
}

void loop() {
  readFromA();
  unsigned long now = millis();

  if (now - tUS >= 60) {                    // ultrasonic
    tUS = now;
    distCm = readDistanceCm();
    if (distCm <= BRAKE_CM) nearCnt++; else nearCnt = 0;
    brake = (nearCnt >= 2);                 // 2 hits in a row = real obstacle
  }

  if (now - tGas >= 100) { tGas = now; updateGas(); }

  int out = cmdPwm;
  if (now - lastCmd > LINK_TIMEOUT) out = 0;   // lost A -> stop
  if (brake) out = 0;                          // obstacle -> stop
  driveMotors(out, brake);

  if (now - tTx >= 100) {                      // report to A
    tTx = now;
    Serial2.printf("D,%d,%d,%d,%d,%d,%d\n",
                   distCm, gasVal, gasThr, brake ? 1 : 0, alc ? 1 : 0, gasReady ? 1 : 0);
  }

  if (now - tDbg >= 500) {
    tDbg = now;
    Serial.printf("dist=%d gas=%d thr=%d ready=%d alc=%d cmd=%d out=%d brake=%d\n",
                  distCm, gasVal, gasThr, gasReady, alc, cmdPwm, out, brake);
  }
}