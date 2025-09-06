#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* -------- Pin assignments -------- */
const int PIN_LDR_A0 = A0;
const int PIN_DHT    = 2;
const int PIN_GAS_A0 = A2;   // MQ-2 analog output
const int PIN_BUZZER = 4;    // via NPN transistor

// L298N Channel for FAN (Motor 1 -> OUT A/B)
const int PIN_ENA = 5;       // PWM
const int PIN_IN1 = 11;
const int PIN_IN2 = 12;

// L298N Channel for PUMP (Motor 2 -> OUT C/D)
const int PIN_ENB = 6;       // PWM
const int PIN_IN3 = 13;
const int PIN_IN4 = A1;      // use analog pin as digital

// Ultrasonic
const int PIN_TRIG = 7;
const int PIN_ECHO = 8;

// IR proximity for dispensing
const int PIN_IR   = 9;

// Manual reset button (to GND)
const int PIN_RESET = 10;

/* -------- Devices -------- */
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* -------- Tunables -------- */
int   LDR_THRESHOLD     = 300;    // System ON if LDR <= this
float TEMP_ON_C         = 31;     // Fan ON threshold
long  TANK_DEPTH_CM     = 30;    
long  ULTRA_TIMEOUT_US  = 25000; 

int   GAS_THRESHOLD     = 300;   // Gas trigger threshold (0–1023)

uint8_t FAN_PWM   = 255;
uint8_t PUMP_PWM  = 255;

/* -------- State -------- */
bool systemActive   = false;
bool alarmLatched   = false;
bool fanOn          = false;
unsigned long lastLCD = 0;

// --- Buzzer silence timer ---
unsigned long buzzerSilenceUntil = 0;  // millis until buzzer is suppressed

/* -------- Helpers -------- */
long measureDistanceCm() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0) return -1;         
  long cm = duration / 58;              
  return cm;
}

void fan_set(bool on, uint8_t duty = 255) {
  if (on) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, duty);
  } else {
    analogWrite(PIN_ENA, 0);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
}

void pump_set(bool on, uint8_t duty = 255) {
  if (on) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, duty);
  } else {
    analogWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  }
}

void setBuzzer(bool on) { digitalWrite(PIN_BUZZER, on ? HIGH : LOW); }

/* -------- Arduino setup -------- */
void setup() {
  Serial.begin(9600);

  pinMode(PIN_IR,    INPUT);
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  fan_set(false, 0);
  pump_set(false, 0);
  setBuzzer(false);

  dht.begin();
  lcd.init(); lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("EEE4604 System");
  lcd.setCursor(0,1); lcd.print("Init...");
  delay(800);
}

/* -------- Main Loop -------- */
void loop() {
  // --- LDR Control ---
  int ldr = analogRead(PIN_LDR_A0);
  systemActive = (ldr <= LDR_THRESHOLD);

  if (!systemActive) {
    fan_set(false, 0);
    pump_set(false, 0);
    setBuzzer(false);
    alarmLatched = false;

    if (millis() - lastLCD > 500) {
      lastLCD = millis();
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("System: OFF");
      lcd.setCursor(0,1); lcd.print("High light...");
    }

    Serial.print("LDR: "); Serial.print(ldr);
    Serial.println(" (System OFF)");
    delay(100);
    return;
  }

  // --- Temperature & Humidity ---
  float tempC = dht.readTemperature();
  float hum   = dht.readHumidity();

  if (!isnan(tempC)) {
    if (tempC >= TEMP_ON_C) {
      fan_set(true, FAN_PWM);
      fanOn = true;
    } else {
      fan_set(false, 0);
      fanOn = false;
    }
  }

  // --- Gas sensor (analog threshold) ---
  int gasValue = analogRead(PIN_GAS_A0);
  bool gasDetected = (gasValue >= GAS_THRESHOLD);   // ✅ buzzer ON if >= threshold

  if (gasDetected) {
    alarmLatched = true;
  } else {
    alarmLatched = false;  // buzzer OFF when below threshold
  }

  // Reset button pressed → silence buzzer for 12 minutes
  if (digitalRead(PIN_RESET) == LOW) {
    alarmLatched = false;
    buzzerSilenceUntil = millis() + (12UL * 60UL * 1000UL); // 12 min
  }

  // Only allow buzzer if not in silence period
  bool buzzerActive = false;
  if (alarmLatched && millis() > buzzerSilenceUntil) {
    buzzerActive = true;
  }
  setBuzzer(buzzerActive);

  // --- Ultrasonic Tank Level ---
  long dist = measureDistanceCm();
  int levelPct = 0;
  if (dist > 0 && dist <= TANK_DEPTH_CM + 5) {
    long filled = TANK_DEPTH_CM - dist;
    if (filled < 0) filled = 0;
    if (filled > TANK_DEPTH_CM) filled = TANK_DEPTH_CM;
    levelPct = (int)((filled * 100L) / TANK_DEPTH_CM);
  }

  // --- IR dispensing (hand detection only) ---
  int irRaw = digitalRead(PIN_IR);
  bool handPresent = (irRaw == LOW);      
  bool pumpActive  = (systemActive && handPresent);

  pump_set(pumpActive, PUMP_PWM);

  if (pumpActive) {
    Serial.println("[PUMP] Dispensing (IR detected hand)");
  }

  // --- LCD Update ---
  if (millis() - lastLCD > 500) {
    lastLCD = millis();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T:"); lcd.print(!isnan(tempC) ? (int)tempC : -1); lcd.print("C ");
    lcd.print("Lvl:"); lcd.print(levelPct); lcd.print("%");
    lcd.setCursor(0,1);
    lcd.print("Fan:"); lcd.print(fanOn ? "ON " : "OFF");
    lcd.print(" Gas:"); lcd.print(gasDetected ? "!" : "-");
  }

  // --- Serial Monitoring (real-time) ---
  Serial.print("LDR: "); Serial.print(ldr);
  Serial.print(" | Temp: "); Serial.print(tempC); Serial.print(" C");
  Serial.print(" | Hum: "); Serial.print(hum); Serial.print(" %");
  Serial.print(" | Water Level: "); Serial.print(levelPct); Serial.print(" %");
  Serial.print(" | Distance: "); Serial.print(dist); Serial.print(" cm");
  Serial.print(" | GasVal: "); Serial.print(gasValue);
  Serial.print(" | Buzzer: "); Serial.print(buzzerActive ? "ON" : "OFF");
  Serial.print(" | IRraw: "); Serial.print(irRaw);
  Serial.println();

  delay(200);
}
