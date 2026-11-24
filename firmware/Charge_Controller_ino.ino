#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.hpp>

#define IR_PIN      3
#define RELAY_PIN   12
#define PWM_PIN     5

LiquidCrystal_I2C lcd(0x27, 16, 2);

const float VREF = 5.0;

// A0 divider: 82 kΩ top, 10 kΩ bottom
const float R1_A0 = 82000.0;
const float R2_A0 = 10000.0;
const float RATIO_A0 = (R1_A0 + R2_A0) / R2_A0;

// A1 divider: 33 kΩ top, 10 kΩ bottom
const float R1_A1 = 33000.0;
const float R2_A1 = 10000.0;
const float RATIO_A1 = (R1_A1 + R2_A1) / R2_A1;

float caliA0 = 1.0;
float caliA1 = 1.0;
float caliA3 = 0.991;

float voltage0, voltage1, voltage3;
float voltage3Filtered = 0.0;
float targetVoltage = 0.0;
int pwmValue = 0;
bool seriesMode = false;

int state = 0;
int firstDigit = 0;
bool inputDisplayActive = false;
unsigned long inputDisplayStart = 0;

const char* surname = "BOOI";

// ----- Custom LCD Characters -----
// Parallel LEFT half: one capacitor plate with leads to the right
byte parallelLeft[8] = {
  0b00001,   
  0b00111,   
  0b00101,   
  0b11100,   
  0b00100,   
  0b00101,   
  0b00111,   
  0b00001    
};

// Parallel RIGHT half: mirrored capacitor plate facing left
byte parallelRight[8] = {
  0b10000,   
  0b11100,   
  0b10100,   
  0b00111,   
  0b00100,   
  0b10100,   
  0b11100,   
  0b10000    
};

// Series: two capacitors stacked horizontally
byte seriesChar[8] = {
  0b00000,   
  0b01010,   
  0b01010,   
  0b11011,   
  0b01010,  
  0b01010, 
  0b00000,   
  0b00000   
};

void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, parallelLeft);
  lcd.createChar(1, parallelRight);
  lcd.createChar(2, seriesChar);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  updateDisplay();
}

void loop() {
  readVoltages();
  controlRelay();

  static unsigned long lastRegUpdate = 0;
  if (millis() - lastRegUpdate > 200) {
    regulateOutput();
    lastRegUpdate = millis();
  }

  if (IrReceiver.decode()) {
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      handleIR(IrReceiver.decodedIRData.command);
    }
    IrReceiver.resume();
  }

  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 50) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  if (inputDisplayActive && (millis() - inputDisplayStart > 1000)) {
    inputDisplayActive = false;
    state = 0;
    updateDisplay();
  }
}

// ================= VOLTAGE READING =================
void readVoltages() {
  long sum0 = 0, sum1 = 0, sum3 = 0;
  for (int i = 0; i < 8; i++) {
    sum0 += analogRead(A0);
    sum1 += analogRead(A1);
    sum3 += analogRead(A3);
  }

  float adc0 = sum0 / 8.0;
  float adc1 = sum1 / 8.0;
  float adc3 = sum3 / 8.0;

  voltage0 = adc0 * (VREF / 1023.0) * RATIO_A0 * caliA0;
  voltage1 = adc1 * (VREF / 1023.0) * RATIO_A1 * caliA1;

  float rawV3 = adc3 * (VREF / 1023.0) * caliA3;

  voltage3Filtered = voltage3Filtered * 0.85 + rawV3 * 0.15;
  voltage3 = voltage3Filtered;
}

void controlRelay() {
  if (voltage0 > 18.5 && !seriesMode) {
    seriesMode = true;
    digitalWrite(RELAY_PIN, HIGH);
    updateDisplay();
  } else if (voltage0 < 10.0 && seriesMode) {
    seriesMode = false;
    digitalWrite(RELAY_PIN, LOW);
    updateDisplay();
  }
}

// ================= OUTPUT REGULATION =================
void regulateOutput() {
  int ffPwm = (int)((targetVoltage + 0.7) / 5.0 * 255.0);
  ffPwm = constrain(ffPwm, 0, 255);

  float error = targetVoltage - voltage3;
  float correctionF = error * 5.0;
  int correction = (int)correctionF;
  if (correctionF > 0 && correction == 0) correction = 1;
  if (correctionF < 0 && correction == 0) correction = -1;

  pwmValue = ffPwm + correction;
  pwmValue = constrain(pwmValue, 0, 255);

  analogWrite(PWM_PIN, pwmValue);
}

void updateDisplay() {
  lcd.setCursor(0, 0);

  if (seriesMode) {
    lcd.write(byte(2));   // series: stacked capacitors
    lcd.write(byte(2));
  } else {
    lcd.write(byte(0));   // parallel: left plate
    lcd.write(byte(1));   // parallel: right plate
  }

  lcd.print(" ");
  lcd.print(voltage3, 2);
  lcd.print("V ");
  lcd.print(surname);

  lcd.setCursor(0, 1);

  if (inputDisplayActive) {
    lcd.print("Input = ");
    lcd.print(targetVoltage, 2);
    lcd.print("V   ");
  } else {
    lcd.print(voltage0, 2);
    lcd.print("V ");
    lcd.print(voltage1, 2);
    lcd.print("V  ");
  }
}

void handleIR(uint8_t cmd) {
  int digit = -1;

  if      (cmd == 0x0C) digit = 0;
  else if (cmd == 0x10) digit = 1;
  else if (cmd == 0x11) digit = 2;
  else if (cmd == 0x12) digit = 3;
  else if (cmd == 0x14) digit = 4;
  else if (cmd == 0x15) digit = 5;
  else if (cmd == 0x16) digit = 6;
  else if (cmd == 0x18) digit = 7;
  else if (cmd == 0x19) digit = 8;
  else if (cmd == 0x1A) digit = 9;
  else return;

  if (state == 0) {
    firstDigit = digit;
    targetVoltage = (float)firstDigit;
    inputDisplayActive = true;
    inputDisplayStart = millis();
    state = 1;
  } else if (state == 1) {
    targetVoltage = (float)firstDigit + (float)digit / 10.0;
    inputDisplayActive = true;
    inputDisplayStart = millis();
    state = 0;
  }

  updateDisplay();
}