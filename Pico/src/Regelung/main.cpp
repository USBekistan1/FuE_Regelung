#include <Wire.h>                                                 //I2C
#include <Adafruit_GFX.h>                                         //OLED
#include <Adafruit_SH110X.h>                                      //OLED
#include "TLx493D_inc.hpp"                                        //Hall Sensor
#include <math.h>
#include <algorithm>

// --- Flash ---
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>   // für memset/memcpy

// Sensor Funktionen
float ReadSensorEMA();
float MagToDiameter(float mag);
void  ResetMagEMA();

// Kalibrierung Funktionen
void RunCalibration();
void SaveCalibrationToFlash(float newCalA, float newCalB);
bool loadCalibrationFromFlash();
void CalculateLogCalibrationFit();

// I2C/Recover  Funktionen
bool requestData();
bool i2cRecoverBus();
bool handleI2CRecoverHold();
void sendSpeedToSlave(int16_t targetStepsPerSec);

// Display Funktionen
void Update_Display_Man(float measuredDiameterMm);
void update_Display_Auto(float targetDiameterMm, float measuredDiameterMm, bool rampDone);
void Update_Display_TargetMM(float targetDiameterMm);
void showError(const char* msg);
void showMessage(const char* msg, int textSize);
void showCalibrationParams(float calA, float calB);

// Modes
void    ManMode();
void    AutoMode();
int16_t CalculateStepsPerSec(float targetDiameterMm);
float   CalculateMetersPerMin(long stepsPerSec);

Adafruit_SH1106G display(128, 64, &Wire);                                 // Selber I2C Bus -> Display Updaterate checken!!
using namespace ifx::tlx493d;
TLx493D_A1B6 Tlv493dMagnetic3DSensor(Wire, TLx493D_IIC_ADDR_A0_e);        // Selber Bus (Sensor & Display)

// ----- Kalibrierung -----
#define PIN_CONFIRM_BUTTON 18            // Pin für den Bestätigungs-Knopf

#define CAL_SAMPLE_COUNT 9
const float CAL_DIAMETERS_MM[CAL_SAMPLE_COUNT] = {1.0f, 1.2f, 1.4f, 1.6f, 1.7f, 1.75f, 1.8f, 1.9f, 2.0f};
float calA = -1.1563, calB = 5.9946;                                              // Koeffizienten der Kalibrierungsgeraden
float magSamples[CAL_SAMPLE_COUNT];    // Sensorwerte (B-Betrag)
float diameterSamples[CAL_SAMPLE_COUNT];    // Referenzdurchmesser
bool regressionDone = false;

// ----- Flash-Speicher für Kalibrierung auf dem Pico -----
#define FLASH_TARGET_OFFSET (256 * 1024)  // 256 kB hinter Programmanfang
#define CALIB_MAGIC_TAG 0x4E696C73            // 'Nils' als Magic

struct CalibData {
    uint32_t magic;
    float calA;
    float calB;
};

// Zeiger auf den Kalibrierbereich im Flash (XIP-Adresse)
const CalibData* const flashCalib = (const CalibData*)(XIP_BASE + FLASH_TARGET_OFFSET);

// ----- EMA -----
const unsigned long SAMPLE_PERIOD_MS = 20;  // 50 Hz

unsigned long emaLastSampleMs = 0;
bool          emaInitialized    = false;
float         emaX = 0.0f, emaY = 0.0f, emaZ = 0.0f;
float         emaLastMagnitude  = 0.0f;

// Weitere Variablen
const int STEPS_PER_REV = 800;                  // Schritte pro Umdrehung
const float DIAMETER = 0.05;                    // Durchmesser der Welle in Metern
const float CIRCUMFERENCE = PI * DIAMETER;      // Umfang in Metern
static unsigned long lastDisplayMs = 0;

// -------- I2C ----------
volatile int16_t targetStepsPerS = 0;                                         // Zielgeschwindigkeit, 2 Byte
volatile int16_t CurrentStepsPerS = 0;                                         // Ist-Geschwindigkeit
volatile bool isAutoMode = false;                                               // Default Regelbetrieb
volatile bool isMotorRunning = true;                                           // warum volatile? Keine ISR oder? Juckt wahrscheinlich nicht

// --- I2C Bus Recovery (Pico Arduino) ---
static const int I2C_SDA_PIN = 4;
static const int I2C_SCL_PIN = 5;
volatile bool blockRegStartUntilNewPress = false;               //Kein Automatischer Regelstart nach Recovery

// ---- Vorsteuerung ----
const float CONST_C = 5.939;       // Prozesskonstante
float targetDiameterMm = 1.75;       // Gewünschter Zieldurchmesser in mm
int16_t calculatedStepsPerSec = 0;       // Der berechnete Wert für den Arduino
bool rampDone = 0;
static unsigned long buttonPressStartMs = 0;
enum AutoState : uint8_t { AUTO_SELECT_SETPOINT, AUTO_FEEDFORWARD_ONLY, AUTO_CONTROL_ACTIVE };
AutoState autoState = AUTO_SELECT_SETPOINT;
volatile int16_t encoderDelta = 0;

// ---- Regelung ----
const float KI_GAIN = 8.0f;
float integralErrorSum = 0.0f;
const float INTEGRAL_MAX = 1000.0f;

// ---- Hysterese ----
const float DEADBAND_IN_MM  = 0.06f;  // Regler geht AN, wenn größer
const float DEADBAND_OUT_MM = 0.04f;  // Regler geht AUS, wenn kleiner
static bool isControlActive = false;


// ======== EMA-Filter für TLx493D ========
// Läuft nicht-blockierend, 50 Hz Abtastrate (20 ms Schrittzeit)
// Glättet die Magnetfeldmessung über X/Y/Z und gibt den Betrag zurück.

float ReadSensorEMA() {
  static uint32_t okCount = 0;                        // Erfolgreiches Auslese?
  static uint32_t failCount = 0;
  static unsigned long lastStatMs = 0;

  // ---- EMA Parameter ----
  const float EMA_TAU_S = 0.20f;                      
  const float dt = SAMPLE_PERIOD_MS / 1000.0f;
  const float alpha = 1.0f - expf(-dt / EMA_TAU_S);
  const unsigned long now = millis();

  // 50 Hz gating
  if (now - emaLastSampleMs < SAMPLE_PERIOD_MS) {
    return emaLastMagnitude;
  }
  emaLastSampleMs = now;

  double x, y, z;

  bool ok = Tlv493dMagnetic3DSensor.getMagneticFieldAndTemperature(&x, &y, &z, nullptr);        // Sensor lesen erfolgreich?

  if (ok) {
    failCount = 0;
    okCount++;

    if (!emaInitialized) {                      // Erste EMA Werte ohen EMA (Initialisierung9)
      emaX = (float)x;
      emaY = (float)y;
      emaZ = (float)z;
      emaInitialized = true;
    } else {                                    // EMA nach Initialisierung
      emaX += alpha * ((float)x - emaX);
      emaY += alpha * ((float)y - emaY);
      emaZ += alpha * ((float)z - emaZ);
    }

    emaLastMagnitude = sqrtf(emaX * emaX + emaY * emaY + emaZ * emaZ);    // Ein Gesamtwert

  } else {                                      // Lesen nicht erfolgreich
    emaInitialized = false;
    failCount++;

    /*if (now - lastReinit > REINIT_PERIOD_MS) {
      lastReinit = now;
      bool rec = i2cRecoverBus();
      if (Serial) Serial.println(rec ? "I2C rec OK (sensor)" : "I2C rec FAIL (sensor)");
      Tlv493dMagnetic3DSensor.begin();
    }*/

    if (failCount >= 10) {
      emaLastMagnitude = NAN;  // NAN Ausgabe auf Display
    }
  }

  // 1 Hz Debug Ausgabe
  if (now - lastStatMs > 1000) {
    lastStatMs = now;
    if (Serial) {
      Serial.print("TLV ok=");
      Serial.print(okCount);
      Serial.print(" fail=");
      Serial.println(failCount);
    }
  }

  return emaLastMagnitude;
}

void ResetMagEMA() {          //Frische Ema für jeden neuen Stab -> Kein Mitschleppen alter Werte der anderen Stäbe
  emaInitialized = false;
  emaX = emaY = emaZ = 0.0f;
  emaLastMagnitude = 0.0f;
  emaLastSampleMs = 0;
}

bool i2cRecoverBus() {
  // 1) Pins als GPIO open-drain-like: INPUT_PULLUP (High über Pullup)
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  // Wenn SCL LOW ist, hält jemand den Clock low -> schwerer Fall (z.B. Slave kaputt)
  if (digitalRead(I2C_SCL_PIN) == LOW) {
    return false;
  }

  // 2) Wenn SDA LOW: versuche, den Slave "auszutakten"
  if (digitalRead(I2C_SDA_PIN) == LOW) {
    pinMode(I2C_SCL_PIN, OUTPUT);

    for (int i = 0; i < 9; i++) {
      digitalWrite(I2C_SCL_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(I2C_SCL_PIN, LOW);
      delayMicroseconds(5);
    }

    // SCL wieder freigeben
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(5);
  }

  // 3) STOP condition erzwingen: SDA LOW -> SCL HIGH -> SDA HIGH
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);

  pinMode(I2C_SCL_PIN, INPUT_PULLUP); // SCL HIGH (über Pullup)
  delayMicroseconds(5);

  pinMode(I2C_SDA_PIN, INPUT_PULLUP); // SDA HIGH (über Pullup) -> STOP
  delayMicroseconds(5);

  // 4) I2C wieder initialisieren
  Wire.begin();
  Wire.setClock(50000);     
  Wire.setTimeout(50);      
  display.begin(0x3C);
  Tlv493dMagnetic3DSensor.begin();  
  
  // Erfolg, wenn beide Linien HIGH sind
  return (digitalRead(I2C_SDA_PIN) == HIGH) && (digitalRead(I2C_SCL_PIN) == HIGH);
}

bool handleI2CRecoverHold() {
  const unsigned long RECOVER_HOLD_MS = 5000;         // Benötigte Knopfdruckdauer

  static unsigned long holdStartMs = 0;
  static bool isArmed = true;

  unsigned long now = millis();
  bool isButtonPressed = (digitalRead(PIN_CONFIRM_BUTTON) == HIGH);

  if (!isButtonPressed) {         // Macht nichts, wenn Knopf nicht gedrückt
    holdStartMs = 0;
    isArmed = true;
    return false;
  }

  if (holdStartMs == 0) holdStartMs = now;

  if (isArmed && (now - holdStartMs >= RECOVER_HOLD_MS)) {
    isArmed = false;

    showMessage("I2C recover...\nBitte warten", 1);
    if (Serial) Serial.println("Manual I2C recover (5s hold)");

    bool rec = i2cRecoverBus();
    blockRegStartUntilNewPress = true;

    if (rec) {
      showMessage("I2C recover: OK", 1);
      if (Serial) Serial.println("I2C recover OK");
    } else {
      showError("I2C recover FAIL");
      if (Serial) Serial.println("I2C recover FAIL");
    }

    ResetMagEMA();

    // Muss loslassen, sonst retrigger/Confirm-chaos
    while (digitalRead(PIN_CONFIRM_BUTTON) == HIGH) delay(5);

    // States zurücksetzen für nächsten Hold
    holdStartMs = 0;
    isArmed = true;

    return true; // wurde getriggert
  }

  return false;
}

void RunCalibration() {
  const unsigned long CAL_WARMUP_MS = 500;   // Zeit, damit EMA sich auf neuen Stab einstellt
  const unsigned long CAL_SETTLE_MS = 1000;  // Zeit, über die wir für den Mittelwert sampeln
  const unsigned long CAL_SAMPLE_DELAY_MS = 20;

  while (digitalRead(PIN_CONFIRM_BUTTON) == HIGH) {       // Warten bis Knopf wieder losgelassen wird
      delay(50);
  }

  for (int sampleIndex = 0; sampleIndex < CAL_SAMPLE_COUNT; sampleIndex++) {        // Abarbeiten der Prüfstäbe
    float diameterMm = CAL_DIAMETERS_MM[sampleIndex];

    // 1. Anzeige des aktuellen Prüfstabs
    char line[20];
    sprintf(line, "Stab: %.2f mm", diameterMm);
    Serial.println(line);
    showMessage(line, 2);

    // 2. Auf Knopfdruck warten (LOW -> HIGH)
    while (digitalRead(PIN_CONFIRM_BUTTON) == LOW) {
      delay(50);
    }
    // warten bis losgelassen (HIGH -> LOW)
    while (digitalRead(PIN_CONFIRM_BUTTON) == HIGH) {
      delay(50);
    }

    ResetMagEMA();        // Reset für neuen Stab

    // 3a. WARMUP-PHASE: EMA darf sich auf den neuen Stab einstellen
    unsigned long startMs = millis();
    while (millis() - startMs < CAL_WARMUP_MS) {
      (void)ReadSensorEMA();   // Wert wird verworfen, nur Filter updaten
      delay(CAL_SAMPLE_DELAY_MS);
    }

    // 3b. SETTLE-PHASE: jetzt Werte für den Mittelwert einsammeln
    float magSum = 0.0f;
    int   magCount = 0;
    startMs = millis();

    while (millis() - startMs < CAL_SETTLE_MS) {      // Summe der EMA Werte über Sample Zeit
      float magNow = ReadSensorEMA();
      magSum += magNow;
      magCount++;
      delay(CAL_SAMPLE_DELAY_MS);
    }

    float magAvg = (magCount > 0) ? (magSum / magCount) : 0.0f;     // Durchschnitt der Werte
    
    // Debug Ausgabe
    Serial.print("B_avg[");
    Serial.print(sampleIndex);
    Serial.print("] = ");
    Serial.println(magAvg, 6);

    if (magAvg <= 0) {
      showError("Ungueltiger Sensorwert");
      return;
    }

    diameterSamples[sampleIndex] = diameterMm;
    magSamples[sampleIndex] = magAvg;
  }

  // 4. Regression durchführen
  CalculateLogCalibrationFit();
  regressionDone = true;

  // 5. Parameter anzeigen
  showCalibrationParams(calA, calB);  // deine Anzeige-Funktion für a,b
  delay(2000);

  // 6. Kalibrierung im Flash speichern
  SaveCalibrationToFlash(calA, calB);
}

void CalculateLogCalibrationFit() {
  float sumLnB = 0.0f, sumD = 0.0f, sumLnB2 = 0.0f, sumLnB_D = 0.0f;
  int nEff = 0;

  for (int i = 0; i < CAL_SAMPLE_COUNT; i++) {
    if (magSamples[i] <= 0) continue;
    float lnB = log(magSamples[i]);
    float D = diameterSamples[i];

    sumLnB   += lnB;
    sumD     += D;
    sumLnB2  += lnB * lnB;
    sumLnB_D += lnB * D;
    nEff++;
  }

  float denom = nEff * sumLnB2 - sumLnB * sumLnB;
    if (denom != 0 && nEff > 0) {
    calA = (nEff * sumLnB_D - sumLnB * sumD) / denom;
    calB = (sumD - calA * sumLnB) / nEff;

    Serial.print("Kalibrierung fertig. a = ");
    Serial.print(calA, 6);
    Serial.print(" , b = ");
    Serial.println(calB, 6);

  } else {
    showError("Regression fehlgeschlagen");
  }
}

bool loadCalibrationFromFlash() {
  if (flashCalib->magic != CALIB_MAGIC_TAG) {
      Serial.println("Flash-Kalibrierung: kein gueltiger Eintrag");
      return false;
  }

  calA = flashCalib->calA;
  calB = flashCalib->calB;

  Serial.print("Flash-Kalibrierung geladen: a=");
  Serial.print(calA, 6);
  Serial.print(" , b=");
  Serial.println(calB, 6);

  return true;
}

void SaveCalibrationToFlash(float newCalA, float newCalB) {
    CalibData data;
    data.magic = CALIB_MAGIC_TAG;
    data.calA = newCalA;
    data.calB = newCalB;

    uint8_t line[FLASH_PAGE_SIZE];
    memset(line, 0xFF, sizeof(line));
    memcpy(line, &data, sizeof(data));

    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, line, FLASH_PAGE_SIZE);
    restore_interrupts(irq_state);

    Serial.println("Kalibrierung im Flash gespeichert");
}

void Update_Display_Man(float measuredDiameterMm) {
  float speed_m_per_min = CalculateMetersPerMin(CurrentStepsPerS);  
  display.clearDisplay();
  // Display statischen Text anzeigen
  display.setTextSize(1.95);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Speed: ");
  //display.setCursor(0, 12);
  display.println(speed_m_per_min, 1);
  display.print(" m/min");
  //display.setCursor(0,24);
  display.println("Ist-Durchm.: ");
  //display.setCursor(0,36);
  display.println(measuredDiameterMm);
  display.println(" mm");

  Serial.println("OLED: before display()");
  display.display();
  Serial.println("OLED: after display()");
}

void update_Display_Auto(float targetDiameterMm, float measuredDiameterMm, bool rampDone){
  float speed_m_per_min = CalculateMetersPerMin(CurrentStepsPerS);  

  display.clearDisplay();
  display.setTextSize(1.95);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);

  display.print("Soll: ");
  display.print(targetDiameterMm, 2);
  display.println(" mm");

  display.print("Ist : ");
  display.print(measuredDiameterMm, 2);
  display.println(" mm");

  display.print("Speed: ");
  display.print(speed_m_per_min, 1);
  display.println(" m/min");

  display.print("AUTO: ");
  display.println(rampDone ? "Regler aktiv" : "Vorsteuerung -> Knopf druecken wenn Durchm. stabil");

  //serial.println("OLED: before display()");
  display.display();
  //serial.println("OLED: after display()");
}

void Update_Display_TargetMM(float targetDiameterMm){
  display.clearDisplay();
  display.setTextSize(1.95);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);

  display.print("Soll: ");
  display.print(targetDiameterMm, 2);
  display.println(" mm");

  display.println();

  display.print("Mit Knopf bestaetigen");

  display.display();
}

void showError(const char* msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.println("!! FEHLER !!");
  display.println(msg);
  display.display();
  Serial.println(msg);
  delay(2000);
}

void showMessage(const char* msg, int textSize) {
  display.clearDisplay();
  display.setTextSize(textSize);
  display.setTextColor(SH110X_WHITE);

  int y = 0;
  const char* p = msg;

  while (*p) {
    display.setCursor(0, y);

    // Zeichen bis zum Zeilenumbruch ausgeben
    while (*p && *p != '\n') {
      display.write(*p++);
    }

    // Zum nächsten Zeichen springen, falls '\n'
    if (*p == '\n') p++;

    y += 12;  // Zeilenhöhe
  }

  display.display();
}

void showCalibrationParams(float a, float b) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  display.println("Kalibrierung OK");
  display.println();
  display.print("a = ");
  display.println(a, 4);
  display.print("b = ");
  display.println(b, 4);

  display.display();
}

bool requestData() {
    const uint8_t expected = 6;

    // Master fragt 6 Bytes beim Uno (Adresse 0x08) an
    uint8_t received = Wire.requestFrom(0x08, expected);  

    if (received != expected) {
        Serial.print("I2C Fehler: erwartet 6, bekommen ");
        Serial.println(received);

        // Buffer leeren, falls Schrott drin ist
        while (Wire.available()) Wire.read();
        return false;
    }

    int16_t speed   = (Wire.read() << 8) | Wire.read();
    bool running    = Wire.read();
    bool modeVal    = Wire.read();

    int16_t delta = (int16_t)((Wire.read() << 8) | Wire.read());

    CurrentStepsPerS = speed;
    isMotorRunning   = running;
    isAutoMode        = modeVal;
    encoderDelta    = delta;

    return true;
}

float CalculateMetersPerMin(long stepsPerSec) {
  float revsPerSec = (float)stepsPerSec / STEPS_PER_REV;      // Umdrehungen pro Sekunde
  float revsPerMin = revsPerSec * 60.0;                       // U/min
  float v_mPerMin = CIRCUMFERENCE * revsPerMin;               // m/min
  return v_mPerMin;
}

void ManMode(){
  if (handleI2CRecoverHold()) return;       //Bus Recover Abfrage
  unsigned long now = millis();
  autoState   = AUTO_SELECT_SETPOINT;
  rampDone    = false;
  integralErrorSum = 0.0f;
  buttonPressStartMs  = 0;  

  // --- Magnetfeld -> Durchmesser ---
  float mag = ReadSensorEMA();
  float diameterMm = NAN;
  if (mag > 1e-6f && isfinite(mag)) {
    diameterMm = calA * logf(mag) + calB;
  }

  // --- Anzeige + Serial alle 200 ms ---
  static unsigned long lastPrintTime = 0;
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;

    Update_Display_Man(diameterMm);

    float t_s = now / 1000.0f;                  // Zeit in Sekunden
    float v_m_per_min = CalculateMetersPerMin(CurrentStepsPerS);

    Serial.print(t_s, 3);        
    Serial.print(';');
    Serial.print(v_m_per_min, 3); 
    Serial.print(';');
    Serial.println(diameterMm, 3);        
  }
}

int16_t CalculateStepsPerSec(float targetDiameterMm) {
   if (targetDiameterMm <= 0.1f) return 0;         // Schutz vor Division durch 0 oder Unsinn

  // 1. Prozessgeschwindigkeit berechnen (v = (C/d)^2)
  float v_soll_m_min = powf(CONST_C / targetDiameterMm, 2.0f);

  // 2. Umrechnung: m/min -> Steps/Sekunde
  // Herleitung aus deiner Funktion 'berechneGeschwindigkeit':
  // v_m_min = (steps / STEPS_PER_REV * 60) * CIRCUMFERENCE
  // => steps = (v_m_min * STEPS_PER_REV) / (60 * CIRCUMFERENCE)
  
  float steps_float  = (v_soll_m_min * (float)STEPS_PER_REV) / (60.0f * CIRCUMFERENCE);
  
  long steps_l = lroundf(steps_float);
  if (steps_l < 0) steps_l = 0;
  if (steps_l > 3000) steps_l = 3000;   // passend zu Arduino setMaxSpeed(3000)
  
  return (int16_t)steps_float;
}

void sendSpeedToSlave(int16_t targetStepsPerSec) {
  Wire.beginTransmission(0x08);
  // Wir senden den int16 in zwei Bytes (High Byte, Low Byte)
  Wire.write((uint8_t)((targetStepsPerSec >> 8) & 0xFF)); 
  Wire.write((uint8_t)(targetStepsPerSec & 0xFF));
  Wire.endTransmission();
}

void AutoMode() {
  if (autoState != AUTO_SELECT_SETPOINT) {           //Bus Recover Abfrage nuraußerhalb vom SetPoint
    if (handleI2CRecoverHold()) return;
  }

  unsigned long now = millis();
  //static unsigned long lastWrite = 0;
  const unsigned long WRITE_PERIOD_MS = 200;
  const unsigned long CONTROL_PERIOD_MS = 10000;
  // --- Hold für Stellwert, wenn Regler "aus" ---
  static int16_t holdCmdStepsPerSec = 0;
  static bool    holdValid    = false;
  static unsigned long lastWriteI2cWriteFFMs  = 0;
  static unsigned long lastWriteI2cCtrlMs = 0;

  // --- 0) Sollwert Einstellen
  if (autoState == AUTO_SELECT_SETPOINT) {
    if (encoderDelta != 0) {
      targetDiameterMm += 0.005f * (float)encoderDelta;              // 0.01mm Schritte

      // Limits setzen
      if (targetDiameterMm < 1.50f) targetDiameterMm = 1.50f;
      if (targetDiameterMm > 2.20f) targetDiameterMm = 2.20f;

      encoderDelta = 0;   // WICHTIG: Delta "verbrauchen"
    }

    Update_Display_TargetMM(targetDiameterMm);

    // Bestätigen per Hold-to-confirm
    if (digitalRead(PIN_CONFIRM_BUTTON) == HIGH) {
      if (buttonPressStartMs == 0) buttonPressStartMs = now;

      if (now - buttonPressStartMs >= 300) {
        // Warten bis Button wieder losgelassen wird
        while (digitalRead(PIN_CONFIRM_BUTTON) == HIGH) {
        delay(1);   // Mini-Delay gegen 100% CPU-Spin
        }

        buttonPressStartMs  = 0;
        integralErrorSum = 0.0f;
        rampDone    = false;               
        autoState   = AUTO_FEEDFORWARD_ONLY;    
      }
        if (now - buttonPressStartMs >= 300) {
          buttonPressStartMs  = 0;
          integralErrorSum = 0.0f;
          rampDone    = false;                 // Regler noch AUS
          autoState   = AUTO_FEEDFORWARD_ONLY;      // weiter: Vorsteuerung-only
          isControlActive = false;
          holdValid = false;
          holdCmdStepsPerSec = 0;
          lastWriteI2cWriteFFMs = now;
          lastWriteI2cCtrlMs = now;   // dt sauber
        }

    } else {
      buttonPressStartMs = 0;
    }     

    return; // im SETPOINT-State nichts senden/regeln
  }

  // --- 1) Durchmesser messen (mit Schutz gegen log(<=0)) ---
  float mag = ReadSensorEMA();
  float measuredDiameterMm = NAN;
  if (mag > 1e-6f && isfinite(mag)) {
  measuredDiameterMm = calA * logf(mag) + calB;
  }

  // --- 2) Vorsteuerung berechnen und an Arduino senden (WRITE) ---
  if (!rampDone){
    if (now - lastWriteI2cWriteFFMs >= WRITE_PERIOD_MS) {
      lastWriteI2cWriteFFMs = now;

      int16_t cmdStepsPerSec = 0;

      // Nur senden, wenn running + gültiger Sensorwert
      if (isMotorRunning) {
        cmdStepsPerSec = CalculateStepsPerSec(targetDiameterMm);
      } else {
        cmdStepsPerSec = 0;
      }

      targetStepsPerS = cmdStepsPerSec;
      sendSpeedToSlave(cmdStepsPerSec);
    }
  }

// --- RampDone erst bei LOSLASSEN nach >=300ms Hold ---
// --- ABER: nach Recover erst "neutral" werden und neuen Druck verlangen ---
if (!rampDone) {
  static bool rampArm = false;

  bool btn = (digitalRead(PIN_CONFIRM_BUTTON) == HIGH);

  // 0) Sperre nach Recover: erst einmal LOW gesehen -> dann freigeben,
  // und erst der NÄCHSTE Press darf wieder armen.
  if (blockRegStartUntilNewPress) {
    // solange Button noch HIGH ist: nix tun
    if (!btn) {
      // Button ist LOW -> System wieder "neutral"
      blockRegStartUntilNewPress = false;
      buttonPressStartMs = 0;
      rampArm = false;
    }
    // egal was: in diesem Zustand keine Arm/Confirm-Logik
  } 
  else {
    // Normaler Ablauf
    if (btn) {
      if (buttonPressStartMs == 0) buttonPressStartMs = now;
      if (!rampArm && (now - buttonPressStartMs >= 300)) {
        rampArm = true;  // wartet auf Release
      }
    } else {
      if (rampArm) {
        rampDone     = true;
        integralErrorSum  = 0.0f;
        holdValid    = false;
        holdCmdStepsPerSec = 0;
      }
      buttonPressStartMs = 0;
      rampArm = false;
    }
  }
} else {
  buttonPressStartMs = 0;
}

  // ---- 3) Regelung ----
  if (rampDone) {
    if (now - lastWriteI2cCtrlMs >= CONTROL_PERIOD_MS) {

      float dt = (now - lastWriteI2cCtrlMs) / 1000.0f;
      lastWriteI2cCtrlMs = now;

      int16_t cmdStepsPerSec = 0;

    // Sicherheitsabfrage: Regeln wir nur, wenn der Motor läuft und der Sensor OK ist

    if (isMotorRunning && !isnan(measuredDiameterMm) && isfinite(measuredDiameterMm)) {

      // A) Vorsteuerung (Feedforward)
      int16_t steps_feedforward = CalculateStepsPerSec(targetDiameterMm);

      // B) Fehler
      float errorMM = measuredDiameterMm - targetDiameterMm;
      float errorAbsMM   = fabsf(errorMM);

      // C) Deadband + Hysterese
      if (!isControlActive && errorAbsMM >= DEADBAND_IN_MM)  isControlActive = true;
      if ( isControlActive && errorAbsMM <= DEADBAND_OUT_MM) isControlActive = false;

      // D) Stellwert bilden
      if (isControlActive) {
        // Integral nur wenn aktiv
        integralErrorSum += errorMM * dt;

        // Anti-Windup
        if (integralErrorSum >  INTEGRAL_MAX) integralErrorSum =  INTEGRAL_MAX;
        if (integralErrorSum < -INTEGRAL_MAX) integralErrorSum = -INTEGRAL_MAX;

        float steps_correction = integralErrorSum * KI_GAIN;

        int32_t cmd = (int32_t)steps_feedforward + (int32_t)lroundf(steps_correction);
        if (cmd < 0) cmd = 0;
        if (cmd > 3000) cmd = 3000;

        cmdStepsPerSec = (int16_t)cmd;

        // HOLD updaten
        holdCmdStepsPerSec = cmdStepsPerSec;
        holdValid    = true;

      } else {
        // Regler AUS -> letzten Stellwert halten
        if (holdValid) {
          cmdStepsPerSec = holdCmdStepsPerSec;
        } else {
          // falls noch kein Hold existiert (z.B. direkt nach rampDone)
          cmdStepsPerSec = steps_feedforward;
          holdCmdStepsPerSec = cmdStepsPerSec;
          holdValid = true;
        }
      }

      // Grenzwerte
      if (cmdStepsPerSec < 0) cmdStepsPerSec = 0;
      if (cmdStepsPerSec > 3000) cmdStepsPerSec = 3000;

    } else {
      // Motor aus oder Sensor ungültig: Integral resetten und Stop
      cmdStepsPerSec = 0;
      integralErrorSum = 0.0f;
      isControlActive = false;
      holdValid    = false;
      holdCmdStepsPerSec = 0;
    }

    targetStepsPerS = cmdStepsPerSec;
    sendSpeedToSlave(cmdStepsPerSec);
    }
  }

  // --- 4) Anzeige ---
  if (millis() - lastDisplayMs >= 150){
    lastDisplayMs = millis();
    update_Display_Auto(targetDiameterMm, measuredDiameterMm, rampDone);
  }

  // --- 5) Logging (optional, passt super fürs Tuning) ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 200) {
    lastPrint = now;

    float t_s = now / 1000.0f;
    float v_m_per_min = CalculateMetersPerMin(CurrentStepsPerS);

    Serial.print(t_s, 3);
    Serial.print(';');
    Serial.print(v_m_per_min, 3);
    Serial.print(';');
    if (!isnan(measuredDiameterMm) && isfinite(measuredDiameterMm)) Serial.print(measuredDiameterMm, 3);
    else Serial.print("nan");
    Serial.print(';');
    Serial.println(targetStepsPerS); // das was du sendest
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_CONFIRM_BUTTON, INPUT_PULLDOWN);

  Serial.begin(115200);
  delay(2000);      // Damit Serial genug Zeit zum starten hat

  Wire.begin();    // Sensor auf I2C0 (Pins 4/5)
  Wire.setClock(50000);  // 50 kHz
  Wire.setTimeout(50);

  if(!display.begin(0x3C)){ while(1); }                                                           //Selbstprüfung
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0); display.print("Start..."); display.display();        

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Start...");
  display.display();

  Tlv493dMagnetic3DSensor.begin();
  Serial.println("Starting");                                                               
  delay(2500); 

  if (!loadCalibrationFromFlash()) {
    Serial.println("Keine gueltige Flash-Kalibrierung, nutze Defaults aus dem Code");
    // a und b bleiben bei -1.1563 und 5.9946
  } else {
    Serial.println("Flash-Kalibrierung aktiv");
  }
}

void loop() {
  static unsigned long lastI2cMs = 0;
  //static int i2cFailCount = 0;                // Für Bus Restart
  unsigned long now = millis();

  static unsigned long heartbeatMs = 0;                // LED Heartbeat für debug
  if (millis() - heartbeatMs > 250) {
    heartbeatMs = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  if (!regressionDone) {
    showMessage("Kalibrieren?\nKurz druecken: Nein\nLang Druecken: Ja", 1);

    // Warten bis der Knopf EINMAL gedrueckt wird (LOW -> HIGH)
    while (digitalRead(PIN_CONFIRM_BUTTON) == LOW) {
      delay(50);   
    }

    unsigned long start = millis();
    delay(50);   // Entprellen

    while(digitalRead(PIN_CONFIRM_BUTTON) == HIGH){
      if (millis()-start >= 2000){
        showMessage("Kalibrierung wird\ngestartet", 1);
        delay (2000);
        RunCalibration();
      }
    }
    regressionDone = true;  // Frage nur einmal stellen
  }

  if (!isAutoMode){
    ManMode();
  }else{
    AutoMode();
  }

     // Zentraler I2C-Read: aktualisiert current_rpm/isrunning/mode
  if (now - lastI2cMs >= 200) {
    lastI2cMs = now;
    requestData();
    /*bool ok = requestData();
    if (!ok) {
      i2cFailCount++;
      Serial.print("requestData() FAIL #");
      Serial.println(i2cFailCount);

      if (i2cFailCount >= 3) {
      Serial.println("I2C stuck -> trying bus recovery...");
      bool rec = i2c_bus_recover();
      Serial.println(rec ? "I2C recovery OK" : "I2C recovery FAILED");
      i2cFailCount = 0;
      }
    } else {
    i2cFailCount = 0;*/

    Serial.print("I2C OK: mode=");
    Serial.print(isAutoMode);
    Serial.print(" running=");
    Serial.print(isMotorRunning);
    Serial.print(" speed=");
    Serial.println(CurrentStepsPerS);
    //}
  }
}

// --- Logging für Regelung ---
// ===== cd (welcher Ordner) 
// ===== dir (Was liegt im aktuellen Ornder?)
// ===== cd "Ordnername" (in Ordner wechseln); TAB vervollständigt Ordnername
// ===== cd .. (eine Ebene hoch)
// ===== pio device monitor --baud 115200 > log.txt (Dateierstellung)
// ===== Str + C zum Beenden des Loggens