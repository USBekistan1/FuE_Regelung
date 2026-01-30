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

float berechneGeschwindigkeit(long stepsProSekunde);
float readMagnet_B_total_filtered();
float B_to_D(float B);
void update_Display_Man(float dIst);
void update_Display_Auto(float dSoll, float dIst, bool rampDone);
void update_Display_Sollwahl(float dSoll);
bool requestData();
void showError(const char* msg);
void showMessage(const char* msg, int TextSize);
void showCalibrationParams(float a, float b);
void modusKalibrierung();
void resetMagnetFilter();
void berechneLogFunktion();
void saveCalibrationToFlash(float aNew, float bNew);
bool loadCalibrationFromFlash();
void ManMode();
void AutoMode();
bool i2c_bus_recover();

//Regelung
int16_t berechneStepsAusDurchmesser(float d_soll);
void sendSpeedToSlave(int16_t speedSteps);


Adafruit_SH1106G display(128, 64, &Wire);                                 // Selber I2C Bus -> Display Updaterate checken!!
using namespace ifx::tlx493d;
TLx493D_A1B6 Tlv493dMagnetic3DSensor(Wire, TLx493D_IIC_ADDR_A0_e);        // Selber Bus (Sensor & Display)

// ----- Kalibrierung -----
#define PIN_CONFIRM 18            // Pin für den Bestätigungs-Knopf

#define NUM_CAL_SAMPLES 9
const float calDiameters[NUM_CAL_SAMPLES] = {1.0f, 1.2f, 1.4f, 1.6f, 1.7f, 1.75f, 1.8f, 1.9f, 2.0f};
float a = -1.1563, b = 5.9946;                                              // Koeffizienten der Kalibrierungsgeraden
float F_vals[NUM_CAL_SAMPLES];    // Sensorwerte (B-Betrag)
float D_vals[NUM_CAL_SAMPLES];    // Referenzdurchmesser
bool regressionDone = false;

// ----- Flash-Speicher für Kalibrierung auf dem Pico -----
#define FLASH_TARGET_OFFSET (256 * 1024)  // 256 kB hinter Programmanfang
#define CALIB_MAGIC 0x4E696C73            // 'Nils' als Magic

struct CalibData {
    uint32_t magic;
    float a;
    float b;
};

// Zeiger auf den Kalibrierbereich im Flash (XIP-Adresse)
const CalibData* const flashCalib = (const CalibData*)(XIP_BASE + FLASH_TARGET_OFFSET);


// ----- EMA -----
const unsigned long SAMPLE_PERIOD_MS = 20;  // 50 Hz

unsigned long emaLastSampleTime = 0;
bool          emaInitialized    = false;
float         emaX = 0.0f, emaY = 0.0f, emaZ = 0.0f;
float         emaLastMagnitude  = 0.0f;



// Weitere Variablen
const int STEPS_PER_REV = 800;                  // Schritte pro Umdrehung
const float DIAMETER = 0.05;                    // Durchmesser der Welle in Metern
const float CIRCUMFERENCE = PI * DIAMETER;      // Umfang in Metern
static unsigned long lastDisp = 0;

// -------- I2C ----------
volatile int16_t targetSpeed = 0;                                         // Zielgeschwindigkeit, 2 Byte
volatile int16_t current_rpm = 0;                                         // Ist-Geschwindigkeit
volatile bool mode = false;                                               // Default Regelbetrieb
volatile bool isrunning = true;                                           // warum volatile? Keine ISR oder? Juckt wahrscheinlich nicht

// --- I2C Bus Recovery (Pico Arduino) ---
static const int I2C_SDA_PIN = 4;
static const int I2C_SCL_PIN = 5;

// ---- Vorsteuerung ----
const float CONST_C = 5.939;       // Prozesskonstante
float sollDurchmesser = 1.75;       // Dein gewünschter Zieldurchmesser in mm
int16_t calculatedSteps = 0;       // Der berechnete Wert für den Arduino
bool rampDone = 0;
static unsigned long pressStart = 0;
enum AutoState : uint8_t { AUTO_SETPOINT, AUTO_FEEDFORWARD, AUTO_REG };
AutoState autoState = AUTO_SETPOINT;
volatile int16_t encDelta = 0;

// ---- Regelung ----
const float REG_KI = 50.0f;
float integralSum = 0.0f;
const float INTEGRAL_MAX = 1000.0f;

// ======== EMA-Filter für TLx493D ========
// Läuft nicht-blockierend, 50 Hz Abtastrate (20 ms Schrittzeit)
// Glättet die Magnetfeldmessung über X/Y/Z und gibt den Betrag zurück.

float readMagnet_B_total_filtered() {
  const float TAU_S = 0.20f;
  const float dt = SAMPLE_PERIOD_MS / 1000.0f;
  const float alpha = 1.0f - expf(-dt / TAU_S);
  const unsigned long now = millis();
   
  static unsigned long lastReinit = 0;
  const unsigned long REINIT_PERIOD_MS = 500;               // Frequenz mit der Sensor neugestartet wird

  if (now - emaLastSampleTime < SAMPLE_PERIOD_MS) {
    return emaLastMagnitude;
  }
  emaLastSampleTime = now;
  double x, y, z;
  
 
  if (Tlv493dMagnetic3DSensor.getMagneticFieldAndTemperature(&x, &y, &z, nullptr)) {
    // FALL: Sensor funktioniert
    if (!emaInitialized) {
      emaX = (float)x; emaY = (float)y; emaZ = (float)z;
      emaInitialized = true;
    } else {
      emaX += alpha * ((float)x - emaX);
      emaY += alpha * ((float)y - emaY);
      emaZ += alpha * ((float)z - emaZ);
    }
    // Betrag berechnen
    emaLastMagnitude = sqrtf(emaX * emaX + emaY * emaY + emaZ * emaZ);
  } 
  else {
    emaInitialized = false;

    if (now - lastReinit > REINIT_PERIOD_MS) {
      lastReinit = now;

      // Erst Bus-Recovery versuchen, dann Sensor neu starten
      bool rec = i2c_bus_recover();
      Serial.println(rec ? "I2C rec OK (sensor)" : "I2C rec FAIL (sensor)");

      Tlv493dMagnetic3DSensor.begin();
    }

    return NAN; 
  }
  return emaLastMagnitude;
}

void resetMagnetFilter() {          //Frische Ema für jeden neuen Stab
  emaInitialized = false;
  emaX = emaY = emaZ = 0.0f;
  emaLastMagnitude = 0.0f;
  emaLastSampleTime = 0;
}

void modusKalibrierung() {
  const unsigned long WARMUP_MS = 500;   // Zeit, damit EMA sich auf neuen Stab einstellt
  const unsigned long SETTLE_MS = 1000;  // Zeit, über die wir für den Mittelwert sampeln
  const unsigned long SAMPLE_DELAY_MS = 20;

  while (digitalRead(PIN_CONFIRM) == HIGH) {
      delay(50);
  }

  for (int sampleIndex = 0; sampleIndex < NUM_CAL_SAMPLES; sampleIndex++) {
    float d_real = calDiameters[sampleIndex];

    // 1. Anzeige des aktuellen Prüfstabs
    char buf[20];
    sprintf(buf, "Stab: %.2f mm", d_real);
    Serial.println(buf);
    showMessage(buf, 2);

    // 2. Auf Knopfdruck warten (INPUT_PULLDOWN + Taster an 3V3)
    // warten bis gedrückt (LOW -> HIGH)
    while (digitalRead(PIN_CONFIRM) == LOW) {
      delay(50);
    }
    // warten bis losgelassen (HIGH -> LOW)
    while (digitalRead(PIN_CONFIRM) == HIGH) {
      delay(50);
    }

    resetMagnetFilter();

    // 3a. WARMUP-PHASE: EMA darf sich auf den neuen Stab einstellen
    unsigned long tStart = millis();
    while (millis() - tStart < WARMUP_MS) {
      (void)readMagnet_B_total_filtered();   // Wert wird verworfen, nur Filter updaten
      delay(SAMPLE_DELAY_MS);
    }

    // 3b. SETTLE-PHASE: jetzt Werte für den Mittelwert einsammeln
    float Bsum = 0.0f;
    int   Bcount = 0;
    tStart = millis();

    while (millis() - tStart < SETTLE_MS) {
      float Bnow = readMagnet_B_total_filtered();
      Bsum += Bnow;
      Bcount++;
      delay(SAMPLE_DELAY_MS);
    }

    float B = (Bcount > 0) ? (Bsum / Bcount) : 0.0f;

    Serial.print("B_avg[");
    Serial.print(sampleIndex);
    Serial.print("] = ");
    Serial.println(B, 6);

    if (B <= 0) {
      showError("Ungueltiger Sensorwert");
      return;
    }

    D_vals[sampleIndex] = d_real;
    F_vals[sampleIndex] = B;
  }

  // 4. Regression durchführen
  berechneLogFunktion();
  regressionDone = true;

  // 5. Parameter anzeigen
  showCalibrationParams(a, b);  // deine Anzeige-Funktion für a,b
  delay(2000);

  // 6. Kalibrierung im Flash speichern
  saveCalibrationToFlash(a, b);
}

void berechneLogFunktion() {
  float sumLnB = 0.0f, sumD = 0.0f, sumLnB2 = 0.0f, sumLnB_D = 0.0f;
  int nEff = 0;

  for (int i = 0; i < NUM_CAL_SAMPLES; i++) {
    if (F_vals[i] <= 0) continue;
    float lnB = log(F_vals[i]);
    float D = D_vals[i];

    sumLnB   += lnB;
    sumD     += D;
    sumLnB2  += lnB * lnB;
    sumLnB_D += lnB * D;
    nEff++;
  }

  float denom = nEff * sumLnB2 - sumLnB * sumLnB;
    if (denom != 0 && nEff > 0) {
    a = (nEff * sumLnB_D - sumLnB * sumD) / denom;
    b = (sumD - a * sumLnB) / nEff;

    Serial.print("Kalibrierung fertig. a = ");
    Serial.print(a, 6);
    Serial.print(" , b = ");
    Serial.println(b, 6);

  } else {
    showError("Regression fehlgeschlagen");
  }
}

bool loadCalibrationFromFlash() {
    if (flashCalib->magic != CALIB_MAGIC) {
        Serial.println("Flash-Kalibrierung: kein gueltiger Eintrag");
        return false;
    }

    a = flashCalib->a;
    b = flashCalib->b;

    Serial.print("Flash-Kalibrierung geladen: a=");
    Serial.print(a, 6);
    Serial.print(" , b=");
    Serial.println(b, 6);

    return true;
}

void saveCalibrationToFlash(float aNew, float bNew) {
    CalibData data;
    data.magic = CALIB_MAGIC;
    data.a = aNew;
    data.b = bNew;

    uint8_t buf[FLASH_PAGE_SIZE];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &data, sizeof(data));

    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, buf, FLASH_PAGE_SIZE);
    restore_interrupts(irq_state);

    Serial.println("Kalibrierung im Flash gespeichert");
}

void update_Display_Man(float dIst) {
  float speed_m_per_min = berechneGeschwindigkeit(current_rpm);  
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
  display.println(dIst);
  display.println(" mm");

  display.display();
  //delay(100);                                                                //mehr delay, warum?
}

void update_Display_Auto(float dSoll, float dIst, bool rampDone){
  float speed_m_per_min = berechneGeschwindigkeit(current_rpm);  

  display.clearDisplay();
  display.setTextSize(1.95);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);

  display.print("Soll: ");
  display.print(dSoll, 2);
  display.println(" mm");

  display.print("Ist : ");
  display.print(dIst, 2);
  display.println(" mm");

  display.print("Speed: ");
  display.print(speed_m_per_min, 1);
  display.println(" m/min");

  display.print("AUTO: ");
  display.println(rampDone ? "Regler aktiv" : "Vorsteuerung -> Knopf drücken wenn Durchm. stabil");

  display.display();
}

void update_Display_Sollwahl(float dSoll){
  display.clearDisplay();
  display.setTextSize(1.95);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);

  display.print("Soll: ");
  display.print(dSoll, 2);
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

void showMessage(const char* msg, int TextSize) {
  display.clearDisplay();
  display.setTextSize(TextSize);
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

    // Master fragt 4 Bytes beim Uno (Adresse 0x08) an
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

    current_rpm = speed;
    isrunning   = running;
    mode        = modeVal;
    encDelta    = delta;

    return true;
}

bool i2c_bus_recover() {
  // 1) I2C "stoppen" – je nach Core gibt's kein Wire.end()
  // Wir machen stattdessen GPIO-Recovery direkt.

  // 2) Pins als GPIO open-drain-like: INPUT_PULLUP (High über Pullup)
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  // Wenn SCL LOW ist, hält jemand den Clock low -> schwerer Fall (z.B. Slave kaputt)
  if (digitalRead(I2C_SCL_PIN) == LOW) {
    return false;
  }

  // 3) Wenn SDA LOW: versuche, den Slave "auszutakten"
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

  // 4) STOP condition erzwingen: SDA LOW -> SCL HIGH -> SDA HIGH
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);

  pinMode(I2C_SCL_PIN, INPUT_PULLUP); // SCL HIGH (über Pullup)
  delayMicroseconds(5);

  pinMode(I2C_SDA_PIN, INPUT_PULLUP); // SDA HIGH (über Pullup) -> STOP
  delayMicroseconds(5);

  // 5) I2C wieder initialisieren
  Wire.begin();
  Wire.setClock(50000);     // zum Testen erstmal langsam (50 kHz)
  Wire.setTimeout(50);      // hast du schon, passt

  // Erfolg, wenn beide Linien HIGH sind
  return (digitalRead(I2C_SDA_PIN) == HIGH) && (digitalRead(I2C_SCL_PIN) == HIGH);
}

float berechneGeschwindigkeit(long stepsProSekunde) {
  float revsPerSec = (float)stepsProSekunde / STEPS_PER_REV;  // Umdrehungen pro Sekunde
  float revsPerMin = revsPerSec * 60.0;                       // U/min
  float v_mPerMin = CIRCUMFERENCE * revsPerMin;               // m/min
  return v_mPerMin;
}

void ManMode(){
  unsigned long now = millis();
  autoState   = AUTO_SETPOINT;
  rampDone    = false;
  integralSum = 0.0f;
  pressStart  = 0;  

  // --- Magnetfeld -> Durchmesser ---
  float B = readMagnet_B_total_filtered();
  float D = NAN;
  if (B > 1e-6f && isfinite(B)) {
    D = a * logf(B) + b;
  }

  // --- Anzeige + Serial alle 200 ms ---
  static unsigned long lastPrintTime = 0;
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;

    update_Display_Man(D);

    float t_s = now / 1000.0f;                  // Zeit in Sekunden
    float v_m_per_min = berechneGeschwindigkeit(current_rpm);

    Serial.print(t_s, 3);        
    Serial.print(';');
    Serial.print(v_m_per_min, 3); 
    Serial.print(';');
    Serial.println(D, 3);        
  }
}

int16_t berechneStepsAusDurchmesser(float d_soll) {
   if (d_soll <= 0.1f) return 0;         // Schutz vor Division durch 0 oder Unsinn

  // 1. Prozessgeschwindigkeit berechnen (v = (C/d)^2)
  float v_soll_m_min = powf(CONST_C / d_soll, 2.0f);

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

void sendSpeedToSlave(int16_t speedSteps) {
  Wire.beginTransmission(0x08);
  // Wir senden den int16 in zwei Bytes (High Byte, Low Byte)
  Wire.write((uint8_t)((speedSteps >> 8) & 0xFF)); 
  Wire.write((uint8_t)(speedSteps & 0xFF));
  Wire.endTransmission();
}

void AutoMode() {
  unsigned long now = millis();
  static unsigned long lastWrite = 0;
  const unsigned long WRITE_PERIOD_MS = 20000;

  // --- 0) Sollwert Einstellen
  if (autoState == AUTO_SETPOINT) {
    if (encDelta != 0) {
      sollDurchmesser += 0.01f * (float)encDelta;

      // Limits setzen
      if (sollDurchmesser < 1.50f) sollDurchmesser = 1.50f;
      if (sollDurchmesser > 2.20f) sollDurchmesser = 2.20f;

      encDelta = 0;   // WICHTIG: Delta "verbrauchen"
    }
    // Anzeige: SETPOINT
    // (baue dir gern "AUTO: Sollwahl" ins Display)
    update_Display_Sollwahl(sollDurchmesser);

    // Bestätigen per Hold-to-confirm
    if (digitalRead(PIN_CONFIRM) == HIGH) {
      if (pressStart == 0) pressStart = now;
        if (now - pressStart >= 300) {
          pressStart  = 0;
          integralSum = 0.0f;
          rampDone    = false;                 // Regler noch AUS
          autoState   = AUTO_FEEDFORWARD;      // weiter: Vorsteuerung-only
        }
    } else {
      pressStart = 0;
    }

    return; // im SETPOINT-State nichts senden/regeln
  }

  // --- 1) Durchmesser messen (mit Schutz gegen log(<=0)) ---
  float B = readMagnet_B_total_filtered();
  float dIst = NAN;
  if (B > 1e-6f && isfinite(B)) {
  dIst = a * logf(B) + b;
  }

  // --- 2) Vorsteuerung berechnen und an Arduino senden (WRITE) ---
  if (!rampDone){
    if (now - lastWrite >= WRITE_PERIOD_MS) {
      lastWrite = now;

      int16_t cmdSteps = 0;

      // Nur senden, wenn running + gültiger Sensorwert
      if (isrunning && !isnan(dIst) && isfinite(dIst)) {
        cmdSteps = berechneStepsAusDurchmesser(sollDurchmesser);
      } else {
        cmdSteps = 0;
      }

      targetSpeed = cmdSteps;
      sendSpeedToSlave(cmdSteps);
    }
  }

  if (!rampDone) {
    if (digitalRead(PIN_CONFIRM) == HIGH) {
      if (pressStart == 0) pressStart = millis();          // Startzeit merken
        if (millis() - pressStart >= 300) {
          rampDone   = true;
          integralSum = 0.0f;                               // sauberer Übergang
          pressStart  = 0;                                  // reset (optional)
        }
      } else {
        pressStart = 0;                                       // losgelassen → zurücksetzen
      }
  } else {
    pressStart = 0;                                           // wenn schon aktiv, Timer sauber halten
  }

  // ---- 3) Regelung ----
  if(rampDone){
    if (now - lastWrite >= WRITE_PERIOD_MS) {
      // Delta-Zeit (dt) berechnen, damit das Integral mathematisch korrekt ist (Fehler * Zeit)
      float dt = (now - lastWrite) / 1000.0f; 
      lastWrite = now;

      int16_t cmdSteps = 0;

      // Sicherheitsabfrage: Regeln wir nur, wenn der Motor läuft und der Sensor OK ist
      if (isrunning && !isnan(dIst) && isfinite(dIst)) {
      
        // A) Vorsteuerung (Feedforward): Die Basis-Geschwindigkeit aus dem Modell
        int16_t steps_feedforward = berechneStepsAusDurchmesser(sollDurchmesser);
      
        // B) I-Anteil berechnen (Feedback)
        // Fehler berechnen: Ist > Soll (zu dick) -> Fehler positiv -> Wir müssen schneller werden
        float abweichung = dIst - sollDurchmesser;
      
        // Integral bilden: Alter Wert + (Fehler * vergangene Zeit)
        integralSum += abweichung * dt;

        // !!! Anti-Windup !!!
        // Begrenzt den Speicher, damit er nicht "überläuft", wenn der Fehler lange besteht.
        if (integralSum > INTEGRAL_MAX) integralSum = INTEGRAL_MAX;
        if (integralSum < -INTEGRAL_MAX) integralSum = -INTEGRAL_MAX;

        // Den I-Anteil in Steps umrechnen
        float steps_correction = integralSum * REG_KI;

        // C) Zusammenführen: Gesamtspeed = Modellwert + Korrekturwert
        cmdSteps = steps_feedforward + (int16_t)steps_correction;

        // Grenzwerte des Motors einhalten
        if (cmdSteps < 0) cmdSteps = 0;
        if (cmdSteps > 3000) cmdSteps = 3000;

        } else {
          // WICHTIG: Wenn Motor aus ist, Integral resetten!
          // Sonst merkt er sich den "Fehler" vom Stillstand und rast beim Starten los.
          cmdSteps = 0;
          integralSum = 0.0f;
        }

      targetSpeed = cmdSteps;
      sendSpeedToSlave(cmdSteps);
    }
  }


  // --- 4) Anzeige ---
  if (millis() - lastDisp >= 150){
    lastDisp = millis();
    update_Display_Auto(sollDurchmesser, dIst, rampDone);
  }


  // --- 5) Logging (optional, passt super fürs Tuning) ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 200) {
    lastPrint = now;

    float t_s = now / 1000.0f;
    float v_m_per_min = berechneGeschwindigkeit(current_rpm);

    Serial.print(t_s, 3);
    Serial.print(';');
    Serial.print(v_m_per_min, 3);
    Serial.print(';');
    if (!isnan(dIst) && isfinite(dIst)) Serial.print(dIst, 3);
    else Serial.print("nan");
    Serial.print(';');
    Serial.println(targetSpeed); // das was du sendest
  }
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_CONFIRM, INPUT_PULLDOWN);

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
  static unsigned long lastI2C = 0;
  static int i2cFailCount = 0;                // Für Bus Restart
  unsigned long now = millis();

  if (!regressionDone) {
    showMessage("Kalibrieren?\nKurz druecken: Nein\nLang Druecken: Ja", 1);

    // Warten bis der Knopf EINMAL gedrueckt wird (LOW -> HIGH)
    while (digitalRead(PIN_CONFIRM) == LOW) {
      delay(50);   
    }

    unsigned long start = millis();
    delay(50);   // Entprellen

    while(digitalRead(PIN_CONFIRM) == HIGH){
      if (millis()-start >= 2000){
        showMessage("Kalibrierung wird\ngestartet", 1);
        delay (2000);
        modusKalibrierung();
      }
    }
    regressionDone = true;  // Frage nur einmal stellen
  }

  if (!mode){
    ManMode();
  }else{
    AutoMode();
  }

     // Zentraler I2C-Read: aktualisiert current_rpm/isrunning/mode
  if (now - lastI2C >= 200) {
    lastI2C = now;

    bool ok = requestData();
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
    i2cFailCount = 0;

    Serial.print("I2C OK: mode=");
    Serial.print(mode);
    Serial.print(" running=");
    Serial.print(isrunning);
    Serial.print(" speed=");
    Serial.println(current_rpm);
    }
  }
}


// --- Logging für Regelung ---
// ===== cd (welcher Ordner) 
// ===== dir (Was liegt im aktuellen Ornder?)
// ===== cd "Ordnername" (in Ordner wechseln); TAB vervollständigt Ordnername
// ===== cd .. (eine Ebene hoch)
// ===== pio device monitor --baud 115200 > log.txt (Dateierstellung)
// ===== Str + C zum Beenden des Loggens