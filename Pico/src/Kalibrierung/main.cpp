#include <Wire.h>                 // I2C
#include <Adafruit_GFX.h>         // OLED
#include <Adafruit_SH110X.h>      // OLED
#include "TLx493D_inc.hpp"        // Hall Sensor
#include <math.h>
#include <algorithm>

#ifndef LED_BUILTIN               // Falls in der Pico-Umgebung nicht definiert
#define LED_BUILTIN 25
#endif

// ---------- Konfiguration ----------

#define PIN_CONFIRM 18            // Pin für den Bestätigungs-Knopf

#define NUM_CAL_SAMPLES 9
const float calDiameters[NUM_CAL_SAMPLES] = {1.0f, 1.2f, 1.4f, 1.6f, 1.7f, 1.75f, 1.8f, 1.9f, 2.0f};

// ---------- Globale Variablen für Kalibrierung ----------

float F_vals[NUM_CAL_SAMPLES];    // Sensorwerte (B-Betrag)
float D_vals[NUM_CAL_SAMPLES];    // Referenzdurchmesser
float a = -1.0f, b = 0.0f;        // Koeffizienten der Kalibrierungsfunktion
bool regressionDone = false;

// ======== EMA-State (global) ========
const unsigned long SAMPLE_PERIOD_MS = 20;  // 50 Hz

unsigned long emaLastSampleTime = 0;
bool          emaInitialized    = false;
float         emaX = 0.0f, emaY = 0.0f, emaZ = 0.0f;
float         emaLastMagnitude  = 0.0f;


// ---------- Display & Sensor ----------

Adafruit_SH1106G display(128, 64, &Wire);
using namespace ifx::tlx493d;
TLx493D_A1B6 Tlv493dMagnetic3DSensor(Wire, TLx493D_IIC_ADDR_A0_e);

// ======== EMA-Filter für TLx493D ========
// Läuft nicht-blockierend, 50 Hz Abtastrate (20 ms Schrittzeit)
// Glättet die Magnetfeldmessung über X/Y/Z und gibt den Betrag zurück.

float readMagnet_B_total_filtered() {
  const float TAU_S = 0.20f;                              // Zeitkonstante -> Größer heißt Filter träger; Macht das ganze zeitabhängig und nicht abhängig von Anzahl Messungen
  const float dt = SAMPLE_PERIOD_MS / 1000.0f;
  const float alpha = 1.0f - expf(-dt / TAU_S);           // Wie stark verdrängt der neue Messwert den alten Wert?

  const unsigned long now = millis();
  if (now - emaLastSampleTime < SAMPLE_PERIOD_MS) {
    // Noch kein neues Sample fällig → letzten Wert zurückgeben
    return emaLastMagnitude;
  }
  emaLastSampleTime = now;

  // --- Sensor auslesen (wie in deinem bestehenden Code) ---
  double x, y, z;                                                                         // loke Variablen für Sensor (jede Achse)
  if (Tlv493dMagnetic3DSensor.getMagneticFieldAndTemperature(&x, &y, &z, nullptr)) {
    // --- EMA-Update auf Achsenebene ---
    if (!emaInitialized) {                                                                   // erster durchlauf -> ungeglättete Werte
      emaX = (float)x; emaY = (float)y; emaZ = (float)z;
      emaInitialized = true;
    } else {
      emaX += alpha * ((float)x - emaX);
      emaY += alpha * ((float)y - emaY);
      emaZ += alpha * ((float)z - emaZ);
    }
  }
  // Falls der Read fehlschlägt, behalten wir die alten EMA-Werte bei.

  // --- Betrag berechnen und zwischenspeichern ---
  emaLastMagnitude = sqrtf(emaX * emaX + emaY * emaY + emaZ * emaZ);
  return emaLastMagnitude;
}

void resetMagnetFilter() {          //Frische Ema für jeden neuen Stab
  emaInitialized = false;
  emaX = emaY = emaZ = 0.0f;
  emaLastMagnitude = 0.0f;
  emaLastSampleTime = 0.0f;
}

// ---------- Anzeige-Helfer ----------

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

void showMessage(const char* msg) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 20);
  display.println(msg);
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


// ---------- Regression ----------

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

// ---------- Kalibrierablauf ----------

void modusKalibrierung() {
  const unsigned long WARMUP_MS = 500;   // Zeit, damit EMA sich auf neuen Stab einstellt
  const unsigned long SETTLE_MS = 1000;  // Zeit, über die wir für den Mittelwert sampeln
  const unsigned long SAMPLE_DELAY_MS = 20;

  for (int sampleIndex = 0; sampleIndex < NUM_CAL_SAMPLES; sampleIndex++) {
    float d_real = calDiameters[sampleIndex];

    // 1. Anzeige des aktuellen Prüfstabs
    char buf[20];
    sprintf(buf, "Stab: %.2f mm", d_real);
    Serial.println(buf);
    showMessage(buf);

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
}


// ---------- Setup & Loop ----------

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_CONFIRM, INPUT_PULLDOWN);           // Knopf and GP18 und 3V3 (nicht ground)

  Serial.begin(115200);
  delay(2000); // Serial hochkommen lassen

  Wire.begin();
  Wire.setClock(500000);  // 50 kHz

  if (!display.begin(0x3C)) {
    // Wenn das Display nicht startet: LED blinken
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Start...");
  display.display();

  Tlv493dMagnetic3DSensor.begin();
  Serial.println("Starting Kalibrierung...");
  delay(1000);

  showMessage("Bereit");
}

void loop() {
  static bool done = false;

  if (!done) {
    modusKalibrierung();   // Einmal komplette Kalibrierung durchführen
    done = true;

    Serial.println("---- Ergebnis ----");
    Serial.print("a = ");
    Serial.println(a, 6);
    Serial.print("b = ");
    Serial.println(b, 6);
    Serial.println("------------------");

    showCalibrationParams(a, b);
  }

  // Optisches Lebenszeichen: LED blinkt langsam
  digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
}

