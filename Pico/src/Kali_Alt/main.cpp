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

#define NUM_CAL_SAMPLES 5
const float calDiameters[NUM_CAL_SAMPLES] = {1.6f, 1.7f, 1.75f, 1.8f, 1.9f};
#define SAMPLE_COUNT 100                                          // 100 Samples pro Messung
#define TRIM_N ((int)(SAMPLE_COUNT * 0.1))                        // !0% der Messwerte getrimmt

// ---------- Globale Variablen für Kalibrierung ----------

float F_vals[NUM_CAL_SAMPLES];    // Sensorwerte (B-Betrag)
float D_vals[NUM_CAL_SAMPLES];    // Referenzdurchmesser
float a = -1.0f, b = 0.0f;        // Koeffizienten der Kalibrierungsfunktion
bool regressionDone = false;


// ---------- Display & Sensor ----------

Adafruit_SH1106G display(128, 64, &Wire);
using namespace ifx::tlx493d;
TLx493D_A1B6 Tlv493dMagnetic3DSensor(Wire, TLx493D_IIC_ADDR_A0_e);

float readMagnet_B_total_filtered() {                                                       
    const int sampleCount = SAMPLE_COUNT;                                                   // sample Count im Moment 100
    double valuesX[sampleCount], valuesY[sampleCount], valuesZ[sampleCount];                // Arrays für x,y,z Achse
    unsigned long start = millis();
    unsigned long stop, dauer;

    digitalWrite(LED_BUILTIN, HIGH);                                                        // LED an

    // Messungen sammeln
    for (int i = 0; i < sampleCount; i++) {
        double x, y, z;

        //--- Vielleicht wieder rein machen?

        // Zeitprüfung: wenn > 2000 ms, abbrechen
/*        if (millis() - start > 40000) {
            digitalWrite(LED_BUILTIN, LOW);
            Serial.println("Messung abgebrochen: Dauer > 40000ms");
            return 0.0; // B=0
        }   */

        if (Tlv493dMagnetic3DSensor.getMagneticFieldAndTemperature(&x, &y, &z, nullptr)) {        // Liefert Sensor Daten?
            valuesX[i] = x;
            valuesY[i] = y;
            valuesZ[i] = z;
        } else {
            valuesX[i] = valuesY[i] = valuesZ[i] = 0;
        }
 
        delay(3);  // Wartezeit zwischen Messungen !! Abfahrt, das sind ja 300ms delay bei 100 Messungen hilfe, ich möchte bitte aus dem Bällebad abgeholt werden
    }

    // Arrays sortieren
    std::sort(valuesX, valuesX + sampleCount);
    std::sort(valuesY, valuesY + sampleCount);
    std::sort(valuesZ, valuesZ + sampleCount);

    // getrimmtes Mittel berechnen
    float sx = 0, sy = 0, sz = 0;
    for (int i = TRIM_N; i < sampleCount - TRIM_N; i++) {
        sx += valuesX[i];
        sy += valuesY[i];
        sz += valuesZ[i];
    }
    int n = sampleCount - 2 * TRIM_N;
    float xm = sx / n, ym = sy / n, zm = sz / n;                              //Gemittelte Magnetfeldkomponenten
    float Z = xm*xm + ym*ym + zm*zm;                                          //Quadrat des Magnetfeldvektors 

    stop = millis();
    dauer = stop - start;

    Serial.print("Dauer: ");
    Serial.print(dauer);
    Serial.print("ms;  B= ");
    Serial.println(Z);

    digitalWrite(LED_BUILTIN, LOW);
    return sqrtf(Z);                                                          //Betrag des Magnetfeldvektors
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
    for (int sampleIndex = 0; sampleIndex < NUM_CAL_SAMPLES; sampleIndex++) {                     //Schleife über Anzahl Kalibrierungspunkte
        float d_real = calDiameters[sampleIndex];

        // 1. Anzeige des aktuellen Prüfstabs
        char buf[20];
        sprintf(buf, "Stab: %.2f mm", d_real);
        Serial.println(buf);
        showMessage(buf);

        // 2. Auf Knopfdruck warten
        while (digitalRead(PIN_CONFIRM) == HIGH) {
            delay(50);  // Entprellen
        }
        while (digitalRead(PIN_CONFIRM) == LOW) {
            delay(50);
        }

        // 3. Messung
        float B = readMagnet_B_total_filtered();
        if (B <= 0) {
            showError("Ungueltiger Sensorwert");
            return;  // Kalibrierung abbrechen wenn B nicht größer 1
        }

        // 4. Messwert speichern
        D_vals[sampleIndex] = d_real;
        F_vals[sampleIndex] = B;
    }

    // 5. Regression durchführen
    berechneLogFunktion();
    regressionDone = true;

    // 6. Erfolgsmeldung
    showMessage("Kalibrierung OK");
    delay(1000);    // für die Lesbarkeit
}


// ---------- Setup & Loop ----------

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_CONFIRM, INPUT_PULLDOWN);           // Knopf and GP18 und 3V3 (nicht ground)

  Serial.begin(115200);
  delay(2000); // Serial hochkommen lassen

  Wire.begin();
  Wire.setClock(50000);  // 50 kHz

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

