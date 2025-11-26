#include <Wire.h>                                                 //I2C
#include <Adafruit_GFX.h>                                         //OLED
#include <Adafruit_SH110X.h>                                      //OLED
#include "TLx493D_inc.hpp"                                        //Hall Sensor
#include <math.h>
#include <algorithm>

const int STEPS_PER_REV = 800;                  // Schritte pro Umdrehung
const float DIAMETER = 0.05;                    // Durchmesser der Welle in Metern
const float CIRCUMFERENCE = PI * DIAMETER;      // Umfang in Metern

// -------- I2C ----------
volatile int16_t targetSpeed = 0;                                         // Zielgeschwindigkeit, 2 Byte
volatile int16_t current_rpm = 0;                                         // Ist-Geschwindigkeit
volatile bool mode = false;                                               // Default Regelbetrieb
volatile bool isrunning = true;               

#ifndef LED_BUILTIN                                               // Fehlt in der Library scheinbar
#define LED_BUILTIN 25
#endif

#define PIN_CONFIRM 18                                            // Pin für den Bestätigungs-Knopf

#define NUM_CAL_SAMPLES 5
#define SAMPLE_COUNT 100                                          // 100 Samples pro Messung
#define TRIM_N ((int)(SAMPLE_COUNT * 0.1))                        // !0% der Messwerte getrimmt
#define RPM_MAX 3000.0f                                           // MaxSpeed Motor
#define TARGET_DIAMETER_MM 1.75f                                  // Zieldurchmesser

//Variablen für Timer
unsigned long stableStartTime = 0;
const float STABLE_TOLERANCE = 0.04f;                             // Toleranz für Dicke
const unsigned long STABLE_DURATION_MS = 30000;                   // 30 Sekunden
bool stable = false;

//Kalibrierung-Prüfstifftgröße
const float calDiameters[] = {1.6f, 1.7f, 1.75f, 1.8f, 1.9f};

Adafruit_SH1106G display(128, 64, &Wire);                                 // Selber I2C Bus -> Display Updaterate checken!!
using namespace ifx::tlx493d;
TLx493D_A1B6 Tlv493dMagnetic3DSensor(Wire, TLx493D_IIC_ADDR_A0_e);        // Selber Bus (Sensor & Display)


float readMagnet_B_total_filtered() {
  static unsigned long lastSampleTime = 0;                // letzter Messzeitpunkt
  static bool initialized = false;                        // erster Durchlauf?
  static float emaX = 0.0f, emaY = 0.0f, emaZ = 0.0f;     // EMA-Zwischenspeicher
  static float lastMagnitude = 0.0f;                      // zuletzt berechneter Betrag


  const unsigned long SAMPLE_PERIOD_MS = 20;              // 50 Hz
  const float TAU_S = 0.20f;                              // Zeitkonstante -> Größer heißt Filter träger; Macht das ganze zeitabhängig und nicht abhängig von Anzahl Messungen
  const float dt = SAMPLE_PERIOD_MS / 1000.0f;
  const float alpha = 1.0f - expf(-dt / TAU_S);           // Wie stark verdrängt der neue Messwert den alten Wert?

  const unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_PERIOD_MS) {
    // Noch kein neues Sample fällig → letzten Wert zurückgeben
    return lastMagnitude;
  }
  lastSampleTime = now;

  // --- Sensor auslesen (wie in deinem bestehenden Code) ---
  double x, y, z;                                                                         // loke Variablen für Sensor (jede Achse)
  if (Tlv493dMagnetic3DSensor.getMagneticFieldAndTemperature(&x, &y, &z, nullptr)) {
    // --- EMA-Update auf Achsenebene ---
    if (!initialized) {                                                                   // erster durchlauf -> ungeglättete Werte
      emaX = (float)x; emaY = (float)y; emaZ = (float)z;
      initialized = true;
    } else {
      emaX += alpha * ((float)x - emaX);
      emaY += alpha * ((float)y - emaY);
      emaZ += alpha * ((float)z - emaZ);
    }
  }
  // Falls der Read fehlschlägt, behalten wir die alten EMA-Werte bei.

  // --- Betrag berechnen und zwischenspeichern ---
  lastMagnitude = sqrtf(emaX * emaX + emaY * emaY + emaZ * emaZ);
  if (lastMagnitude < 1e-6f) lastMagnitude = 1e-6f;   // nie 0 zurückgeben
  return lastMagnitude;
}

void showStatus(float dSet,float dMeas,const char* modeTxt){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.print(modeTxt);
  display.setCursor(0,12);
  display.print("Soll D: "); display.print(dSet,2); display.println(" mm");
  display.setCursor(0,24);
  display.print("Ist  D: ");
  if(isnan(dMeas)) display.println("-");
  else { display.print(dMeas,2); display.println(" mm"); }
  display.setCursor(0,36);
  if (stable) {
    display.println("Stabil");
  } else {
    display.println(" ");
  }

  display.display();
}

void berechneLogFunktion(){
  float sumLnB=0,sumD=0,sumLnB2=0,sumLnB_D=0;
  int nEff=0;
  for(int i=0;i<NUM_CAL_SAMPLES;i++){
    if(F_vals[i]<=0) continue;
    float lnB=log(F_vals[i]);
    float D=D_vals[i];
    sumLnB+=lnB; sumD+=D; sumLnB2+=lnB*lnB; sumLnB_D+=lnB*D;
    nEff++;
  }
  float denom = nEff*sumLnB2 - sumLnB*sumLnB;
  if(denom!=0){
    a=(nEff*sumLnB_D - sumLnB*sumD)/denom;
    b=(sumD - a*sumLnB)/nEff;
    Serial.print("a= ");
    Serial.print(a);
    Serial.print(" b= ");
    Serial.println(b);
  }
}


//--- Fehleranzeige
void showError(const char* msg){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.println("!! FEHLER !!");
  display.println(msg);
  display.display();
  Serial.println(msg);
  delay(2000); // Damit der Error lesbar ist
}

//--- beliebige Anzeige
void showMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);              // größere Schrift für Status
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 20);            // zentriert in etwa
    display.println(msg);
    display.display();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(2000);      // Damit Serial genug Zeit zum starten hat

  Serial.println("t_s;v_m_per_min;D_mm");            // CSV-Header: Zeit [s]; Geschwindigkeit [m/min]; Durchmesser [mm]

  Wire.begin();    // Sensor auf I2C0 (Pins 4/5)
  Wire.setClock(100000);  // 100 kHz

  if(!display.begin(0x3C)){ while(1); }                                                           //Selbstprüfung
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0); display.print("Start..."); display.display();        

  Tlv493dMagnetic3DSensor.begin();
  Serial.println("Starting");                                                               
  delay(2500); 

}


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