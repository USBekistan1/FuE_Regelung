#include <Wire.h>                                                 //I2C
#include <Adafruit_GFX.h>                                         //OLED
#include <Adafruit_SH110X.h>                                      //OLED
#include "TLx493D_inc.hpp"                                        //Hall Sensor
#include <math.h>
#include <algorithm>

// float a = -1.07f, b = 5.67f;                                              // Koeffizienten der Kalibrierungsgeraden (oiginal)

//float a = -0.8932, b = 6.3685;                                            // Mittel aus den drei Werten der ersten Kalibrierung

//float a = -0.2624, b = 2.7512;

//float a = -0.8962, b = 5.0466;                                          

//float a = -0.9242, b = 5.1471;                                        //Interpolation (nicht gut)

//float a = -0.9672, b = 5.2892;                                          //perfekt zwischen 1,6mm und 1,7mm

//float a = -1.2415, b = 6.2856;                                            //alte Logik, warum anders?

float a = -1.1563, b = 5.9946;                                          // 9 Messpunkte, sehr gut zwischen 1-2mm

//float a = -1.0608, b = 5.6301;                                          // Auch ohne Stab kalibriert -> Größere Abweichungen zwischen 1 und 2mm                             

unsigned long now =0; 

Adafruit_SH1106G display(128, 64, &Wire);                                 // Selber I2C Bus -> Display Updaterate checken!!
using namespace ifx::tlx493d;
TLx493D_A1B6 Tlv493dMagnetic3DSensor(Wire, TLx493D_IIC_ADDR_A0_e);        // Selber Bus (Sensor & Display)


// ======== EMA-Filter für TLx493D ========
// Läuft nicht-blockierend, 50 Hz Abtastrate (20 ms Schrittzeit)
// Glättet die Magnetfeldmessung über X/Y/Z und gibt den Betrag zurück.

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

void update_Display(float dIst) {
  display.clearDisplay();
    // Display statischen Text anzeigen
  display.setTextSize(1.95);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,24);
  display.print("Ist-Durchm.: ");
  display.setCursor(0,36);
  display.print(dIst);
  display.println(" mm");

  display.display();
  //delay(100);                                                                //mehr delay, warum?
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(2000);      // Damit Serial genug Zeit zum starten hat
  Wire.begin();    // Sensor auf I2C0 (Pins 4/5)
  Wire.setClock(10000);  // 50 kHz

   if(!display.begin(0x3C)){ while(1); }                                                           //Selbstprüfung
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0); display.print("Start..."); display.display();        

  Tlv493dMagnetic3DSensor.begin();
  Serial.println("Starting");                                                               
  delay(2500); 

}

float D= 0;
static unsigned long lastPrintTime = 0;

// -------- Loop ----------
void loop() {
      now = millis();                                             

    // Wert immer berechnen (Sensor-EMA läuft intern mit 50 Hz)
    D = a * log(readMagnet_B_total_filtered()) + b;

    // Nur alle 500 ms einmal auf Serial ausgeben -> 2 Hz
    if (now - lastPrintTime >= 500) {
        lastPrintTime = now;
        update_Display(D);
        Serial.println(D);   // oder Serial.print("D = "); Serial.println(D);
    }
}
