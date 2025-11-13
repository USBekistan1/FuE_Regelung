#include <Wire.h>                                                 //I2C
#include <Adafruit_GFX.h>                                         //OLED
#include <Adafruit_SH110X.h>                                      //OLED
#include "TLx493D_inc.hpp"                                        //Hall Sensor
#include <math.h>
#include <algorithm>

float berechneGeschwindigkeit(long stepsProSekunde);
float readMagnet_B_total_filtered();
float B_to_D(float B);
void update_Display(float dIst);
bool requestData();

float a = -1.07f, b = 5.67f;                                              // Koeffizienten der Kalibrierungsgeraden

Adafruit_SH1106G display(128, 64, &Wire);                                 // Selber I2C Bus -> Display Updaterate checken!!
using namespace ifx::tlx493d;
TLx493D_A1B6 Tlv493dMagnetic3DSensor(Wire, TLx493D_IIC_ADDR_A0_e);        // Selber Bus (Sensor & Display)


// Weitere Variablen
const int STEPS_PER_REV = 800;                  // Schritte pro Umdrehung
const float DIAMETER = 0.05;                    // Durchmesser der Welle in Metern
const float CIRCUMFERENCE = PI * DIAMETER;      // Umfang in Metern

// -------- I2C ----------
volatile int16_t targetSpeed = 0;                                         // Zielgeschwindigkeit, 2 Byte
volatile int16_t current_rpm = 0;                                         // Ist-Geschwindigkeit
volatile bool mode = false;                                               // Default Regelbetrieb
volatile bool isrunning = true;                                           // warum volatile? Keine ISR oder? Juckt wahrscheinlich nicht


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
  float speed_m_per_min = berechneGeschwindigkeit(current_rpm);  
  display.setCursor(0, 0);
  display.print("Speed: ");
  display.setCursor(0, 12);
  display.print(speed_m_per_min, 1);
  display.print(" m/min");
  display.setCursor(0,24);
  display.print("Ist-Durchm.: ");
  display.setCursor(0,36);
  display.print(dIst);
  display.println(" mm");

  display.display();
  //delay(100);                                                                //mehr delay, warum?
}

bool requestData() {
    const uint8_t expected = 4;

    // Master fragt 4 Bytes beim Uno (Adresse 0x08) an
    uint8_t received = Wire.requestFrom(0x08, expected);  

    if (received != expected) {
        Serial.print("I2C Fehler: erwartet 4, bekommen ");
        Serial.println(received);

        // Buffer leeren, falls Schrott drin ist
        while (Wire.available()) Wire.read();
        return false;
    }

    int16_t speed   = (Wire.read() << 8) | Wire.read();
    bool running    = Wire.read();
    bool modeVal    = Wire.read();

    current_rpm = speed;
    isrunning   = running;
    mode        = modeVal;

    return true;
}


float berechneGeschwindigkeit(long stepsProSekunde) {
  float revsPerSec = (float)stepsProSekunde / STEPS_PER_REV;  // Umdrehungen pro Sekunde
  float revsPerMin = revsPerSec * 60.0;                       // U/min
  float v_mPerMin = CIRCUMFERENCE * revsPerMin;               // m/min
  return v_mPerMin;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(2000);      // Damit Serial genug Zeit zum starten hat
  Wire.begin();    // Sensor auf I2C0 (Pins 4/5)
  Wire.setClock(100000);  // 100 kHz

   if(!display.begin(0x3C)){ while(1); }                                                           //Selbstprüfung
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0); display.print("Start..."); display.display();        

  Tlv493dMagnetic3DSensor.begin();
  Serial.println("Starting");                                                               
  delay(2500); 

}

// -------- Loop ----------
void loop() {
    unsigned long now = millis();

    // --- I2C: alle 50 ms den Uno abfragen ---
    static unsigned long lastI2C = 0;
    if (now - lastI2C >= 50) {
        lastI2C = now;
        requestData();                   // aktualisiert current_rpm
    }

    // --- Magnetfeld -> Durchmesser ---
    float D = a * log(readMagnet_B_total_filtered()) + b;

    // --- Anzeige + Serial alle 500 ms ---
    static unsigned long lastPrintTime = 0;
    if (now - lastPrintTime >= 500) {
        lastPrintTime = now;

        update_Display(D);

        Serial.print("D = ");
        Serial.print(D);
        Serial.print(" mm, v = ");
        Serial.print(berechneGeschwindigkeit(current_rpm));
        Serial.println(" m/min");
    }
}

