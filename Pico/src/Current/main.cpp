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
void showError(const char* msg);
void showMessage(const char* msg, int TextSize);
void showCalibrationParams(float a, float b);
void modusKalibrierung();
void resetMagnetFilter();
void berechneLogFunktion();


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

// -------- I2C ----------
volatile int16_t targetSpeed = 0;                                         // Zielgeschwindigkeit, 2 Byte
volatile int16_t current_rpm = 0;                                         // Ist-Geschwindigkeit
volatile bool mode = false;                                               // Default Regelbetrieb
volatile bool isrunning = true;                                           // warum volatile? Keine ISR oder? Juckt wahrscheinlich nicht




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
  pinMode(PIN_CONFIRM, INPUT_PULLDOWN);

  Serial.begin(115200);
  delay(2000);      // Damit Serial genug Zeit zum starten hat

  Wire.begin();    // Sensor auf I2C0 (Pins 4/5)
  Wire.setClock(100000);  // 100 kHz
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

}

// -------- Loop ----------
void loop() {
    unsigned long now = millis();

    if (!regressionDone) {
    showMessage("Kalibrieren?\nKurz drücken: Nein\nLang Drücken: Ja", 1);

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

    // --- I2C: alle 50 ms den Uno abfragen ---
    static unsigned long lastI2C = 0;
    if (now - lastI2C >= 200) {
        lastI2C = now;
        requestData();                   // aktualisiert current_rpm
    }

    // --- Magnetfeld -> Durchmesser ---
    float D = a * log(readMagnet_B_total_filtered()) + b;

    // --- Anzeige + Serial alle 500 ms ---
    static unsigned long lastPrintTime = 0;
    if (now - lastPrintTime >= 200) {
        lastPrintTime = now;

        update_Display(D);

        // --- Logging für Regelung ---
        // ===== cd (welcher Ordner) 
        // ===== dir (Was liegt im aktuellen Ornder?)
        // ===== cd "Ordnername" (in Ordner wechseln); TAB vervollständigt Ordnername
        // ===== cd .. (eine Ebene hoch)
        // ===== pio device monitor --baud 115200 > Mess1.txt (Dateierstellung)
        // ===== Str + C zum Beenden des Loggens


        float t_s = now / 1000.0f;                  // Zeit in Sekunden
        float v_m_per_min = berechneGeschwindigkeit(current_rpm);

        Serial.print(t_s, 3);        
        Serial.print(';');
        Serial.print(v_m_per_min, 3); 
        Serial.print(';');
        Serial.println(D, 3);         
    }
}
