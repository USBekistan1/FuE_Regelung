/*
  Raspberry Pi Pico - I2C-Master für Arduino Slave
  Funktionen:
  - Liefert targetSpeed per I2C an Arduino
  - Modus (0=Kalibrierung,1=PWM,2=Regel) wird vom Arduino gesetzt
  - Fehler werden auf OLED angezeigt
*/

#include <Wire.h>                                                 //I2C
#include <Adafruit_GFX.h>                                         //OLED
#include <Adafruit_SH110X.h>                                      //OLED
#include "TLx493D_inc.hpp"                                        //Hall Sensor
#include <math.h>
#include <algorithm>

// ---- Funktionsdeklarierungen ----
float berechneGeschwindigkeit(long stepsProSekunde);
float berechneTotzeit(long stepsProSekunde);

float readMagnet_B_total_filtered();    

void berechneLogFunktion();
float B_to_D(float B);

float volumeFeedForward(float dSet, float dMeas, float rpmCurrent);

void update_Display(float dIst);
void showStatus(float dSet, float dMeas, const char* modeTxt);
void showError(const char* msg);
void showMessage(const char* msg);

void sendData(int16_t val);
void requestData();

void modusKalibrierung();
void modusPWM();
void modusRegelbetrieb();

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


/*
// -------- PID ----------

struct PID {
  float kp, ki, kd;
  float i = 0.0f;
  float prevPv = NAN;
  float outMin, outMax;

  float update(float sp, float pv, float dt) {
    float e = sp - pv;
    i += e * ki * dt;
    float d = (!isnan(prevPv)) ? (pv - prevPv) / dt : 0.0f;
    prevPv = pv;
    float u = kp*e + i - kd*d;
    if(u>outMax){u=outMax;if(i>outMax)i=outMax;}
    if(u<outMin){u=outMin;if(i<outMin)i=outMin;}
    return u;
  }
} pid;


const float PID_KP = 1.1f;
const float PID_KI = 0.4f;
const float PID_KD = 0.0f;
*/

// -------- Display ----------
Adafruit_SH1106G display(128, 64, &Wire);                                 // Selber I2C Bus -> Display Updaterate checken!!
using namespace ifx::tlx493d;
TLx493D_A1B6 Tlv493dMagnetic3DSensor(Wire, TLx493D_IIC_ADDR_A0_e);        // Selber Bus (Sensor & Display)

// -------- Kalibrierung ----------
float F_vals[NUM_CAL_SAMPLES];                                            // Rohdaten des Sensors
float D_vals[NUM_CAL_SAMPLES];                                            // Referenzdurchmesser
bool regressionDone = true;                                               // Kalibrierung schon durchgeführt?
float a = -1.07f, b = 5.67f;                                              // Koeffizienten der Kalibrierungsgeraden
 
// -------- I2C ----------
volatile int16_t targetSpeed = 0;                                         // Zielgeschwindigkeit, 2 Byte
volatile int16_t current_rpm = 0;                                         // Ist-Geschwindigkeit
volatile bool mode = false;                                               // Default Regelbetrieb
volatile bool isrunning = true;                                           // warum volatile? Keine ISR oder? Juckt wahrscheinlich nicht


// Weitere Variablen
const float DIAMETER = 0.05;                    // Durchmesser der Welle in Metern
const float CIRCUMFERENCE = PI * DIAMETER;      // Umfang in Metern
const float TOTSTRECKE = 2.0;                   // Totstrecke in Metern
const int STEPS_PER_REV = 800;                  // Schritte pro Umdrehung
unsigned long regelPauseBis = 0;                // Regelung greift nur periodisch wegen langsamer Änderung -> keine Schwingung

// Funktion: Umfangsgeschwindigkeit berechnen (m/min)
float berechneGeschwindigkeit(long stepsProSekunde) {
  float revsPerSec = (float)stepsProSekunde / STEPS_PER_REV;  // Umdrehungen pro Sekunde
  float revsPerMin = revsPerSec * 60.0;                       // U/min
  float v_mPerMin = CIRCUMFERENCE * revsPerMin;               // m/min
  return v_mPerMin;
}

// Funktion: Totzeit berechnen (s): Messänderung zu Abzugsgeschwindigkeitsänderung
float berechneTotzeit(long stepsProSekunde) {
  float v_mPerMin = berechneGeschwindigkeit(stepsProSekunde);
  float v_mPerSec = v_mPerMin / 60.0;                         // m/s
  if (v_mPerSec <= 0) return -1;                              // Schutz vor Division durch 0
  float t = TOTSTRECKE / v_mPerSec;                           // s
  return t;
}

// --Eventuell Problem mit I2C?? Sehr lang, blockiert Master -> Eventuell Sensor read nicht am Stück abfertigen?
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

//--- Parameterberechnung für LogFunk bei Kalibrierung (funktioniert); Regression
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

float B_to_D(float B){ return a*log(B)+b; }                                   //Kalibrierte Umrechnung


//--- greift in Automatikmodus; 

float Faktor = 0.05;

float volumeFeedForward(float dSet,float dMeas,float rpmCurrent){
  Serial.print("VolumeFeedForward");
  if(dMeas<=0) return rpmCurrent;
  float ratio=(dMeas*dMeas)/(dSet*dSet);                                      //Verhältnis von gemessenem Durchmesser und Zieldurchmesser

  if (ratio < 1){ratio = ratio*(1+Faktor);}                                   //Überkompensiert
  if (ratio > 1){ratio = ratio*(1-Faktor);}                                   //Dämpft
  return rpmCurrent*ratio;
}

//--- Display Zeugs für Manuell
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


  if (!isrunning) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print("Stopped");
  }

  display.display();
  delay(100);                                                                //mehr delay, warum?
}

//--- Display Zeugs für Automatik
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

// -------- I2C Handlers ----------
// --- Senden von targetSpeed ---
void sendData(int16_t val) {
    Wire.beginTransmission(8); // Arduino Slave
    Wire.write((val >> 8) & 0xFF);  // High Byte
    Wire.write(val & 0xFF);         // Low Byte
    Wire.endTransmission();  // Fehlercode speichern

    Serial.print("Master -> Slave gesendet: ");                       //Serial auf Master Seite wahrscheinlich kein Problem, aber recherchieren
    Serial.println(val);
}

void requestData() {
  int n = Wire.requestFrom(8, 4); // Master fragt die 4 Bytes, 2 Bytes speed + 1 Byte running + 1 Byte mode; n nicht weiter genutzt
 /* Serial.print("Angefordert=4, erhalten=");
  Serial.println(n);*/

    unsigned long startTime = millis();
    while (Wire.available() < 4) {                                            //Wartet bis alle 4 Bytes im Puffer sind
        if (millis() - startTime > 10) { // max 10 ms warten
            Serial.println("Fehler: unvollständige Daten empfangen!");
            return;
        }
    }

    int16_t speed = (Wire.read() << 8) | Wire.read();                         //Gesplittete Bytes rekonstrukiert
    bool running = Wire.read();
    bool modeVal = Wire.read();

    //Globale Variablen aktualisieren
    current_rpm = speed;
    isrunning = running;
    mode = modeVal;
    
    //debug ausgabe; Auch hier, Serial bei Master vermutlich ok, aber Recherche
    Serial.print("Master <- Slave: Speed=");
    Serial.print(speed);
    Serial.print(", Running=");
    Serial.print(running);
    Serial.print(", Mode=");
    Serial.println(modeVal);
}
 
// -------- Setup ----------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_CONFIRM, INPUT_PULLUP);

  Serial.begin(115200);
  delay(2000);      // Damit Serial genug Zeit zum starten hat
  Wire.begin();    // Sensor auf I2C0 (Pins 4/5)
  Wire.setClock(50000);  // 50 kHz




  if(!display.begin(0x3C)){ while(1); }                                                           //Selbstprüfung
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0); display.print("Start..."); display.display();                           //der Kollge wird dauerhaft ausgegeben im Moment -> Display Boot Problem

  Tlv493dMagnetic3DSensor.begin();
  TLx493D_t* Typ = Tlv493dMagnetic3DSensor.getSensor();                                           // Ungenutzt

//  pid.kp=PID_KP; pid.ki=PID_KI; pid.kd=PID_KD;
//  pid.outMin=0; pid.outMax=RPM_MAX;
  Serial.println("Starting");                                                               
  delay(2500); 

}

// -------- Loop ----------
void loop() {
  requestData();                                      //Abfrage von Modus, Messdaten,...
  if (!regressionDone) {                              //Kalibrierungsabfrage
    modusKalibrierung();
  } else {
      if (mode) modusRegelbetrieb();                  //Automatik
      else      modusPWM();                           //Manuell
    }
}


// -------- Modus-Funktionen ----------
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

void modusPWM() {
    float B = readMagnet_B_total_filtered();                                                      //Magnetfelddaten inkl Trimmung
    Serial.println(B);                                                                            //Debug Ausgabe
    if(B <= 0){ showError("Ungueltiger Sensorwert"); return; }                                    //Plausibilitätscheck
    float D_meas = B_to_D(B);
    
    update_Display(D_meas);                                                                       // Zeigt die aktuellen Werte vom Arduino
    delay(550);                                                                                   //Cüs Junge was ein Delay
}

void modusRegelbetrieb() {
    // Wenn wir uns noch in der "Pausezeit" befinden → überspringen
    if (millis() < regelPauseBis) {
    delay(10);
      return;  
    }

    float B = readMagnet_B_total_filtered();                                                    //Getrimmte Magnetfeldwerte
    Serial.println(B);
    if(B <= 0){ showError("Ungueltiger Sensorwert"); return; }                                  //Plausibilitätscheck
    float D_meas = B_to_D(B);
    Serial.print("D_meas= ");
    Serial.println(D_meas);
    if(isnan(D_meas)){ showError("Fehler Berechnung D"); return; }                              //Prüft Gültigkeit des Ergebnisses

    float err = TARGET_DIAMETER_MM - D_meas;                                                    // Abweichung zwischen Soll und Ist
    float rpmSet = 0.0f;                                                                        //Geforderte Drehzahl

    // Feed-Forward (PID ist auskommentiert)
    rpmSet = volumeFeedForward(TARGET_DIAMETER_MM, D_meas, (float)current_rpm);

    rpmSet = constrain(rpmSet, 0.0f, RPM_MAX);                                                  //Untere Grenze -> Kein Rückwärtslauf
    targetSpeed = (int16_t)rpmSet; // immer aktiv

    // Stabilitäts-Check
    if(fabs(err) <= STABLE_TOLERANCE) {
        if(stableStartTime == 0) stableStartTime = millis();
        if(millis() - stableStartTime >= STABLE_DURATION_MS) {                                  //Muss bestimmte Zeit im Toleranzbereich sein um als stabil zu gelten
            stable = true;
        }
    } else {
        stableStartTime = 0;
        stable = false;
    }

    showStatus(TARGET_DIAMETER_MM, D_meas, "Regel");                                            //Zeigt Soll und Ist auf Monitor
    sendData(targetSpeed);                                                                      //Schickt Target Speed an Arduino

    float t_sec = berechneTotzeit(targetSpeed);                                                 //Totzeit zur Stabilisierung
    if (t_sec > 0) {
        unsigned long t_ms = (unsigned long)(t_sec * 1000.0f);
//      Serial.print("Dynamische Pause bis: ");
//      Serial.println(millis() + t_ms);
        regelPauseBis = millis() + t_ms;   // anstatt delay()
    }
}