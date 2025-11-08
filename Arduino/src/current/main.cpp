#include <AccelStepper.h>                     //Motor Bib
#include <Wire.h>                             //I2C Bib

#define SLAVE_ADDR 0x08                       //I2C Adresse Arduino; und Slave

#define DIR_PIN 5                     
#define STEP_PIN 6
#define modeButton 7
#define buttonStart 8
#define buttonStop 9
#define outputLED 10

//---Funnktionsdeklarationen---
void updateEncoder();
void requestEvent();
void receiveEvent(int);
void handleEncoder();

// Rotary Encoder
const int PIN_A = 2;                          // Interrupt fähige Pins (korrekt)
const int PIN_B = 3;
const int PIN_SWITCH = 4;

// Stop-Taster Debounce 
const unsigned long stableTime = 50;          // Signal muss 50ms stabil bleiben (gegen Tasterprellen)
unsigned long signalStartTime = 0;          
bool signalStable = false;

// Encoder-Variablen
volatile int encoderPos = 60;                 // Startwert für Geschwindigkeit
volatile bool aSet = false;                   // Volatile -> Variable immer aus Speicher gelesen und Änderungen sofort zurückgeschrieben; wichtig für interrupt

int currentSpeed = 30;                        // Gesetzer Wert an Stepper
volatile int desiredSpeed = 60;               // Soll von Encoder/I2C -> Startwert, weil zu Beginn noch keine Daten

// Modus / Laufzustand; Status Flags
bool mode = false;                            // Manuell; Automatik
bool isrunning = false;                       // Läuft Motor?
bool Controllmode = false;                    // Same wie mode?!

// Debounce für Encoder
const unsigned long debounceDelay = 50;       // übernimmt encoder Wert alle 50ms
unsigned long lastDebounceTime = 0;

// Motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);            // Driver Mode -> Steuersignale and Motortreiber


volatile bool sendFlag = false;                // Debug Daten für I2C
volatile bool sendFlag2 = false;
volatile int16_t receivedSpeed = 0;
volatile int debugSpeed;
bool debugRun, debugMode;



void setup() {
  pinMode(PIN_A, INPUT_PULLUP);   
  pinMode(PIN_B, INPUT_PULLUP);
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  pinMode(modeButton, INPUT_PULLUP);
  pinMode(buttonStart, INPUT_PULLUP);
  pinMode(buttonStop, INPUT_PULLUP);
  pinMode(outputLED, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_A), updateEncoder, CHANGE);           // Interrupt wenn change an Pin A

  Serial.begin(9600);

  stepper.setMaxSpeed(3000);                  // Grenzen für Motor
  stepper.setAcceleration(50);

  Wire.begin(SLAVE_ADDR);                     // Startet I2C im Slave Modus
  Wire.onRequest(requestEvent);               // Master liest, Arduino antwortet
  Wire.onReceive(receiveEvent);               // Master schreibt, Arduino empfängt
}

void loop() {
  // Outsourced Serial print in normale Loop -> Serial zu langsam für Callback (mega smart)
  if (sendFlag) {                                           // Wird in callback Schleife true gesetzt
    sendFlag = false;                                       // Beendet callback flag -> wird erst wieder nach neuem callback Input durchlaufen
    Serial.print("Slave -> Master gesendet: Speed=");       // serial outputs in normaler Loop
    Serial.print(debugSpeed);
    Serial.print(", Running=");
    Serial.print(debugRun);
    Serial.print(", Mode=");
    Serial.println(debugMode);
  }

  if (sendFlag2) {
    sendFlag2 = false;
    noInterrupts();                                             //Keine ISR beim lesen globaler Variablen -> Kaputte Werte
    int16_t rx = receivedSpeed;                                 // atomar kopieren
    interrupts();
    Serial.print("Slave <- Master empfangen: ");                // !!!Serial eigentlich zu langsam für I2C Callback; Könnte Probleme machen!!!
    Serial.println(rx);
  }

  //unsigned long now = millis();                             // Zeitstempel --- Momentan ungenutzt

  // Mode Button prüfen
  mode = (digitalRead(modeButton) == LOW);                    // Aktiv wenn Eingangspin 0V
  Controllmode = mode ? 1 : 0;                                // ? -> 1 wenn true; 0 wenn false -> Ist doch voll überflüßsig? Warum nicht einfach eine bool?

    static bool lastMode = false;                             // Übernimmt bei Moduswechsel aktuellen Speed als Sollwert -> Sanfter Übergang
  if (mode != lastMode) {
    desiredSpeed = currentSpeed;  // aktuellen Wert übernehmen
    lastMode = mode;
  }

  // Start-Taster
  if (digitalRead(buttonStart) == LOW) {
 //   Serial.println("Start Button Pressed");
    isrunning = true;
    digitalWrite(outputLED, HIGH);                            // LED an wenn Motor läüft
  }

  // Stop-Taster
  int reading = digitalRead(buttonStop);                      // Stoppt Motor
  if (reading == LOW && !signalStable) {                      // prüft nur wenn Signal noch nicht als stabil gilt 
    if (millis() - signalStartTime > stableTime) {            // stabil ab 50ms
      signalStable = true;
    //  Serial.println("Stop Button Pressed");
      isrunning = false;                                      // dann motor aus -> Entprellen
      digitalWrite(outputLED, LOW);
      stepper.setSpeed(0);
      // Encoderwert zurücksetzen optional
      encoderPos = 60;                                        // Zurücksetzen auf default speed
    }
  } else if (reading == HIGH) {
    signalStartTime = millis();
    signalStable = false;
  }

  // Steuerung abhängig vom Modus
  if (isrunning) {
    if (Controllmode == 0) {                                  // mode 0 -> Sollwert vom Encoder
      handleEncoder();
    } else if (Controllmode == 1) {                           // I2C mode
      if (desiredSpeed != currentSpeed) {
        currentSpeed = desiredSpeed;
        stepper.setSpeed(-currentSpeed);                      // Vorzeichen für korrekte Richtung
      }
    }
  }

  stepper.runSpeed();                                         // Keine Rampe für sanfte Tempoanpassung?! -> Vmtl kein Problem mit Regelung/Encoder; Vlt trotzdem einfügen
}

// --- Encoder Verarbeitung ---
void handleEncoder() {
  unsigned long now = millis();
  if (now - lastDebounceTime > debounceDelay) {                // alle 50ms abgefragt
    lastDebounceTime = now;

    desiredSpeed = encoderPos;                                 // neuer desired speed
    if (desiredSpeed != currentSpeed) {
      currentSpeed = desiredSpeed;                             // set speed
      stepper.setSpeed(-currentSpeed);                         // Vorzeichen anpassen
    }
  }
}

// --- Encoder ISR ---
void updateEncoder() {                                          // Aufgerufen bei Encoder change (ISR)
  bool newA = digitalRead(PIN_A);
  bool newB = digitalRead(PIN_B);

  if (newA != aSet) {                                           // nur auf Flanke von A reagieren; B bestimmt Richtung
    aSet = newA;

    if (newA == newB) encoderPos++;                             // Vorwärts
    else encoderPos--;                                          // Rückwärts

    // Begrenzung auf 0-3000
    if (encoderPos < 0) encoderPos = 0;
    if (encoderPos > 3000) encoderPos = 3000;
  }
}

// --- I2C Funktionen ---
// --- Master fragt Daten ab ---
void requestEvent() {                                           // Slave sendet 4 Bytes auf Abfrage
  Wire.write((currentSpeed >> 8) & 0xFF);                       // I2C Schnittstelle immer 1 Byte auf einmal -> 16 bit aufgeteilt
  Wire.write(currentSpeed & 0xFF);
  Wire.write(isrunning ? 1 : 0);
  Wire.write(Controllmode ? 1 : 0);

  debugSpeed = currentSpeed;
  debugRun   = isrunning;
  debugMode  = Controllmode;
  sendFlag = true;                                              // debug im nächsten Durchlauf
}

void receiveEvent(int howMany) {                                // Slave empfängt 2 Bytes
    if (howMany < 2) return;                                    // Abbruch bei weniger 2 Bytes

    byte highByte = Wire.read();                                // 16bit wieder aufgeteilt
    byte lowByte  = Wire.read();

    receivedSpeed = (highByte << 8) | lowByte;                  // Bytes kombinieren
    desiredSpeed = receivedSpeed;
    sendFlag2 = true;
}