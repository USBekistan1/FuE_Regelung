#include <AccelStepper.h>
#include <Wire.h>

#define SLAVE_ADDR 0x08

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
const int PIN_A = 2;
const int PIN_B = 3;
const int PIN_SWITCH = 4;

// Stop-Taster Debounce
const unsigned long stableTime = 50;
unsigned long signalStartTime = 0;
bool signalStable = false;

// Encoder-Variablen
volatile int encoderPos = 60;   // Startwert für Geschwindigkeit
volatile bool aSet = false;

int currentSpeed = 30;
int desiredSpeed = 60;

// Modus / Laufzustand
bool mode = false;
bool isrunning = false;
bool Controllmode = false;

// Debounce für Encoder
const unsigned long debounceDelay = 50; 
unsigned long lastDebounceTime = 0;

// Motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


volatile bool sendFlag = false;
int debugSpeed;
bool debugRun, debugMode;



void setup() {
  pinMode(PIN_A, INPUT_PULLUP);   
  pinMode(PIN_B, INPUT_PULLUP);
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  pinMode(modeButton, INPUT_PULLUP);
  pinMode(buttonStart, INPUT_PULLUP);
  pinMode(buttonStop, INPUT_PULLUP);
  pinMode(outputLED, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_A), updateEncoder, CHANGE);

  Serial.begin(9600);

  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(50);

  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent); 
}

void loop() {

  if (sendFlag) {
    sendFlag = false;
    Serial.print("Slave -> Master gesendet: Speed=");
    Serial.print(debugSpeed);
    Serial.print(", Running=");
    Serial.print(debugRun);
    Serial.print(", Mode=");
    Serial.println(debugMode);
  }

  //unsigned long now = millis();                       --- Nicht Genutzt!!

  // Mode Button prüfen
  mode = (digitalRead(modeButton) == LOW);
  Controllmode = mode ? 1 : 0;

    static bool lastMode = false;
  if (mode != lastMode) {
    desiredSpeed = currentSpeed;  // aktuellen Wert übernehmen
    lastMode = mode;
  }

  // Start-Taster
  if (digitalRead(buttonStart) == LOW) {
 //   Serial.println("Start Button Pressed");
    isrunning = true;
    digitalWrite(outputLED, HIGH);
  }

  // Stop-Taster
  int reading = digitalRead(buttonStop);
  if (reading == LOW && !signalStable) {
    if (millis() - signalStartTime > stableTime) {
      signalStable = true;
    //  Serial.println("Stop Button Pressed");
      isrunning = false;
      digitalWrite(outputLED, LOW);
      stepper.setSpeed(0);
      // Encoderwert zurücksetzen optional
      encoderPos = 60;
    }
  } else if (reading == HIGH) {
    signalStartTime = millis();
    signalStable = false;
  }

  // Steuerung abhängig vom Modus
  if (isrunning) {
    if (Controllmode == 0) {
      handleEncoder();
    } else if (Controllmode == 1) {
      if (desiredSpeed != currentSpeed) {
        currentSpeed = desiredSpeed;
        stepper.setSpeed(-currentSpeed); // Vorzeichen für korrekte Richtung
      }
    }
  }

  stepper.runSpeed();
}

// --- Encoder Verarbeitung ---
void handleEncoder() {
  unsigned long now = millis();
  if (now - lastDebounceTime > debounceDelay) {
    lastDebounceTime = now;

    desiredSpeed = encoderPos;
    if (desiredSpeed != currentSpeed) {
      currentSpeed = desiredSpeed;
      stepper.setSpeed(-currentSpeed); // Vorzeichen anpassen
    }
  }
}

// --- Encoder ISR ---
void updateEncoder() {
  bool newA = digitalRead(PIN_A);
  bool newB = digitalRead(PIN_B);

  if (newA != aSet) { // nur auf Flanke von A reagieren
    aSet = newA;

    if (newA == newB) encoderPos++;  // Vorwärts
    else encoderPos--;              // Rückwärts

    // Begrenzung auf 0-3000
    if (encoderPos < 0) encoderPos = 0;
    if (encoderPos > 3000) encoderPos = 3000;
  }
}

// --- I2C Funktionen ---
// --- Master fragt Daten ab ---
void requestEvent() {
  Wire.write((currentSpeed >> 8) & 0xFF);
  Wire.write(currentSpeed & 0xFF);
  Wire.write(isrunning ? 1 : 0);
  Wire.write(Controllmode ? 1 : 0);

  debugSpeed = currentSpeed;
  debugRun   = isrunning;
  debugMode  = Controllmode;
  sendFlag = true;
}

void receiveEvent(int howMany) {
    if (howMany < 2) return;

    byte highByte = Wire.read();
    byte lowByte  = Wire.read();

    int16_t receivedSpeed = (highByte << 8) | lowByte;
    desiredSpeed = receivedSpeed;

    Serial.print("Slave <- Master empfangen: ");
    Serial.println(receivedSpeed);
}