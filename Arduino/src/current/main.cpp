#include <Wire.h>
#include <Arduino.h>

#define SLAVE_ADDR 0x08

// --- Pin Definitionen ---
#define DIR_PIN   5
#define STEP_PIN  6
#define MODE_BUTTON_PIN 7
#define START_BUTTON_PIN 8
#define STOP_BUTTON_PIN 9
#define LED_PIN 10

// Encoder
const int PIN_A = 2;
const int PIN_B = 3;

// --- Variablen für Speed & Rampe ---
volatile float currentSpeed = 0.0;    
volatile float targetSpeed = 0.0;     
float accelMan = 400.0;           // Steps/sec²
float accelAuto = 25.0f;              // ≈ 0,3 m/min pro Sekunde

// Encoder Variablen
volatile long encoderPos = 60;        
volatile bool aSet = false;

// I2C Kommunikation
volatile bool sendFlag = false;
volatile bool sendFlag2 = false;
volatile int16_t receivedSpeed = 0;
volatile int16_t desiredSpeed = 0;
static long lastEncPosForI2C = 0;

// Minimale und Maximale OCR Werte
const unsigned int MIN_OCR = 100;     
const unsigned int MAX_OCR = 65000;   

// --- Status-Variablen ---
bool isRunning = false;       
bool isAutoMode = false;      

// Variablen für die Stop-Entprellung (Gegen Geister-Stopps)
unsigned long stopSignalStartTime = 0;
bool stopSignalStable = false;
const unsigned long stableTime = 50; 

// -----------------------------------------------------------------------------
// TIMER1 ISR: Erzeugt die Pulse
// -----------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
    PORTD |= (1 << 6);   // STEP_PIN HIGH
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); 
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); 
    PORTD &= ~(1 << 6);  // STEP_PIN LOW
}

// -----------------------------------------------------------------------------
// Berechnet den Timer-Wert (OCR1A) und setzt die Richtung
// -----------------------------------------------------------------------------
void setTimerFrequency(float speed)
{
    // RICHTUNG GEÄNDERT:
    // Wenn speed >= 0 ist, setzen wir den Pin jetzt auf LOW (vorher HIGH).
    if (speed >= 0) {
        PORTD &= ~(1 << 5); // DIR_PIN LOW
    } else {
        PORTD |= (1 << 5);  // DIR_PIN HIGH
    }

    float absSpeed = abs(speed);

    if (absSpeed < 1.0) {
        TIMSK1 &= ~(1 << OCIE1A); 
        return;
    }

    TIMSK1 |= (1 << OCIE1A); 
    long ocr_val = (16000000L / (8L * (long)absSpeed)) - 1;

    if (ocr_val < MIN_OCR) ocr_val = MIN_OCR;
    if (ocr_val > MAX_OCR) ocr_val = MAX_OCR;

    noInterrupts();
    OCR1A = (unsigned int)ocr_val;
    interrupts();
}

// -----------------------------------------------------------------------------
// Rampe berechnen
// -----------------------------------------------------------------------------
void updateRamp()
{
    static unsigned long lastUpdate = 0;
    unsigned long now = micros();
    unsigned long dt = now - lastUpdate;

    if (dt < 2000) return; 
    lastUpdate = now;

    float timeStep = dt / 1000000.0; 

    float acc = isAutoMode ? accelAuto : accelMan;      //getrennte Rampen für manuell und Automatik

    if (currentSpeed < targetSpeed) {
        currentSpeed += acc * timeStep;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    } 
    else if (currentSpeed > targetSpeed) {
        currentSpeed -= acc * timeStep;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }

    setTimerFrequency(currentSpeed);
}

// -----------------------------------------------------------------------------
// Encoder ISR
// -----------------------------------------------------------------------------
void updateEncoder()
{
    bool newA = (PIND & (1 << 2)); 
    bool newB = (PIND & (1 << 3)); 

    if (newA != aSet) {
        aSet = newA;
        if (newA == newB) encoderPos++;
        else encoderPos--;

        // Begrenzung (Wieder auf 0-3000 wie im alten Code, falls gewünscht)
        if (encoderPos < 0) encoderPos = 0; 
        if (encoderPos > 3000) encoderPos = 3000;
    }
}

// ----------------------------------------------------------------------------
// I2C Callbacks
// ----------------------------------------------------------------------------
void requestEvent() {
    int16_t speedToSend = (int16_t)currentSpeed;

    long encNow = encoderPos;                  
    long diffL  = encNow - lastEncPosForI2C;
    lastEncPosForI2C = encNow;

    if (diffL >  32767) diffL =  32767;
    if (diffL < -32768) diffL = -32768;
    int16_t encDelta = (int16_t)diffL;

    Wire.write((speedToSend >> 8) & 0xFF);
    Wire.write(speedToSend & 0xFF);

    Wire.write(isRunning ? 1 : 0);
    Wire.write(isAutoMode ? 1 : 0);

    Wire.write((encDelta >> 8) & 0xFF);
    Wire.write(encDelta & 0xFF);

    sendFlag = true;
}

void receiveEvent(int howMany) {
    if (howMany < 2) return;
    byte highByte = Wire.read();
    byte lowByte  = Wire.read();
    receivedSpeed = (highByte << 8) | lowByte;
    desiredSpeed = receivedSpeed;
    sendFlag2 = true;
    while (Wire.available()) {
    Wire.read();
    }
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_A), updateEncoder, CHANGE);
    Serial.begin(9600);

    // Timer 1 Setup
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    TCCR1B |= (1 << WGM12); 
    TCCR1B |= (1 << CS11);  
    OCR1A = 60000; 
    TIMSK1 |= (1 << OCIE1A);
    interrupts();

    Wire.begin(SLAVE_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    
    targetSpeed = 0;
    currentSpeed = 0;
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    // 1. Mode Taster Polling
    isAutoMode = (digitalRead(MODE_BUTTON_PIN) == LOW);

    //if (digitalRead(MODE_BUTTON_PIN) == LOW) {
     //   delay(50); 
     //   if (digitalRead(MODE_BUTTON_PIN) == LOW) {
     //        isAutoMode = !isAutoMode;
      //       while(digitalRead(MODE_BUTTON_PIN) == LOW); 
      //       if (!isAutoMode) {
      //           encoderPos = (long)currentSpeed;
      //       }
      //  }
    //}

    //Wechselerkennung -> Handover
    static bool lastAuto = false;

    // Detect mode change
    if (isAutoMode != lastAuto) {
        if (isAutoMode) {
        // MAN -> AUTO: starte Auto mit dem aktuellen Manual-Sollwert
        receivedSpeed = (int16_t)encoderPos;     // Auto bekommt "aktuellen Manual-Wert"
        lastEncPosForI2C = encoderPos;           // Damit delta = 0 bei Auto Start
    } else {
        // AUTO -> MAN: starte Manual mit dem aktuellen Auto-Sollwert (oder currentSpeed)
        encoderPos = (long)receivedSpeed;        // Encoder springt auf Auto-Wert
        // alternativ: encoderPos = (long)currentSpeed;
    }
    lastAuto = isAutoMode;
    }

    // Start Taster
    if (digitalRead(START_BUTTON_PIN) == LOW) {
        isRunning = true;
        digitalWrite(LED_PIN, HIGH);
    }

    // --- STOP Taster mit Sicherheits-Entprellung ---
    int stopReading = digitalRead(STOP_BUTTON_PIN);
    if (stopReading == LOW && !stopSignalStable) {
        if (millis() - stopSignalStartTime > stableTime) {
            stopSignalStable = true;
            isRunning = false;
            digitalWrite(LED_PIN, LOW);
            encoderPos = 60; 
        }
    } else if (stopReading == HIGH) {
        stopSignalStartTime = millis();
        stopSignalStable = false;
    }

    // 2. Soll-Geschwindigkeit festlegen
    if (!isRunning) {
        targetSpeed = 0;
    } 
    else {
        if (isAutoMode) {
            targetSpeed = (float)receivedSpeed;
            ////encoderPos = receivedSpeed; --> Überschreibt encoderPos bei dSoll Wahl           --Überschreibt encoderposition im Automode
        } else {
            targetSpeed = (float)encoderPos;
        }
    }

    // 3. Rampe und Timer updaten
    updateRamp();

    // 4. Debugging Ausgaben
    static unsigned long lastPrint = 0;
    if (now - lastPrint > 500) {
        lastPrint = now;
        Serial.print("Mode: "); Serial.print(isAutoMode ? "AUTO" : "MANU");
        Serial.print(" | Speed: "); Serial.println(currentSpeed);
    }
}