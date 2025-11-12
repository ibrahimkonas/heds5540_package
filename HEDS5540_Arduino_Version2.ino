// HEDS5540_Arduino.ino
// Genel HEDS-5540 örnek sketch (quadrature CHANGE ile pozisyon okuma)
// D2 = A, D3 = B, D4 = Z (opsiyonel)
// Harici pull-up: 2.7k önerildi (datasheet)

const int pinA = 2;    // INT0
const int pinB = 3;    // INT1
const int pinZ = 4;    // index (opsiyonel)

volatile long position = 0;
volatile unsigned long pulseCount = 0;

// Ayarlayın:
const unsigned int PPR = 128; // datasheet'teki cycles per rev (örnek). Bunu gerçek değerle değiştirin.
const unsigned int countsPerRev = PPR * 4; // quadrature counts

unsigned long lastReportMs = 0;
const unsigned long reportIntervalMs = 500;

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinZ, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(pinZ), isrZ, RISING); // opsiyonel

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("HEDS-5540 Arduino genel örnek başlıyor...");
}

void loop() {
  unsigned long now = millis();
  if (now - lastReportMs >= reportIntervalMs) {
    noInterrupts();
    unsigned long p = pulseCount;
    pulseCount = 0;
    long pos = position;
    interrupts();

    double intervalSec = (now - lastReportMs) / 1000.0;
    double revs = (double)p / (double)countsPerRev;
    double rpm = (revs / intervalSec) * 60.0;
    double degrees = (double)pos * 360.0 / (double)countsPerRev;

    Serial.print("RPM: ");
    Serial.print(rpm, 2);
    Serial.print(" | Pos(counts): ");
    Serial.print(pos);
    Serial.print(" | Angle(deg): ");
    Serial.println(degrees, 2);

    lastReportMs = now;
  }
}

void isrA() {
  bool a = digitalRead(pinA);
  bool b = digitalRead(pinB);
  if (a == b) position++;
  else position--;
  pulseCount++;
}

// void isrZ() { position = 0; } // opsiyonel