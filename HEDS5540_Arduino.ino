// HEDS5540_Arduino.ino
// Örnek: HEDS-5540 enkoderini Arduino Uno ile okumak (quadrature decoding).
// Dikkat: countsPerRev değerini kendi enkoderinizin datasheet'ine göre ayarlayın.
// Eğer enkoder PPR (cycles per rev) veriyorsa, countsPerRev = PPR * 4 (quadrature).

const int pinA = 2;    // INT0
const int pinB = 3;    // INT1 (okuma için, gerekirse interrupt'a alınabilir)
const int pinZ = 4;    // index (opsiyonel)

volatile long position = 0;     // mutabık sayaç (kesme içinde güncellenir)
volatile unsigned long pulseCount = 0; // döngü bazlı sayım

// Ayarlayın:
const unsigned int PPR = 128; // örnek: enkoder üzerinde yazan cycles per rev (datasheet'e bakın)
const unsigned int countsPerRev = PPR * 4; // quadrature decoding ile saydığımız counts/rev

unsigned long lastReportMs = 0;
const unsigned long reportIntervalMs = 500; // her 500 ms raporlama

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinZ, INPUT);

  // Eğer enkoder open-collector ise dış pull-up kullanın; isterseniz iç pull-up'u aktif edin:
  // digitalWrite(pinA, HIGH); digitalWrite(pinB, HIGH); digitalWrite(pinZ, HIGH);

  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
  // index kullanacaksanız:
  // attachInterrupt(digitalPinToInterrupt(pinZ), isrZ, RISING);

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("HEDS-5540 Arduino örnek başlıyor...");
}

void loop() {
  unsigned long now = millis();
  if (now - lastReportMs >= reportIntervalMs) {
    noInterrupts();
    unsigned long p = pulseCount;
    pulseCount = 0;
    long pos = position; // pozisyonu oku
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

// A kanal kesme rutini (hızlı olmalı)
void isrA() {
  bool a = digitalRead(pinA);
  bool b = digitalRead(pinB);
  if (a == b) position++;
  else position--;
  pulseCount++;
}

// İndex kanalı için örnek (opsiyonel)
// void isrZ() {
//   // örn: index geldiğinde pozisyonu sıfırla / referans al
//   position = 0;
// }