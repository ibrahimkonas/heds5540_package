// HEDS5540_Nano.ino
// HEDS-5540 (5-pin) test skeci - Arduino Nano
// D2 = A (INT0), D3 = B (INT1), D4 = Z (opsiyonel)
// Pull-up: harici 2.7kΩ A -> +5V, B -> +5V (Z için opsiyonel)

const uint8_t pinA = 2; // INT0
const uint8_t pinB = 3; // INT1
const uint8_t pinZ = 4; // index (opsiyonel)

volatile long position = 0;
volatile unsigned long pulses = 0;

// ENKODERİN PPR değerini datasheet'e göre girin (ör. 128 cycles/rev -> countsPerRev = 128*4)
const unsigned int PPR = 128; // örnek, datasheet'ten doğrulayın
const unsigned int countsPerRev = PPR * 4;

unsigned long lastReport = 0;
const unsigned long reportIntervalMs = 500;

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinZ, INPUT);

  // NOT: harici 2.7k pull-up kullanıyoruz; isteğe bağlı olarak INTERNAL pull-up aktif etmek için:
  // digitalWrite(pinA, HIGH); digitalWrite(pinB, HIGH);

  // İnterruptları her iki kanal için de CHANGE tetikleyici ile kur
  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);
  // index için:
  // attachInterrupt(digitalPinToInterrupt(pinZ), isrZ, RISING);

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("HEDS-5540 test (Nano) basladi");
}

void loop() {
  unsigned long now = millis();
  if (now - lastReport >= reportIntervalMs) {
    noInterrupts();
    unsigned long p = pulses;
    pulses = 0;
    long pos = position;
    interrupts();

    double intervalSec = (now - lastReport) / 1000.0;
    double revs = (double)p / (double)countsPerRev;
    double rpm = (revs / intervalSec) * 60.0;
    double angle = (double)pos * 360.0 / (double)countsPerRev;

    Serial.print("RPM: ");
    Serial.print(rpm, 2);
    Serial.print(" | Pos(counts): ");
    Serial.print(pos);
    Serial.print(" | Angle(deg): ");
    Serial.println(angle, 2);

    lastReport = now;
  }
}

// Basit quadrature okuma: A veya B değiştiğinde her iki pini oku ve yönü belirle
void isrA() { updateFromPins(); }
void isrB() { updateFromPins(); }

inline void updateFromPins() {
  // Hız için mümkün olduğunca hızlı yapın:
  bool a = digitalRead(pinA);
  bool b = digitalRead(pinB);
  // yön belirleme: klasik yöntem
  if (a == b) {
    position++;
  } else {
    position--;
  }
  pulses++;
}

// Opsiyonel index ISR
// void isrZ() {
//   // index geldiğinde referans sıfırlama örneği
//   position = 0;
// }