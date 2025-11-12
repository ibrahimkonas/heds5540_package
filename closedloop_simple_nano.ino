// closedloop_simple_nano.ino
// Basit kapalı döngü deneme (Arduino Nano) - düşük hız/test amaçlı
// STEP -> D9, DIR -> D8, ENABLE -> D10 (opsiyonel)
// A -> D2 (INT0), B -> D3 (INT1)
// Harici pull-ups: A,B -> 2.7k -> 5V
// NOT: Bu sketch düşük hız testleri içindir. Yüksek hızda kaçırma olabilir.

const uint8_t PIN_STEP = 9;
const uint8_t PIN_DIR  = 8;
const uint8_t PIN_EN   = 10;

const uint8_t PIN_A = 2;
const uint8_t PIN_B = 3;
volatile long encoder_count = 0;

const long motor_full_steps_per_rev = 200; // motorunuzun full step değeri
const int microstep = 16; // TB6600 DIP ayarı ile eşleşmeli
const long motor_steps_per_rev = motor_full_steps_per_rev * (long)microstep;

// Kullanıcı hedefi (adım cinsinden)
volatile long target_steps = 0;
bool stepping = false;

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW); // aktif (sürücüdeki ters olabilir, kontrol edin)

  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), isrB, CHANGE);

  Serial.begin(115200);
  while(!Serial);

  Serial.println("Basit closed-loop test hazır. Seri üzerinden 'g' ile ileri 1000 adim, 'r' ile geri 1000 adim gönder.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      target_steps += 1000;
      Serial.println("Hedef 1000 adim ileri eklendi.");
    } else if (c == 'r') {
      target_steps -= 1000;
      Serial.println("Hedef 1000 adim geri eklendi.");
    }
  }

  // Basit kontrol: eğer encoder pozisyonu hedefin gerisindeyse adım at
  noInterrupts();
  long enc = encoder_count;
  long tgt = target_steps;
  interrupts();

  // encoder_count değeri motor adım karşılığına çevrilmelidir.
  // Eğer enkoder doğrudan şaftta ise: encoder_counts_per_rev bilinmeli.
  // Bu örnek: encoder_count birimlerini "microstep karşılığı" hesaplamayı sizin ölçümünüze göre ayarlayın.
  // Burada basit: eğer hedef > enc => ileri adım gönder; tersi halde dur/geri.
  if (tgt > enc) {
    // ileri yön, tek adım at
    digitalWrite(PIN_DIR, HIGH);
    singleStep();
    delay(2); // adım hızını kontrol eden basit bekleme (daha iyi: step timer, rampa)
  } else if (tgt < enc) {
    digitalWrite(PIN_DIR, LOW);
    singleStep();
    delay(2);
  } else {
    // hedefe ulaştı
  }

  // Durumu yazdır
  static unsigned long last = 0;
  if (millis() - last > 500) {
    Serial.print("Encoder counts: "); Serial.print(enc);
    Serial.print(" | Target steps: "); Serial.println(tgt);
    last = millis();
  }
}

void singleStep() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(8); // Güvenli pulse width (sürücü datasheet'ine göre ayarlayın)
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(8);
}

// Basit quadrature okumaları (yavaş/orta hız testleri için)
// NOT: Bu ISR'ler yüksek hızlarda kaçırabilir; sadece test amaçlı.
void isrA() {
  bool a = digitalRead(PIN_A);
  bool b = digitalRead(PIN_B);
  if (a == b) encoder_count++;
  else encoder_count--;
}

void isrB() {
  bool a = digitalRead(PIN_A);
  bool b = digitalRead(PIN_B);
  if (a != b) encoder_count++;
  else encoder_count--;
}