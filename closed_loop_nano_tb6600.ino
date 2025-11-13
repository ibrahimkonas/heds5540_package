// Nano + TB6600 + HEDS-5540 (yazılım quadrature + Timer1 step üretimi)
// A->D2, B->D3, STEP->D9 (OC1A), DIR->D8, EN->D10
// Harici 2.7k pull-ups A/B -> +5V
//
// Measured encoder CPR: 400 -> quadrature counts/rev = 400 * 4 = 1600

#include <Arduino.h>

// Pins
const uint8_t PIN_A   = 2;  // INT0
const uint8_t PIN_B   = 3;
const uint8_t PIN_Z   = 4;  // optional
const uint8_t PIN_DIR = 8;
const uint8_t PIN_STEP = 9; // OC1A
const uint8_t PIN_EN  = 10;

// Hardware params
const long motor_full_steps_per_rev = 200;
const int microstep = 16;
const long motor_steps_per_rev = motor_full_steps_per_rev * (long)microstep;

// SET THIS AFTER MEASUREMENT:
volatile long encoder_counts_per_rev = 1600; // CPR 400 -> quadrature counts/rev = 1600
double encoder_counts_per_step = 0.0;

const double MAX_STEP_FREQ_HZ = 2000.0;
const double MIN_STEP_FREQ_HZ = 0.5;

double Kp = 0.8;
double Ki = 0.04;
double Kd = 0.0;

volatile long encoder_count = 0;
volatile long target_steps = 0;

double integral = 0;
double lastError = 0;

void isrA_rising();
void setupTimer1Disabled();
bool setStepFrequency(double freqHz);
void stopSteps();

void setup() {
  cli();
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  pinMode(PIN_Z, INPUT);

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_EN, LOW);

  if (encoder_counts_per_rev > 0) encoder_counts_per_step = (double)encoder_counts_per_rev / (double)motor_steps_per_rev;
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrA_rising, RISING);

  setupTimer1Disabled();

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Closed-loop Nano+TB6600 başlatıldı.");
  Serial.print("Encoder CPR (measured): 400 -> encoder_counts/rev = "); Serial.println(encoder_counts_per_rev);
  sei();
}

unsigned long lastLoopMs = 0;
const unsigned long loopDtMs = 20;

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') target_steps += 1000;
    if (c == 'r') target_steps -= 1000;
    if (c == 's') { target_steps = 0; stopSteps(); Serial.println("Stop"); }
  }

  unsigned long now = millis();
  if (now - lastLoopMs >= loopDtMs) {
    lastLoopMs = now;
    noInterrupts();
    long enc = encoder_count;
    interrupts();

    double actual_steps = 0.0;
    if (encoder_counts_per_step > 0.0) actual_steps = (double)enc / encoder_counts_per_step;
    long actual_steps_round = (long)round(actual_steps);

    double error = (double)target_steps - actual_steps;
    double dt = (double)loopDtMs / 1000.0;
    integral += error * dt;
    double derivative = (error - lastError) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    double desired_freq = fabs(output);
    if (desired_freq > MAX_STEP_FREQ_HZ) desired_freq = MAX_STEP_FREQ_HZ;
    if (desired_freq < MIN_STEP_FREQ_HZ) desired_freq = 0.0;

    if (output >= 0) digitalWrite(PIN_DIR, HIGH); else digitalWrite(PIN_DIR, LOW);

    if (desired_freq > 0.0) {
      bool ok = setStepFrequency(desired_freq);
      if (!ok) stopSteps();
    } else {
      stopSteps();
    }

    static unsigned long lastPrint = 0;
    if (now - lastPrint > 500) {
      Serial.print("Tgt:"); Serial.print(target_steps);
      Serial.print(" Act:"); Serial.print(actual_steps_round);
      Serial.print(" Err:"); Serial.print((long)round(error));
      Serial.print(" Freq:"); Serial.print(desired_freq,1);
      Serial.print(" enc_counts:"); Serial.println(enc);
      lastPrint = now;
    }
  }
}

void isrA_rising() {
  bool b = (PIND & (1 << PD3));
  if (b) encoder_count++; else encoder_count--;
}

void setupTimer1Disabled() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  OCR1A = 0;
  digitalWrite(PIN_STEP, LOW);
  sei();
}

struct Presc { uint16_t val; uint8_t csbits; };
const Presc prescalers[] = {
  {1,   (1<<CS10)},
  {8,   (1<<CS11)},
  {64,  (1<<CS11)|(1<<CS10)},
  {256, (1<<CS12)},
  {1024,(1<<CS12)|(1<<CS10)}
};

bool setStepFrequency(double freqHz) {
  if (freqHz <= 0.0) { setupTimer1Disabled(); return false; }
  const double F_CPU_D = (double)F_CPU;
  uint16_t bestOCR = 0;
  uint8_t bestCS = 0;
  bool found = false;
  for (unsigned i = 0; i < sizeof(prescalers)/sizeof(Presc); ++i) {
    double pres = (double)prescalers[i].val;
    double ocrd = F_CPU_D / (2.0 * pres * freqHz) - 1.0;
    if (ocrd >= 0.0 && ocrd <= 65535.0) {
      bestOCR = (uint16_t)round(ocrd);
      bestCS = prescalers[i].csbits;
      found = true;
      break;
    }
  }
  if (!found) return false;
  cli();
  TCCR1A = (1 << COM1A0);
  TCCR1B = (1 << WGM12);
  OCR1A = bestOCR;
  TCCR1B = (TCCR1B & ~((1<<CS12)|(1<<CS11)|(1<<CS10))) | bestCS;
  sei();
  return true;
}

void stopSteps() { setupTimer1Disabled(); }