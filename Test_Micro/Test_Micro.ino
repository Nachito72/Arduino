const int MIC_PIN = A0;
int center = 512;
unsigned long t0;

void setup() {
  Serial.begin(115200);

  // Calibrar centro (200 ms)
  long sum = 0;
  const int N = 300;
  for (int i = 0; i < N; i++) {
    sum += analogRead(MIC_PIN);
    delay(1);
  }
  center = sum / N;

  t0 = micros();

  Serial.println("t_us,raw,centered");
}

void loop() {
  unsigned long t = micros() - t0;
  int raw = analogRead(MIC_PIN);
  int centered = raw - center;

  Serial.print(t);
  Serial.print(",");
  Serial.print(raw);
  Serial.print(",");
  Serial.println(centered);

  // ~4 kHz
  delayMicroseconds(250);
}
