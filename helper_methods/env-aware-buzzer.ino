#define BUZZER_PIN 5
int sound_sensor = A1;

const int buzzerChannel = 0;    // LEDC channel
const int buzzerResol = 8;      // 8-bit resolution

// Songs
int songOneMelody[] = {523, 587, 659, 784, 784, 659, 587, 523, 659, 698, 784, 1047, 988, 784, 659, 587};
int songOneDurations[] = {300, 300, 300, 400, 400, 300, 300, 500, 300, 300, 300, 400, 400, 300, 300, 500};

// --- Calibration ---
int baseline = 0;              // ambient noise
const int CALIBRATION_TIME = 3000; // ms
const int SAMPLE_COUNT = 32;       // samples for averaging

// Volume mapping
const int MIN_DUTY = 50;       // quiet buzzer
const int MAX_DUTY = 255;      // loud buzzer

void setup() {
  Serial.begin(115200);
  ledcAttach(BUZZER_PIN, 1000, buzzerResol); // initial freq 1kHz, resolution 8-bit
  ledcWrite(BUZZER_PIN, 0);

  // --- Calibrate ambient noise ---
  Serial.println("Calibrating ambient noise, stay quiet...");
  long sum = 0;
  int count = 0;
  unsigned long start = millis();
  while (millis() - start < CALIBRATION_TIME) {
    int val = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) val += analogRead(sound_sensor);
    val >>= 5;
    sum += val;
    count++;
    delay(10);
  }
  baseline = sum / count;
  Serial.print("Calibration complete. Baseline = ");
  Serial.println(baseline);
}

// --- Play music with dynamic volume ---
void playMusic() {
  int songLength = sizeof(songOneMelody) / sizeof(songOneMelody[0]);

  for (int i = 0; i < songLength; i++) {
    int freq = songOneMelody[i];
    int duration = songOneDurations[i];

    unsigned long startTime = millis();
    while (millis() - startTime < duration) {

      // --- Read sound sensor ---
      int soundValue = 0;
      for (int j = 0; j < SAMPLE_COUNT; j++) soundValue += analogRead(sound_sensor);
      soundValue >>= 5;

      // --- Map soundValue to buzzer PWM ---
      int duty = map(soundValue, baseline, 1023, MIN_DUTY, MAX_DUTY);
      duty = constrain(duty, MIN_DUTY, MAX_DUTY);

      ledcWrite(BUZZER_PIN, duty);
      ledcChangeFrequency(BUZZER_PIN, freq, buzzerResol);

      // --- Print values for Serial Plotter ---
      Serial.print("Sound:");
      Serial.print(soundValue);
      Serial.print("\tDuty:");
      Serial.println(duty);

      delay(10); // small delay to prevent overwhelming serial
    }
  }

  ledcWrite(BUZZER_PIN, 0); // stop buzzer
}

void loop() {
  playMusic();
}
