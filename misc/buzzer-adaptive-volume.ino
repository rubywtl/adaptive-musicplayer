#define BUZZER_PIN 5
int sound_sensor = 1;

const int buzzerResol = 8;      // 8-bit resolution
const int buzzerChannel = 0;

// --- Happy Birthday Melody ---
int songMelody[] = {
  264, 264, 297, 264, 352, 330,  // Happy Birthday to You
  264, 264, 297, 264, 396, 352,  // Happy Birthday to You
  264, 264, 528, 440, 352, 330, 297, // Happy Birthday Dear [Name]
  466, 466, 440, 352, 396, 352   // Happy Birthday to You
};

// Note durations (ms)
int songDurations[] = {
  250, 125, 500, 500, 500, 1000,
  250, 125, 500, 500, 500, 1000,
  250, 125, 500, 500, 500, 500, 1000,
  250, 125, 500, 500, 500, 1000
};

// --- Calibration ---
int baseline = 0;              
const int CALIBRATION_TIME = 3000; 
const int SAMPLE_COUNT = 32;       

// Volume mapping
const int MIN_DUTY = 50;       
const int MAX_DUTY = 255;      

const float PITCH_SCALE = 3.5;

// Smoothing
float currentDuty = MAX_DUTY; // start loud
const float DUTY_ALPHA = 0.1f;

void setup() {
  Serial.begin(115200);
  int val = ledcSetup(buzzerChannel, 1000, buzzerResol);
  printf("Setup Val: %d\n", val);
  ledcAttachPin(BUZZER_PIN, buzzerChannel);
  ledcWrite(buzzerChannel, 0);

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
  int songLength = sizeof(songMelody) / sizeof(songMelody[0]);

  for (int i = 0; i < songLength; i++) {
    int baseFreq = songMelody[i];
    int freq = (int)(baseFreq * PITCH_SCALE);
    int duration = songDurations[i];

    unsigned long startTime = millis();
    while (millis() - startTime < duration) {

      // --- Read sound sensor ---
      int soundValue = 0;
      for (int j = 0; j < SAMPLE_COUNT; j++) soundValue += analogRead(sound_sensor);
      soundValue >>= 5;

      // --- Map soundValue to buzzer PWM ---
      int targetDuty = map(soundValue, baseline, 1023, MIN_DUTY, MAX_DUTY);
      targetDuty = constrain(targetDuty, MIN_DUTY, MAX_DUTY);

      // exponential smoothing -> make currentDuty chase targetDuty 
      // small alpha(smooth)
      // big alpha (very sensitive)
      currentDuty = currentDuty + DUTY_ALPHA * (targetDuty - currentDuty);
      int dutyToWrite = (int)currentDuty;
      
      ledcWrite(buzzerChannel, dutyToWrite);
      int ret = ledcChangeFrequency(buzzerChannel, freq, buzzerResol);

      // printf("freq given: %d, returned freq %d\n", freq, ret);

      --- Print values for Serial Plotter ---
      Serial.print("Min:0 Max:1023 "); // keep min/max for plotter
      Serial.print("Sound:");
      Serial.print(soundValue);
      Serial.print("\tTargetDuty:");
      Serial.print(targetDuty);
      Serial.print("\tSmoothDuty:");
      Serial.println(dutyToWrite);

      delay(10); // small delay to prevent overwhelming serial
    }
  }

  ledcWrite(BUZZER_PIN, 0); // stop buzzer
}

void loop() {
  playMusic();
}
