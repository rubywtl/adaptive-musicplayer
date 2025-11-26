#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ------------------------------------------------------------------
// MACROS
// ------------------------------------------------------------------
#define BUZZER_PIN 5
#define SOUND_SENSOR A1
#define SDA 16
#define SCL 17

// ------------------------------------------------------------------
// GLOBALS
// ------------------------------------------------------------------

// current song index
volatile int currentsong = 0;

// buzzer setup
const int buzzerChannel = 0;
const int buzzerResol   = 8;   // 8-bit PWM

// Song struct
struct Song {
    const char* name;
    int* melody;
    int* durations;
    int length;
};

// ------------------------------------------------------------------
// SONGS
// ------------------------------------------------------------------

// HAPPY BIRTHDAY
int HBDMelody[] = {
  264, 264, 297, 264, 352, 330,
  264, 264, 297, 264, 396, 352,
  264, 264, 528, 440, 352, 330, 297,
  466, 466, 440, 352, 396, 352
};

int HBDDurations[] = {
  250,125,500,500,500,1000,
  250,125,500,500,500,1000,
  250,125,500,500,500,500,1000,
  250,125,500,500,500,1000
};

// JINGLE BELLS
int jingleMelody[] = {
  330,330,330, 330,330,330, 330,392,262,294,330,
  349,349,349,349,349,330,330,330,330,294,294,330,294,392
};

int jingleDurations[] = {
  250,250,500, 250,250,500, 250,250,250,250,500,
  250,250,375,125,500, 250,250,375,125,250,250,250,250,750
};

// WE WISH YOU A MERRY CHRISTMAS
int merryMelody[] = {
  392,392,440,392,523,494,
  392,392,440,392,587,523,
  392,392,784,659,523,494,440,
  698,698,659,523,587,523
};

int merryDurations[] = {
  300,300,300,300,300,600,
  300,300,300,300,300,600,
  300,300,300,300,300,300,600,
  300,300,300,300,300,600
};

// SILENT NIGHT
int silentMelody[] = {
  392,392,440,392,494,466,
  392,392,440,392,523,494,
  392,392,784,659,523,494,440,
  698,698,659,523,587,523
};

int silentDurations[] = {
  600,600,700,700,700,700,
  600,600,700,700,700,700,
  600,600,700,700,700,700,900,
  700,700,700,700,700,900
};

// PLAYLIST
const Song songs[] = {
    { "Happy Birthday", HBDMelody, HBDDurations, sizeof(HBDMelody)/4 },
    { "Jingle Bells", jingleMelody, jingleDurations, sizeof(jingleMelody)/4 },
    { "Merry Xmas", merryMelody, merryDurations, sizeof(merryMelody)/4 },
    { "Silent Night", silentMelody, silentDurations, sizeof(silentMelody)/4 }
};

const int TOTAL_SONGS = sizeof(songs) / sizeof(Song);

// Sound calibration
int baseline = 0;
const int CALIBRATION_TIME = 3000;
const int SAMPLE_COUNT = 32;
const int MIN_DUTY = 50;
const int MAX_DUTY = 255;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// LCD DISPLAY
void showSongOnLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(songs[currentsong].name);

  lcd.setCursor(0, 1);
  lcd.print("Volume:");
  lcd.print(baseline);
}

// CHANGE SONG
void changeSong() {
  currentsong++;
  if (currentsong >= TOTAL_SONGS)
    currentsong = 0;

  Serial.print("Next Song -> ");
  Serial.println(songs[currentsong].name);

  showSongOnLCD();
}

// PLAY SONG WITH R E S T S
void playFromCurrentSong() {
  const Song& s = songs[currentsong];

  for (int i = 0; i < s.length; i++) {
    int freq = s.melody[i];
    int duration = s.durations[i];

    // ------ play note ------
    unsigned long start = millis();
    while (millis() - start < duration) {

      // read sound sensor
      int soundValue = 0;
      for (int j = 0; j < SAMPLE_COUNT; j++)
        soundValue += analogRead(SOUND_SENSOR);
      soundValue >>= 5;

      // map to buzzer duty
      int duty = map(soundValue, baseline, 1023, MIN_DUTY, MAX_DUTY);
      duty = constrain(duty, MIN_DUTY, MAX_DUTY);

      ledcWrite(BUZZER_PIN, duty);
      ledcChangeFrequency(BUZZER_PIN, freq, buzzerResol);

      Serial.print("Sound:");
      Serial.print(soundValue);
      Serial.print("\tDuty:");
      Serial.println(duty);

      delay(5);
    }

    // ------ REST after each note ------
    ledcWrite(buzzerChannel, 0);
    delay(120);  // **makes music MUCH smoother**
  }
}

// ------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  Wire.begin(SDA, SCL);

  lcd.init();
  lcd.backlight();

  ledcAttach(buzzerChannel, 1000, buzzerResol);
  ledcWrite(buzzerChannel, 0);

  // ---- calibrate ambient ----
  Serial.println("Calibrating...");
  long sum = 0; int n = 0;
  unsigned long start = millis();

  while (millis() - start < CALIBRATION_TIME) {
    int v = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++)
      v += analogRead(SOUND_SENSOR);
    v >>= 5;

    sum += v; n++;
    delay(10);
  }

  baseline = sum / n;

  Serial.print("Baseline = ");
  Serial.println(baseline);

  showSongOnLCD();
}

// rotate through the song list
void loop() {
  playFromCurrentSong();

  ledcWrite(buzzerChannel, 0);
  delay(1200);

  changeSong();
}
