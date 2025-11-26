// ------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ------------------------------------------------------------------
// MACROS
// ------------------------------------------------------------------
#define BUZZER_PIN 5
#define SOUND_SENSOR A1
#define SDA 16
#define SCL 17

// ----------------------------------------------------------------------
// Global Variables and Data Structures
// ----------------------------------------------------------------------
// mode
enum Mode { AUTO_MODE, MANUAL_MODE };
Mode mode = AUTO_MODE; // default to auto mode

// buzzer setup
const int buzzerChannel = 0;
const int buzzerResol   = 8; 

// calibration
const int SAMPLE_COUNT = 32;

// Song struct
struct Song {
    const char* name;
    int* melody;
    int* durations;
    int length;
};

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

const Song songs[] = {
    { "Happy Birthday", HBDMelody, HBDDurations, sizeof(HBDMelody)/4 },
    { "Jingle Bells", jingleMelody, jingleDurations, sizeof(jingleMelody)/4 },
    { "Merry Xmas", merryMelody, merryDurations, sizeof(merryMelody)/4 },
    { "Silent Night", silentMelody, silentDurations, sizeof(silentMelody)/4 }
};


// Current state variables
volatile int volume;           // volume level (0-255)
SemaphoreHandle_t volumeMutex;
volatile int currentsong = 0; // current song index


// ----------------------------------------------------------------------
// Interrupts
// ----------------------------------------------------------------------

// TODO: REMOTE CONTROL 
    // a. Control Songs
    // b. Adjust Volume
    // c. Pause
void receivedRemoteSignal(){
    // adjust volume
    int volume = 5;
    changeVolume(volume);


    // adjust songs - map buttons
    // if click right -> go to next song
    nextSongHelper();
    // if click left -> go to the prev song
    prevSongHelper();

    // pause - map buttons
    // need a state to keep track of the state of the song
    // bool paused;
    // if pressed ok and the state is unpaused->pause
        // if(paused && button == ok)
    // if pressed ok and the state is pasued -> unpase
        // if(!paused && button == ok)
    
}


// ----------------------------------------------------------------------
// Tasks
// ----------------------------------------------------------------------

// ======== DATA COLLECT ========

// FreeRTOS Task to collect data from sound sensor in auto mode
void collectSoundDataTask(void *pvParameters){
    // collect data from sound sensor over a period of time
    // process data to determine appropriate volume level
    if(mode == AUTO_MODE){
        while(1){
            int soundValue = 0;
            
            // 1. collect data
            for (int j = 0; j < SAMPLE_COUNT; j++) soundValue += analogRead(sound_sensor);
            soundValue >>= 5;

            // 2. change volume if needed
            //  change if encountered convo
            // --- Map soundValue to buzzer PWM ---
            int targetDuty = map(soundValue, baseline, 1023, MIN_DUTY, MAX_DUTY);
            targetDuty = constrain(targetDuty, MIN_DUTY, MAX_DUTY);

            currentDuty = currentDuty + DUTY_ALPHA * (targetDuty - currentDuty);
            int dutyToWrite = (int)currentDuty;

            changeVolumeHelper(dutyToWrite);
        }
    } 
    TaskDelay(pdMS_TO_TICKS(100)); // yield 100ms
}


// ======== VOLUME ADJUST ==========

void changeVolumeHelper(int new_volume){
    // out of bounds check
    if(new_volume < 0) new_volume = 0;
    if(new_volume > 255) new_volume = 255;
    
    // assign new volume
    if(xSemaphoreTake(volumeMutex, portMAX_DELAY) == pdTRUE) {
        volume = new_volume;  // safe write
        xSemaphoreGive(volumeMutex);  // release mutex
    }
}

// ========== SONG HELPER ==========
// go to the next song
void nextSongHelper() {
  currentsong++;
  if (currentsong >= TOTAL_SONGS)
    currentsong = 0;

  Serial.print(">> Next Song: ");
  Serial.println(songs[currentsong].name);
}

// go back to the previous song
void prevSongHelper() {
  currentsong--;
  if (currentsong < 0)
    currentsong = TOTAL_SONGS - 1;

  Serial.print("<< Prev Song: ");
  Serial.println(songs[currentsong].name);
}


// ======== PLAY MUSIC ========
// Task to play music
void playMusicTask(void *pvParameters){
    // play music with volume set to 'volume' variable
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

        ledcWrite(BUZZER_PIN, dutyToWrite);
        ledcChangeFrequency(BUZZER_PIN, freq, buzzerResol);
        }
    }
}


// ======== LCD DISPLAY ========
LiquidCrystal_I2C lcd(0x27, 16, 2);

void displayOnLCDTask(void *pvParameters) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(songs[currentsong].name);

  lcd.setCursor(0, 1);
  lcd.print("Volume:");

  int currentVol;
  if(xSemaphoreTake(volumeMutex, portMAX_DELAY) == pdTRUE){
    currentVol = volume;  // safe read
    xSemaphoreGive(volumeMutex);
    }
    lcd.print(currentVol);

  lcd.setCursor(10,1);
  lcd.print(mode == AUTO_MODE ? "AUTO" : "MANUAL");
}

// ======== Setup and Loop ========
int calibration(){
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
  return baseline;
}

void setup(){
  Serial.begin(115200);

  Wire.begin(SDA, SCL);

  lcd.init();
  lcd.backlight();

  ledcAttach(buzzerChannel, 1000, buzzerResol);
  ledcWrite(buzzerChannel, 0);

  Serial.println("Calibrating ambient noise, stay quiet...");
  int baseline == calibration();
  Serial.print("Calibration complete. Baseline = ");
  Serial.println(baseline);

  volumeMutex = xSemaphoreCreateMutex();
  if (volumeMutex == NULL) {
    Serial.println("Failed to create mutex!");
  }

  // ===== Create FreeRTOS tasks =====
    xTaskCreatePinnedToCore(
        collectSoundDataTask,  // Task function
        "SoundData",            // Name
        2048,                   // Stack size
        NULL,                   // Parameters
        2,                      // Priority
        NULL,                   // Task handle
        0                       // Core 0
    );

    xTaskCreatePinnedToCore(
        playMusicTask,
        "PlayMusic",
        4096,
        NULL,
        2,
        NULL,
        1                       // Core 1
    );

    xTaskCreatePinnedToCore(
        displayOnLCDTask,
        "LCD",
        2048,
        NULL,
        1,
        NULL,
        1                       // Core 1
    );

    // TODO: music change task

}

void loop() {
    // nothing needed, tasks run independently
}
