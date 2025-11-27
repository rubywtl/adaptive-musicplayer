// ==================================================================
// INCLUDES
// ==================================================================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ==================================================================
// MACROS
// ==================================================================
#define BUZZER_PIN 5
#define SOUND_SENSOR 1
#define SDA 8
#define SCL 9

// ==================================================================
// GLOBAL VARIABLES
// ==================================================================

// --- Buzzer Configuration ---
const int buzzerChannel = 0;
const int buzzerResol = 8;

// --- Calibration & Volume Mapping ---
int baseline = 0;               
const int CALIBRATION_TIME = 3000; 
const int SAMPLE_COUNT = 32;       
const int MIN_DUTY = 50;       
const int MAX_DUTY = 255;      
const float PITCH_SCALE = 1.0;
float currentDuty = MAX_DUTY;
const float DUTY_ALPHA = 0.1f;

// --- Mode ---
enum Mode { AUTO_MODE, MANUAL_MODE };
Mode mode = AUTO_MODE;

// --- Shared State (Mutex Protected) ---
volatile int volume = MAX_DUTY;
SemaphoreHandle_t volumeMutex;
volatile int currentsong = 0;

// --- Song Data Structure ---
struct Song {
    const char* name;
    int* melody;
    int* durations;
    int length;
};

// --- Song Arrays ---
int HBDMelody[] = {264,264,297,264,352,330,264,264,297,264,396,352,264,264,528,440,352,330,297,466,466,440,352,396,352};
int HBDDurations[] = {250,125,500,500,500,1000,250,125,500,500,500,1000,250,125,500,500,500,500,1000,250,125,500,500,500,1000};

int jingleMelody[] = {330,330,330, 330,330,330, 330,392,262,294,330, 349,349,349,349,349,330,330,330,330,294,294,330,294,392};
int jingleDurations[] = {250,250,500, 250,250,500, 250,250,250,250,500, 250,250,375,125,500, 250,250,375,125,250,250,250,250,750};

int merryMelody[] = {392,392,440,392,523,494, 392,392,440,392,587,523, 392,392,784,659,523,494,440, 698,698,659,523,587,523};
int merryDurations[] = {300,300,300,300,300,600, 300,300,300,300,300,600, 300,300,300,300,300,300,600, 300,300,300,300,300,600};

int silentMelody[] = {392,392,440,392,494,466, 392,392,440,392,523,494, 392,392,784,659,523,494,440, 698,698,659,523,587,523};
int silentDurations[] = {600,600,700,700,700,700, 600,600,700,700,700,700, 600,600,700,700,700,700,900, 700,700,700,700,700,900};

const int TOTAL_SONGS = 4;
const Song songs[TOTAL_SONGS] = {
    {"Happy Birthday", HBDMelody, HBDDurations, sizeof(HBDMelody)/sizeof(int)},
    {"Jingle Bells", jingleMelody, jingleDurations, sizeof(jingleMelody)/sizeof(int)},
    {"Merry Xmas", merryMelody, merryDurations, sizeof(merryMelody)/sizeof(int)},
    {"Silent Night", silentMelody, silentDurations, sizeof(silentMelody)/sizeof(int)}
};

// --- LCD Object ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==================================================================
// FUNCTION DECLARATIONS
// ==================================================================
void changeVolumeHelper(int new_volume);
void nextSongHelper();
void prevSongHelper();
int calibration();

// Task Declarations
void collectSoundDataTask(void *pvParameters);
void playMusicTask(void *pvParameters);
void displayOnLCDTask(void *pvParameters);

// ==================================================================
// FUNCTION IMPLEMENTATIONS
// ==================================================================

// --- Volume Control ---
void changeVolumeHelper(int new_volume){
    if(new_volume < MIN_DUTY) new_volume = MIN_DUTY;
    if(new_volume > MAX_DUTY) new_volume = MAX_DUTY;

    if(xSemaphoreTake(volumeMutex, portMAX_DELAY) == pdTRUE){
        volume = new_volume;
        xSemaphoreGive(volumeMutex);
    }
}

// --- Song Navigation ---
void nextSongHelper(){
    currentsong++;
    if(currentsong >= TOTAL_SONGS) currentsong = 0;
    Serial.print(">> Next Song: "); 
    Serial.println(songs[currentsong].name);
}

void prevSongHelper(){
    currentsong--;
    if(currentsong < 0) currentsong = TOTAL_SONGS - 1;
    Serial.print("<< Prev Song: "); 
    Serial.println(songs[currentsong].name);
}

// --- Calibration ---
int calibration(){
    long sum = 0;
    int count = 0;
    unsigned long start = millis();
    
    while(millis() - start < CALIBRATION_TIME){
        int val = 0;
        for(int i = 0; i < SAMPLE_COUNT; i++){
            val += analogRead(SOUND_SENSOR);
        }
        val /= SAMPLE_COUNT; 
        sum += val;
        count++;
        delay(10);
    }
    
    baseline = sum / count;
    
    // sanity check
    if(baseline >= 4000) baseline = 2000;
    if(baseline <= 10) baseline = 100; 
    
    return baseline;
}


// ==================================================================
// FREERTOS TASKS
// ==================================================================

// --- Task: Collect Sound Data (Auto Mode) ---
void collectSoundDataTask(void *pvParameters){
    printf("data collection task\n");

    while(1){
        // only collects sound data in AUTO mode
        if(mode == AUTO_MODE){
            int soundValue = 0;
            
            // sample sound sensor
            for(int j = 0; j < SAMPLE_COUNT; j++){
                soundValue += analogRead(SOUND_SENSOR);
            }
            soundValue >>= 5;  // divide by 32

            // map to duty cycle range
            // - instead of setting duty cycles directly, use smoothing
            // - this makes volume changes less abrupt
            int targetDuty = map(soundValue, baseline, 4095, MIN_DUTY, MAX_DUTY);
            targetDuty = constrain(targetDuty, MIN_DUTY, MAX_DUTY);
            
            // smooth transition: currentDuty chases targetDuty
            currentDuty = currentDuty + DUTY_ALPHA * (targetDuty - currentDuty);
            
            Serial.print("Current duty: ");
            Serial.println((int)currentDuty);
            
            // Update volume
            changeVolumeHelper((int)currentDuty);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- Task: Play Music ---
void playMusicTask(void *pvParameters){
    printf("play music task\n");

    while(1){
        Song song = songs[currentsong];
        
        Serial.print("Playing: ");
        Serial.println(song.name);
        
        // play each note in the song
        for(int i = 0; i < song.length; i++){
            int freq = (int)(song.melody[i] * PITCH_SCALE);
            int dur = song.durations[i];
            
            // play note for its duration
            unsigned long start = millis();
            while(millis() - start < dur){
                // read volume safely
                int vol;
                if(xSemaphoreTake(volumeMutex, portMAX_DELAY) == pdTRUE){
                    vol = volume;
                    xSemaphoreGive(volumeMutex);
                }
                
                // set frequency and volume
                ledcChangeFrequency(buzzerChannel, freq, buzzerResol);
                ledcWrite(buzzerChannel, vol);
                
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            // silence between notes
            ledcWrite(buzzerChannel, 0);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        
        // pause before repeating song
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- Task: Display on LCD ---
void displayOnLCDTask(void *pvParameters){
    printf("display task\n");

    while(1){
        lcd.clear();
        
        // Line 1: Song name
        lcd.setCursor(0, 0);
        lcd.print(songs[currentsong].name);

        // Line 2: Volume and mode
        lcd.setCursor(0, 1);
        lcd.print("Vol:");
        
        int currentVol;
        if(xSemaphoreTake(volumeMutex, portMAX_DELAY) == pdTRUE){
            currentVol = volume;
            xSemaphoreGive(volumeMutex);
        }
        lcd.print(currentVol);

        lcd.setCursor(10, 1);
        lcd.print(mode == AUTO_MODE ? "AUTO" : "MANUAL");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ==================================================================
// SETUP & LOOP
// ==================================================================

void setup(){
    // --- Serial Communication ---
    Serial.begin(115200);
    Serial.println("\n=== Music Player Starting ===");

    // --- I2C & LCD ---
    Wire.begin(SDA, SCL);
    lcd.init(); 
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    
    // --- Buzzer Setup ---
    ledcAttach(BUZZER_PIN, 1000, buzzerResol);
    ledcWrite(buzzerChannel, 0);

    // --- Calibration ---
    Serial.println("Calibrating ambient noise...");
    lcd.setCursor(0, 1);
    lcd.print("Calibrating...");
    
    baseline = calibration();
    
    Serial.print("Calibration complete. Baseline = "); 
    Serial.println(baseline);

    // --- Mutex Creation ---
    volumeMutex = xSemaphoreCreateMutex();
    if(volumeMutex == NULL){
        Serial.println("ERROR: Failed to create mutex!");
        lcd.clear();
        lcd.print("Mutex Error!");
        while(1);  // Halt
    }

    // --- Create FreeRTOS Tasks ---

    // core 0 handles all the I/O tasks (sound data collection, LCD display)
    xTaskCreatePinnedToCore(
        collectSoundDataTask,   // Task function
        "SoundData",            // Task name
        4096,                   // Stack size
        NULL,                   // Parameters
        2,                      // Priority
        NULL,                   // Task handle
        0                       // Core 0
    );

    xTaskCreatePinnedToCore(
        displayOnLCDTask,       // Task function
        "LCD",                  // Task name
        4096,                   // Stack size
        NULL,                   // Parameters
        1,                      // Priority
        NULL,                   // Task handle
        0                       // Core 0
    );

    // core 1 handles music playback
    // this task is CPU intensive due to frequent frequency changes
    // and also it needs to be smooth for music quality
    xTaskCreatePinnedToCore(
        playMusicTask,          // Task function
        "PlayMusic",            // Task name
        4096,                   // Stack size
        NULL,                   // Parameters
        2,                      // Priority
        NULL,                   // Task handle
        1                       // Core 1
    );


    // TODO: Initialize change songs
    // - only allow change when user is authenticated
    // xTaskCreatePinnedToCore(
    //     changeSongTask,       // Task function
    //     "ChangeSong",                  // Task name
    //     4096,                   // Stack size
    //     NULL,                   // Parameters
    //     1,                      // Priority
    //     NULL,                   // Task handle
    //     1                       // Core 1
    // );
    
    Serial.println("=== System Initialized ===");
}

void loop(){
    // All work is done by FreeRTOS tasks
    // vTaskDelay(pdMS_TO_TICKS(1000));
}