// Adaptive Music Player
// author: Ruby Lee, Feier Long, 2025

// ==================================================================
// INCLUDES
// ==================================================================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "songs.h"  // Song definitions

#include "songs.h"  // Song definitions

// ==================================================================
// MACROS
// ==================================================================
#define BUZZER_PIN 5
#define SOUND_SENSOR 1
#define SDA 8
#define SCL 9
#define IR_PIN 18

// Remote codes
#define CODE_UP    0xFF18E7
#define CODE_DOWN  0xFF4AB5
#define CODE_LEFT  0xFF10EF
#define CODE_RIGHT 0xFF5AA5
#define CODE_STAR  0xFF6897
#define CODE_HASH  0xFFB04F

// ==================================================================
// GLOBAL VARIABLES
// ==================================================================

// --- Buzzer Configuration ---
const int buzzerChannel = 0;
const int buzzerResol = 8;

// --- Calibration & Volume Mapping ---
int baseline = 0;               
const int CALIBRATION_TIME = 3000; 
const int SAMPLE_COUNT_BITS = 5;  

const int MIN_DUTY = 50;       
const int MAX_DUTY = 255;      
const float PITCH_SCALE = 1.0;
float currentDuty = MAX_DUTY;
const float DUTY_ALPHA = 0.1f;

// -- IR Receiver --
IRrecv irrecv(IR_PIN);
decode_results results;

// IR debouncing
uint32_t lastCode = 0;
unsigned long lastTime = 0;
const unsigned long DEBOUNCE_DELAY = 150; // fast but prevents double triggers

QueueHandle_t irQueue;
enum IrEventType { IR_NEXT, IR_PREV, IR_VOL_UP, IR_VOL_DOWN, IR_MANUAL, IR_AUTO };

// --- Mode ---
enum Mode { AUTO_MODE, MANUAL_MODE };
Mode mode = AUTO_MODE;

// --- Shared State (Mutex Protected) ---
volatile int volume = MAX_DUTY;
SemaphoreHandle_t volumeMutex;
volatile int currentsong = 0;
SemaphoreHandle_t songMutex; 
volatile bool songChanged = false;
SemaphoreHandle_t songChangeMutex;

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
void IRDecodeTask(void *pvParameters);
void IRHandlerTask(void *pvParameters);

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
    if(xSemaphoreTake(songMutex, portMAX_DELAY) == pdTRUE){
        currentsong++;
        if(currentsong >= TOTAL_SONGS) currentsong = 0;
        Serial.print(">> Next Song: "); 
        Serial.println(songs[currentsong].name);
        xSemaphoreGive(songMutex);
    }

    // mark song change
    if(xSemaphoreTake(songChangeMutex, 0) == pdTRUE){
        songChanged = true;
        xSemaphoreGive(songChangeMutex);
    }
}

void prevSongHelper(){
    if(xSemaphoreTake(songMutex, portMAX_DELAY) == pdTRUE){
        currentsong--;
        if(currentsong < 0) currentsong = TOTAL_SONGS - 1;
        Serial.print("<< Prev Song: "); 
        Serial.println(songs[currentsong].name);
        xSemaphoreGive(songMutex);
    }

    // mark song change
    if(xSemaphoreTake(songChangeMutex, 0) == pdTRUE){
        songChanged = true;
        xSemaphoreGive(songChangeMutex);
    }
}


// --- Calibration ---
int calibration(){
    long sum = 0;
    int count = 0;
    unsigned long start = millis();
    
    while(millis() - start < CALIBRATION_TIME){
        int val = 0;
        for(int i = 0; i < (1 << SAMPLE_COUNT_BITS); i++){
            val += analogRead(SOUND_SENSOR);
        }
        val >>= SAMPLE_COUNT_BITS;  // divide by 32
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
            for(int j = 0; j < (1 << SAMPLE_COUNT_BITS); j++){
                soundValue += analogRead(SOUND_SENSOR);
            }
            soundValue >>= SAMPLE_COUNT_BITS;  // divide by 32

            // map to duty cycle range
            int targetDuty = map(soundValue, baseline, 4095, MIN_DUTY, MAX_DUTY);
            targetDuty = constrain(targetDuty, MIN_DUTY, MAX_DUTY);
            
            // smooth transition: currentDuty chases targetDuty
            currentDuty = currentDuty + DUTY_ALPHA * (targetDuty - currentDuty);
            
            // Format for serial plotter: label:value
            // Serial.print("CurrentDuty:");
            // Serial.println((int)currentDuty);
            
            // Update volume
            changeVolumeHelper((int)currentDuty);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- Task: Play Music ---
void playMusicTask(void *pvParameters){
    Serial.println("[PlayMusic] Task started");
    bool changed = false;

    while(1){
        // Get current song safely
        int songIndex;
        if(xSemaphoreTake(songMutex, portMAX_DELAY) == pdTRUE){
            songIndex = currentsong;
            xSemaphoreGive(songMutex);
        }
        
        Song song = songs[songIndex];
        
        Serial.print("[PlayMusic] Playing: ");
        Serial.println(song.name);
        
        // play each note in the song
        for(int i = 0; i < song.length; i++){

            // --- check for song change before playing note ---
            changed = false;
            if(xSemaphoreTake(songChangeMutex, 0) == pdTRUE){
                changed = songChanged;
                if(changed) songChanged = false; // reset flag
                xSemaphoreGive(songChangeMutex);
            }

            if(changed){
                Serial.println("[PlayMusic] Song change detected! Switching immediately.");
                break;  // exit note loop, reload new song
            }

            int freq = (int)(song.melody[i] * PITCH_SCALE);
            int dur = song.durations[i];

            unsigned long start = millis();
            while(millis() - start < dur){
                int vol;
                if(xSemaphoreTake(volumeMutex, portMAX_DELAY) == pdTRUE){
                    vol = volume;
                    xSemaphoreGive(volumeMutex);
                }

                ledcChangeFrequency(buzzerChannel, freq, buzzerResol);
                ledcWrite(buzzerChannel, vol);

                vTaskDelay(pdMS_TO_TICKS(10));
            }

            ledcWrite(buzzerChannel, 0);
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        
        // Song finished - move to next song automatically
        if(!changed){
            Serial.println("[PlayMusic] Song complete, moving to next...");
            nextSongHelper();
        }
        
        // pause before starting next song
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- Task: Display on LCD ---
void displayOnLCDTask(void *pvParameters){
    Serial.println("[LCD] Task started");

    while(1){
        lcd.clear();
        
        // Get current song safely
        int songIndex;
        if(xSemaphoreTake(songMutex, portMAX_DELAY) == pdTRUE){
            songIndex = currentsong;
            xSemaphoreGive(songMutex);
        }
        
        // Line 1: Song name
        lcd.setCursor(0, 0);
        lcd.print(songs[songIndex].name);

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

// --- Task: IR Decode ---
void IRDecodeTask(void *pvParameters){
    Serial.println("[IR] Task started");
    Serial.println("[IR] Waiting for IR signals...");
    
    int loopCount = 0;
    
    while(1){
        
        if (irrecv.decode(&results)) {
            Serial.println("[IR] Signal received!");
            
            uint32_t value = results.value;
            
            // Handle repeat code
            if (value == 0xFFFFFFFF) {
                value = lastCode;  // repeat last command
            } else {
                lastCode = value;  // store new command
            }
            
            unsigned long currentTime = millis();
            
            // check debounce
            if (currentTime - lastTime > DEBOUNCE_DELAY) {
                lastTime = currentTime;
                
                Serial.print("[IR] Received code: 0x");
                Serial.print(value, HEX);
                Serial.print(" -> ");

                IrEventType event;
                switch (value) {
                    case CODE_UP:
                        Serial.println("EVENT: Volume Up");
                        event = IR_VOL_UP;
                        xQueueSend(irQueue, &event, 0);
                        break;

                    case CODE_DOWN:
                        Serial.println("EVENT: Volume Down");
                        event = IR_VOL_DOWN;
                        xQueueSend(irQueue, &event, 0);
                        break;

                    case CODE_LEFT:
                        Serial.println("EVENT: Previous Song");
                        event = IR_PREV;
                        xQueueSend(irQueue, &event, 0);
                        break;

                    case CODE_RIGHT:
                        Serial.println("EVENT: Next Song");
                        event = IR_NEXT;
                        xQueueSend(irQueue, &event, 0);
                        break;

                    case CODE_HASH:
                        Serial.println("EVENT: Manual Mode (#)");
                        event = IR_MANUAL;
                        xQueueSend(irQueue, &event, 0);
                        break;

                    case CODE_STAR:
                        Serial.println("EVENT: Auto Mode (*)");
                        event = IR_AUTO;
                        xQueueSend(irQueue, &event, 0);
                        break;

                    default:
                        Serial.println("Unknown button");
                        break;
                }
            } else {
                Serial.println("[IR] Signal ignored (debounce)");
            }

            irrecv.resume(); // receive the next value
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // very short delay - checks IR every 10ms
    }
}

void IRHandlerTask(void *pvParameters){
    Serial.println("[IR Handler] Task started");

    IrEventType event;

    while (1){
        if (xQueueReceive(irQueue, &event, portMAX_DELAY) == pdTRUE){

            switch(event){

                case IR_VOL_UP:
                    // only allow volume up in MANUAL mode  
                    if (mode == MANUAL_MODE) {
                        changeVolumeHelper(volume + 5);
                    } else {
                        Serial.println("[IR Handler] Ignored: Volume UP disabled in AUTO mode");
                    }
                    break;

                case IR_VOL_DOWN:
                    if (mode == MANUAL_MODE) {
                        changeVolumeHelper(volume - 5);
                    } else {
                        Serial.println("[IR Handler] Ignored: Volume DOWN disabled in AUTO mode");
                    }
                    break;

                case IR_NEXT:
                    nextSongHelper();
                    break;

                case IR_PREV:
                    prevSongHelper();
                    break;

                case IR_MANUAL:
                    // directly switch to manual mode
                    // it's okay to not have a mutex beecause only this task modifies mode
                    // and other tasks only read it
                    mode = MANUAL_MODE;
                    break;

                case IR_AUTO:
                    mode = AUTO_MODE;
                    break;
            }
        }
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
    ledcAttachPin(BUZZER_PIN, buzzerChannel);
    ledcWrite(buzzerChannel, 0);
    
    // --- IR Receiver Setup ---
    irrecv.enableIRIn();
    Serial.println("IR Receiver enabled");

    // --- IR Event Queue ---
    irQueue = xQueueCreate(10, sizeof(IrEventType));
    if (irQueue == NULL) {
        Serial.println("ERROR: Failed to create IR queue!");
        lcd.clear();
        lcd.print("Queue Error!");
        while(1);
    }

    // --- Calibration ---
    Serial.println("Calibrating ambient noise...");
    lcd.setCursor(0, 1);
    lcd.print("Calibrating...");
    
    baseline = calibration();
    
    Serial.print("Calibration complete. Baseline = "); 
    Serial.println(baseline);

    // --- Mutex Creation ---
    volumeMutex = xSemaphoreCreateMutex();
    songMutex = xSemaphoreCreateMutex();
    songChangeMutex = xSemaphoreCreateMutex();
    
    if(volumeMutex == NULL || songMutex == NULL || songChangeMutex == NULL){
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

    // IR decode task on core 0
    xTaskCreatePinnedToCore(
        IRDecodeTask,           // Task function
        "IR_Decode",            // Task name
        4096,                   // Stack size
        NULL,                   // Parameters
        1,                      // Priority
        NULL,                   // Task handle
        0                       // Core 0
    );

    xTaskCreatePinnedToCore(
        IRHandlerTask,
        "IR_Handler",
        4096,
        NULL,
        2,     // higher priority than decode
        NULL,
        0
    );

    // core 1 handles music playback
    xTaskCreatePinnedToCore(
        playMusicTask,          // Task function
        "PlayMusic",            // Task name
        4096,                   // Stack size
        NULL,                   // Parameters
        2,                      // Priority
        NULL,                   // Task handle
        1                       // Core 1
    );
    
    Serial.println("=== System Initialized ===");
}

void loop(){
    // All work is done by FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}