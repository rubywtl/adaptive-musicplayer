/**
 * @file adaptive_music_player.ino
 * @brief Adaptive Music Player using ESP32, FreeRTOS, IR remote, RFID, LCD, and buzzer.
 *
 * This application implements:
 *  - Auto volume control based on a sound sensor
 *  - Manual volume and song navigation via IR remote
 *  - RFID-based authentication gate for manual control
 *  - Multi-tasking with FreeRTOS on dual-core ESP32
 *
 * @author Ruby Lee
 * @author Feier Long
 * @date 2025
 */

// ==================================================================
// INCLUDES
// ==================================================================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <MFRC522.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "songs.h"  ///< Song definitions (melodies, durations, names, TOTAL_SONGS)

// ==================================================================
// MACROS
// ==================================================================
/// Buzzer output pin
#define BUZZER_PIN 5
/// Analog sound sensor input channel
#define SOUND_SENSOR 1
/// I2C SDA pin
#define SDA 8
/// I2C SCL pin
#define SCL 9
/// IR receiver input pin
#define IR_PIN 18

/// RFID SS (SDA) pin
#define SS_PIN 10
/// RFID RST pin
#define RST_PIN 3

// Remote codes (example NEC codes)
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
/** @brief LEDC channel for the buzzer PWM. */
const int buzzerChannel = 0;
/** @brief LEDC resolution in bits. */
const int buzzerResol = 8;

// --- Calibration & Volume Mapping ---
/** @brief Quiet ambient baseline value from the sound sensor. */
int baseline = 0;
/** @brief Loud reference value from the sound sensor. */
int loudLevel = 3000;
/** @brief Calibration duration per phase in milliseconds. */
const int CALIBRATION_TIME = 3000;
/** @brief Number of samples as 2^SAMPLE_COUNT_BITS for averaging. */
const int SAMPLE_COUNT_BITS = 5;

/** @brief Minimum duty cycle for buzzer volume. */
const int MIN_DUTY = 0;
/** @brief Maximum duty cycle for buzzer volume. */
const int MAX_DUTY = 5;
/** @brief Global pitch scale factor applied to song frequencies. */
const float PITCH_SCALE = 1.0;
/** @brief Smoothed current duty cycle used by auto volume tracking. */
float currentDuty = MAX_DUTY;
/** @brief Smoothing factor for exponential moving average of duty cycle. */
const float DUTY_ALPHA = 0.1f;

// -- IR Receiver --
/** @brief IR receiver object. */
IRrecv irrecv(IR_PIN);
/** @brief Decoded IR results structure. */
decode_results results;

/** @brief Last decoded IR code for handling repeat frames. */
uint32_t lastCode = 0;
/** @brief Timestamp of last accepted IR code (for debouncing). */
unsigned long lastTime = 0;
/** @brief Debounce delay in milliseconds for IR input. */
const unsigned long DEBOUNCE_DELAY = 150;

/**
 * @brief IR event types pushed from decode task to handler task.
 */
enum IrEventType {
    IR_NEXT,    ///< Next song request
    IR_PREV,    ///< Previous song request
    IR_VOL_UP,  ///< Volume up request
    IR_VOL_DOWN,///< Volume down request
    IR_MANUAL,  ///< Switch to manual mode
    IR_AUTO     ///< Switch to auto mode
};

/** @brief FreeRTOS queue for passing IR events. */
QueueHandle_t irQueue;

// --- Mode ---
/**
 * @brief Player mode.
 */
enum Mode {
    AUTO_MODE,   ///< Auto volume based on sound sensor
    MANUAL_MODE  ///< Manual volume via IR remote
};

/** @brief Current operating mode (AUTO or MANUAL). */
Mode mode = AUTO_MODE;

// --- Shared State (Mutex Protected) ---
/** @brief Current volume (duty cycle), protected by @ref volumeMutex. */
volatile int volume = MAX_DUTY;
/** @brief Mutex protecting access to @ref volume. */
SemaphoreHandle_t volumeMutex;

/** @brief Index of the current song, protected by @ref songMutex. */
volatile int currentsong = 0;
/** @brief Mutex protecting access to @ref currentsong. */
SemaphoreHandle_t songMutex;

/** @brief Flag indicating that the song should change immediately. */
volatile bool songChanged = false;
/** @brief Mutex protecting access to @ref songChanged. */
SemaphoreHandle_t songChangeMutex;

// --- LCD Object ---
/** @brief Global 16x2 I2C LCD instance. */
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- RFID Authentication State ---
/** @brief Absolute expiration time (ms) of current RFID authentication. */
unsigned long authExpiration = 0;
/** @brief Current authentication active state. */
volatile bool authActive = false;

/** @brief Known-good RFID UID. */
byte GOOD_UID[] = {0x84, 0x1E, 0xB0, 0x02};
/** @brief Length of known-good RFID UID. */
const byte GOOD_UID_SIZE = 4;

// --- RFID Object --
/** @brief RFID reader object. */
MFRC522 rfid(SS_PIN, RST_PIN);

// ==================================================================
// FUNCTION DECLARATIONS
// ==================================================================
/**
 * @brief Safely change the global volume (duty cycle) with bounds checking.
 *
 * @param new_volume Desired volume value before clamping to [MIN_DUTY, MAX_DUTY].
 */
void changeVolumeHelper(int new_volume);

/**
 * @brief Advance to the next song in the playlist and mark a song change.
 */
void nextSongHelper();

/**
 * @brief Go to the previous song in the playlist and mark a song change.
 */
void prevSongHelper();

/**
 * @brief Perform two-phase calibration for quiet and loud sound levels.
 *
 * Uses the sound sensor to determine @ref baseline and @ref loudLevel.
 * Displays instructions on the LCD during calibration.
 */
void calibration();

/**
 * @brief Compare a given UID with the known-good UID.
 *
 * @param uid Pointer to the UID byte array to compare.
 * @param uidSize Number of bytes in the provided UID.
 * @return true if the UID matches the known-good UID; false otherwise.
 */
bool uidMatch(const byte* uid, byte uidSize);

// Task Declarations
/**
 * @brief FreeRTOS task: collect sound data and update volume in AUTO mode.
 *
 * @param pvParameters Unused task parameter.
 */
void collectSoundDataTask(void *pvParameters);

/**
 * @brief FreeRTOS task: play music using LEDC PWM on the buzzer.
 *
 * @param pvParameters Unused task parameter.
 */
void playMusicTask(void *pvParameters);

/**
 * @brief FreeRTOS task: update the LCD with song info, volume, mode, and auth timer.
 *
 * @param pvParameters Unused task parameter.
 */
void displayOnLCDTask(void *pvParameters);

/**
 * @brief FreeRTOS task: decode IR signals and post events to the IR queue.
 *
 * @param pvParameters Unused task parameter.
 */
void IRDecodeTask(void *pvParameters);

/**
 * @brief FreeRTOS task: process IR events and modify global state (volume, song, mode).
 *
 * @param pvParameters Unused task parameter.
 */
void IRHandlerTask(void *pvParameters);

/**
 * @brief FreeRTOS task: manage RFID authentication state and expiration.
 *
 * @param pvParameters Unused task parameter.
 */
void RFIDTask(void *pvParameters);

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
void calibration(){
    int quietSum = 0;
    int loudSum = 0;
    int count = 0;

    // Phase 1: Capture quiet baseline
    Serial.println("Phase 1: Stay QUIET for 3 seconds...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Stay QUIET!");
    lcd.setCursor(0, 1);
    lcd.print("Calibrating...");

    unsigned long start = millis();
    while(millis() - start < CALIBRATION_TIME){
        int val = 0;
        for(int i = 0; i < (1 << SAMPLE_COUNT_BITS); i++){
            val += analogRead(SOUND_SENSOR);
        }
        val >>= SAMPLE_COUNT_BITS;
        quietSum += val;
        count++;
        delay(10);
    }

    int quietBaseline = quietSum / count;

    // Phase 2: Capture loud talking
    Serial.println("Phase 2: TALK LOUDLY close to sensor for 3 seconds...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TALK LOUDLY!");
    lcd.setCursor(0, 1);
    lcd.print("Near sensor...");

    count = 0;
    start = millis();
    while(millis() - start < CALIBRATION_TIME){
        int val = 0;
        for(int i = 0; i < (1 << SAMPLE_COUNT_BITS); i++){
            val += analogRead(SOUND_SENSOR);
        }
        val >>= SAMPLE_COUNT_BITS;
        loudSum += val;
        count++;
        delay(10);
    }

    int loudBaseline = loudSum / count;

    // Store values
    baseline = quietBaseline;

    // add some margin to the loud level for headroom
    if(loudBaseline > 3900){
        loudLevel = min(4095, 3500); // cap if it is not loud enough
    }
    loudLevel = min(4095, loudBaseline + 50);

    Serial.print("Calibration complete. Quiet = ");
    Serial.print(baseline);
    Serial.print(", Loud = ");
    Serial.println(loudLevel);
}

// --- Authentication---
bool uidMatch(const byte* uid, byte uidSize){
    if(uidSize != GOOD_UID_SIZE) return false;

    for(byte i = 0; i < GOOD_UID_SIZE; i++){
        if(uid[i] != GOOD_UID[i]) return false;
    }
    return true;
}

// ==================================================================
// FREERTOS TASKS
// ==================================================================

// --- Task: Collect Sound Data (Auto Mode) ---
void collectSoundDataTask(void *pvParameters){
    (void)pvParameters;
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
            int targetDuty = map(soundValue, baseline, loudLevel, MAX_DUTY, MIN_DUTY);
            targetDuty = constrain(targetDuty, MIN_DUTY, MAX_DUTY);

            // smooth transition: currentDuty chases targetDuty
            currentDuty = currentDuty + DUTY_ALPHA * (targetDuty - currentDuty);

            // Update volume
            changeVolumeHelper((int)currentDuty);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- Task: Play Music ---
void playMusicTask(void *pvParameters){
    (void)pvParameters;
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
            while(millis() - start < (unsigned long)dur){
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
    (void)pvParameters;
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

        // Line 2: Volume, mode, and auth timer
        lcd.setCursor(0, 1);
        lcd.print("V:");

        int currentVol;
        if(xSemaphoreTake(volumeMutex, portMAX_DELAY) == pdTRUE){
            currentVol = volume;
            xSemaphoreGive(volumeMutex);
        }
        lcd.print(currentVol);
        lcd.print(" ");

        // Mode: A = AUTO | M = MANUAL
        lcd.print(mode == AUTO_MODE ? "Auto" : "Manual");
        lcd.print(" ");

        if(authActive){
            long remaining = (long)(authExpiration - millis());
            if (remaining < 0) remaining = 0;
            int sec = remaining / 1000;

            lcd.print(sec);
            lcd.print("s");
        } else {
            // clear leftover chars if auth inactive
            lcd.print(" x ");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --- Task: IR Decode ---
void IRDecodeTask(void *pvParameters){
    (void)pvParameters;
    Serial.println("[IR] Task started");
    Serial.println("[IR] Waiting for IR signals...");

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

// --- Task: IR Handler ---
void IRHandlerTask(void *pvParameters){
    (void)pvParameters;
    Serial.println("[IR Handler] Task started");

    IrEventType event;

    while (1){
        if (xQueueReceive(irQueue, &event, portMAX_DELAY) == pdTRUE){

            // if not authenticated - ignore everything
            if (!authActive) {
                Serial.println("[IR Handler] Ignored: not authenticated");
                continue;
            }

            switch(event){

                case IR_VOL_UP:
                    // only allow volume up in MANUAL mode
                    if (mode == MANUAL_MODE) {
                        changeVolumeHelper(volume + 1);
                    } else {
                        Serial.println("[IR Handler] Ignored: Volume UP disabled in AUTO mode");
                    }
                    break;

                case IR_VOL_DOWN:
                    if (mode == MANUAL_MODE) {
                        changeVolumeHelper(volume - 1);
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
                    // it's okay to not have a mutex because only this task modifies mode
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

// --- Task: RFID Authentication ---
void RFIDTask(void *pvParameters) {
    (void)pvParameters;
    Serial.println("[RFID] Task started");

    while(1){
        if(authActive){
            long remaining = (long)(authExpiration - millis());
            // Handles Timeout
            if(remaining <= 0) {
                authActive = false;
                Serial.println("[RFID] Auth expired - returning to AUTO mode");
            }
        }

        if(rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()){
            if(uidMatch(rfid.uid.uidByte, rfid.uid.size)){
                Serial.println("[RFID] Authenticated card");

                authActive = true;
                authExpiration = millis() + 30000;
            } else {
                Serial.println("[RFID] Not Authenticated");
            }

            rfid.PICC_HaltA();
            rfid.PCD_StopCrypto1();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==================================================================
// SETUP & LOOP
// ==================================================================

/**
 * @brief Arduino setup function: initializes peripherals, RTOS objects, and tasks.
 *
 * Responsibilities:
 *  - Initialize Serial, I2C, LCD, buzzer (LEDC), IR receiver, RFID (SPI)
 *  - Perform calibration for sound sensor
 *  - Create FreeRTOS mutexes and queue
 *  - Create and pin tasks to appropriate cores
 */
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

    // --- RFID Setup ---
    SPI.begin(36, 37, 35, SS_PIN);  // SCK, MISO, MOSI, SS
    rfid.PCD_Init();

    // --- Calibration ---
    Serial.println("Calibrating ambient noise...");
    lcd.setCursor(0, 1);
    lcd.print("Calibrating...");
    calibration();

    Serial.print("Calibration complete. ");

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

    // RFID task on core 0
    xTaskCreatePinnedToCore(
        RFIDTask,
        "RFID",
        4096,
        NULL,
        3,     // higher priority than everything
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

/**
 * @brief Arduino main loop function.
 *
 * All work is performed in FreeRTOS tasks; loop() remains idle.
 */
void loop(){
    // All work is done by FreeRTOS tasks
    // vTaskDelay(pdMS_TO_TICKS(1000));
}
