#include <Arduino.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>

const uint16_t RECV_PIN = 18;
IRrecv irrecv(RECV_PIN);
decode_results results;

// Remote codes
#define CODE_UP    0xFF18E7
#define CODE_DOWN  0xFF4AB5
#define CODE_LEFT  0xFF10EF
#define CODE_RIGHT 0xFF5AA5
#define CODE_STAR  0xFF6897
#define CODE_HASH  0xFFB04F

uint32_t lastCode = 0;         // store the last received code
unsigned long lastTime = 0;    // store the last time a command was processed
const unsigned long DEBOUNCE_DELAY = 200; // debounce time in milliseconds

void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn();
  Serial.println("IR receiver started...");
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (irrecv.decode(&results)) {

    uint32_t value = results.value;

    // Handle repeat code
    if (value == 0xFFFFFFFF) {
      value = lastCode;  // repeat last command
    } else {
      lastCode = value;  // store new command
    }

    unsigned long currentTime = millis();

    // Check debounce
    if (currentTime - lastTime > DEBOUNCE_DELAY) {
      lastTime = currentTime;

      Serial.print("Received code: 0x");
      Serial.println(value, HEX);

      switch (value) {
        case CODE_UP:
          Serial.println("Volume Up");
          break;
        case CODE_DOWN:
          Serial.println("Volume Down");
          break;
        case CODE_LEFT:
          Serial.println("Previous Song");
          break;
        case CODE_RIGHT:
          Serial.println("Next Song");
          break;
        case CODE_HASH:
          Serial.println("Manual Mode Activated (#)");
          break;
        case CODE_STAR:
          Serial.println("Auto Mode Activated (*)");
          break;
        default:
          Serial.println("Unknown Button");
          break;
      }
    }

    irrecv.resume(); // Receive the next value
  }
}
