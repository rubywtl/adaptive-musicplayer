#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SS_PIN   10      // FSPICS0
#define RST_PIN   3      // RST pin at top left
MFRC522 rfid(SS_PIN, RST_PIN);

#define LCD_SDA  5
#define LCD_SCL  6
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Timer Config
unsigned long expirationTime = 0;
bool timerActive = false;

// GOOD Card UID
byte GOOD_UID[] = {0x1A, 0x86, 0x0A, 0x01};
const byte GOOD_UID_SIZE = 4;

bool uidMatch(const byte* uid, byte uidSize){
  // if(uidSize != GOOD_UID_SIZE) return false;

  // for(byte i = 0; i < GOOD_UID_SIZE; i++){
  //   if(uid[i] != GOOD_UID[i]) return false;
  // }

  // assume true for now
  return true;
}

void setup() {
  Serial.begin(115200);

  // Initialize FSPI (GPIO pins for ESP32-S3)
  SPI.begin(36, 37, 35, SS_PIN);  
  //        SCK  MISO MOSI

  Serial.println("Initializing MFRC522...");
  rfid.PCD_Init();
  delay(50);

  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Currently");
  lcd.setCursor(0, 1);
  lcd.print("Playing...");
}

void loop() {
  if(timerActive){
      unsigned long remaining = (expirationTime - millis());

      if((long)remaining > 0){
          lcd.setCursor(0, 1);
          lcd.print("Time: ");
          lcd.print(remaining / 1000);  // convert ms → seconds
          lcd.print("s   ");            // pad to clear old characters
      }
      else {
          lcd.setCursor(0, 1);
          lcd.print("Time: 0s   ");
          timerActive = false;          // stop timer

          // go back to default state
          lcd.setCursor(0, 0);
          lcd.print("Currently      ");
          lcd.setCursor(0, 1);
          lcd.print("Playing...     ");
      }
  } 

  // // Look for new cards
  // if (!rfid.PICC_IsNewCardPresent()) return;
  // Serial.println("running");

  // // Read the card
  // if (!rfid.PICC_ReadCardSerial()) {
  //   Serial.println("Card detected but read failed");
  //   delay(200);
  //   return;
  // }

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    // Print UID
    // Serial.print("Card UID: ");
    // for (byte i = 0; i < rfid.uid.size; i++) {
    //   Serial.print(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
    //   Serial.print(rfid.uid.uidByte[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();

    // check if UID matches
    if(uidMatch(rfid.uid.uidByte, rfid.uid.size)){
      lcd.setCursor(0, 0);
      lcd.print("Authenticated!   ");

      // start a timer
      expirationTime = millis() + 30000; 
      timerActive = true;
    } else {
      Serial.println("Byeeeeee");
      lcd.setCursor(0, 0);
      lcd.print("Not Authenticated");
    }
    // Halt PICC (good practice)
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }
  delay(50); 
}
