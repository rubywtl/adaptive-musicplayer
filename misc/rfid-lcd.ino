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



// GOOD Card UID
byte GOOD_UID[] = {0x1A, 0x86, 0x0A, 0x01};
const byte GOOD_UID_SIZE = 4;

bool uidMatch(const byte* uid, byte uidSize){
  if(uidSize != GOOD_UID_SIZE) return false;

  for(byte i = 0; i < GOOD_UID_SIZE; i++){
    if(uid[i] != GOOD_UID[i]) return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);

  // Initialize FSPI (GPIO pins for ESP32-S3)
  SPI.begin(36, 37, 35, SS_PIN);  
  //          SCK  MISO MOSI

  rfid.PCD_Init();
  delay(50);

  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("RFID Ready");
}

void loop() {
  // Look for new cards
  if (!rfid.PICC_IsNewCardPresent()) return;

  // Read the card
  if (!rfid.PICC_ReadCardSerial()) return;

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
    Serial.println("Hello World");
    lcd.setCursor(0, 0);
    lcd.print("Hello World!");
  } else {
    Serial.println("Byeeeeee");
    lcd.setCursor(0, 0);
    lcd.print("Not Authenticated");
  }

  delay(1500); // keep message on screen
  lcd.clear();

  // Halt PICC (good practice)
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

}
