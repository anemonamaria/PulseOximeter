#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#include <util/twi.h>

#define REPORTING_PERIOD_MS 1000
PulseOximeter pox;
uint32_t tsLastReport = 0;
int buzzer = 11;

void onBeatDetected() {
  Serial.println("Beat!");
  tone(buzzer, 450);
}

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
// put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Initializing pulse oximeter..");
  
  lcd.begin();
  lcd.backlight();
  lcd.print("Hello,world!");

  
  if(!pox.begin()) {
    Serial.println("Failed!");
    for(;;);
  } else {
    Serial.println("Success");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
  // put your main code here, to run repeatedly:
  pox.update();
  if(millis() - tsLastReport > REPORTING_PERIOD_MS) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Heart rate:");
    Serial.print("Heart rate:");
    lcd.print(pox.getHeartRate());
    if(pox.getHeartRate() != 0) {
      tone(buzzer,900, 20);
      noTone(buzzer);
    }
    Serial.print(pox.getHeartRate());
    lcd.print("bpm /SpO2:");
    Serial.print("bpm /SpO2:");
    lcd.print(pox.getSpO2());
    lcd.print("%");
    Serial.print(pox.getSpO2());
    Serial.print("%");
    tsLastReport = millis();
  }
}
