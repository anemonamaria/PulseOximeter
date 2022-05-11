#include <LiquidCrystal_I2C.h>

//#include <LiquidCrystalIO.h>

//#include <BigCrystal.h>
//#include <BigFont.h>

#include <LiquidCrystal.h>

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
////


#include <util/twi.h>

/* ==== High-level AVR I2C routines ==== */
// constants
#define I2C_TIMEOUT 20  /* increments of 50 microseconds => 1ms */
#define I2C_READ    1 // read mode (LSBit must be 1)
#define I2C_WRITE   0 // write transaction (LSBit must be 0)
// I2C error statuses
#define I2C_ERR_TIMEOUT -1
#define I2C_ERR_START   -2
#define I2C_ERR_ACK     -3



#define REPORTING_PERIOD_MS 1000
////
PulseOximeter pox;
uint32_t tsLastReport = 0;
////
void onBeatDetected() {
Serial.println("Beat!");
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
////  // put your main code here, to run repeatedly:
  pox.update();
if(millis() - tsLastReport > REPORTING_PERIOD_MS) {
 Serial.print("Heart rate:");
 Serial.print(pox.getHeartRate());
 Serial.print("bpm /SpO2:");
Serial.print(pox.getSpO2());
Serial.print("%");
////
 tsLastReport = millis();
 }
}





/** Waits for an I2C operation to complete */
uint8_t I2C_waitForInterrupt() {
  uint8_t i = 0;
  while (!(TWCR & (1<<TWINT))) {
    if (i >= I2C_TIMEOUT) return 0;
    _delay_us(50); i++;
  }
  return 1;
}

/** Starts an I2C transaction */
int8_t I2C_start(uint8_t address, uint8_t mode) {
  TWCR = 0;
  // transmit START condition (clear interrupt flag + send start bit + enable TWI module)
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  // wait for completion
  if (!I2C_waitForInterrupt()) /* wait for compeltion */
    return I2C_ERR_TIMEOUT;
  // check if the start condition was successfully transmitted
  uint8_t twst = (TWSR & TW_STATUS_MASK);
  if (twst != TW_START) {
    return I2C_ERR_START;
  }
  // load slave address into data register
  TWDR = (((address) << 1) | (mode));
  // start transmission of address
  TWCR = (1<<TWINT) | (1<<TWEN);
  if (!I2C_waitForInterrupt()) /* wait for compeltion */
    return I2C_ERR_TIMEOUT;
  // check if the device has acknowledged the READ / WRITE mode
  twst = (TWSR & TW_STATUS_MASK);
  if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK))
    return I2C_ERR_ACK;
  return 0;
}

/** Stops an I2C transaction */
void I2C_stop() {
  // clear interrupt flag + send Stop bit + enable TWI module
  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) ;
}

/**
 * Sends 1 byte over the I2C line.
 * (Note: I2C_WRITE transaction must be started first!)
 */
int8_t I2C_write(uint8_t data) {
  // load data into register
  TWDR = data;
  // configure write operation (simply clear interrupt flag + enable TWI)
  TWCR = (1<<TWINT) | (1<<TWEN);
  if (!I2C_waitForInterrupt()) /* wait for compeltion */
     return I2C_ERR_TIMEOUT;
  if ((TWSR & TW_STATUS_MASK) != TW_MT_DATA_ACK)
     return I2C_ERR_ACK;
  return 0;
}

/**
 * Reads 1 byte from the I2C line (and ACK or NACK at the end).
 * (Note: I2C_READ transaction must be started first!)
 */
int16_t I2C_read(uint8_t ack_more) {
  // configure read operation; if ACK was requested, also set "EnableAck" flag
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack_more ? (1 << TWEA) : 0);
  if (!I2C_waitForInterrupt()) /* wait for compeltion */
    return I2C_ERR_TIMEOUT;
  return TWDR;
}

//void setup()
//{
//  // initialize UART for console printing
//  Serial.begin(9600);
//  // Initialize I2C to a 100KHz clock
//  // TWI Status Register: initialize prescaler to 1
//  TWSR = (0b00 << TWPS0);
//  // TWI Bitrate Register: set bitrate
//  // SCL_Freq = CPU_Freq / (16 + 2*TWBR * TWSR_Prescaler)
//  // so: TWBR = (SCL_Freq / CPU_Freq - 16) / (TWSR_Prescaler * 2)
//  TWBR = 72; // (16000000/100000 - 16) / (1 * 2)
//}

int I2C_Scan()
{
  uint8_t foundAddress = 0;
  int res;
  for (uint8_t addr=1; addr <= 0x7F; addr++) {
    res = I2C_start(addr, I2C_WRITE); // start a dummy write transaction
    I2C_stop();
    // TODO: I2C_Scan: check result
    if (res == 0) {
      Serial.print("Found I2C dev at 0x");
      Serial.println(addr, HEX);
//      firstAddress = addr;
    }
  }
  return foundAddress;
}

// TODO: implement high-level register write routine
int I2C_RegisterWrite(uint8_t addr, uint8_t reg_id, uint8_t *data, uint8_t data_len)
{
  int res;
  // TODO: 1. start a transaction
  if (I2C_start(addr, I2C_WRITE) < 0) return -1;
  // TODO: 2. write the address of the register
  I2C_write(reg_id);
  for (int i=0; i<data_len; i++) {
    // TODO: 3. write the register's contents (`data_len` bytes)
    res = I2C_write(data[i]);
    if (res < 0) { I2C_stop(); return res; }
  }
  // finally, stop the transaction
  I2C_stop();
  return 0;
}

// TODO: implement high-level register read routine
int I2C_RegisterRead(uint8_t addr, uint8_t reg_id, uint8_t *data_out, uint8_t data_len)
{
  int res;
  // TODO: 1. start a transaction
  if (I2C_start(addr, I2C_WRITE) < 0) return -1;
  // TODO: 2. write the address of the register
  I2C_write(reg_id);
  // TODO: 3. re-start with a I2C_READ transaction
  if (I2C_start(addr, I2C_READ) < 0) return -1;
  // TODO: 4. read the register's contents (`data_len` bytes)
  for (int i=0; i<data_len; i++) {
    res = I2C_read((i < (data_len - 1))); // do not ACK for the last byte
    // append the result into our data array
    if (res >= 0) data_out[i] = res;
  }
  // finally, stop the transaction
  I2C_stop();
  return 0;
}

//void loop()
//{
//  Serial.println("Scanning...");
//  int address = I2C_Scan();
//  if (!address) {
//    Serial.println("Nothing found :(");
//    delay(5000); return;
//  } else  {
//    Serial.println("address found");
//  }

//}
