//////////////
// based on https://github.com/szczys/avr-i2c/blob/master/bitbang/i2c-bitbang.c
#include <avr/io.h>
#include <Arduino.h>
#include <util/delay.h>

#define I2C_PORT  PORTB
#define I2C_DDR   DDRB
#define SCK   1<<PB0
#define SDA   1<<PA7

#define I2C_SDA_LO    DDRA |= SDA
#define I2C_SCK_LO    I2C_DDR |= SCK
#define I2C_SDA_HI    DDRA &= ~(SDA)
#define I2C_SCK_HI    I2C_DDR &= ~(SCK)
#define I2C_WAIT      _delay_us(8)

void i2c_start(void) {
  I2C_SCK_HI;
  I2C_SDA_HI;
  I2C_SDA_LO;
  I2C_WAIT;
  I2C_SCK_LO;
  I2C_WAIT;
}

void i2c_stop(void) {
  I2C_SDA_LO;
  I2C_WAIT;
  I2C_SCK_HI;
  I2C_WAIT;
  I2C_SDA_HI;
  I2C_WAIT;
}

void i2c_writebit(uint8_t val) {
  if (val) { I2C_SDA_HI; }
  else { I2C_SDA_LO; }
  I2C_WAIT;
  I2C_SCK_HI;
  I2C_WAIT;
  I2C_SCK_LO;
}

void i2c_writebyte(uint8_t byte) {
  for (uint8_t i=0; i<8; i++) {
    i2c_writebit(byte & 0x80);
    byte <<= 1;
  }

  //Make sure you read the ACK
  I2C_SDA_HI;
  I2C_WAIT;
  I2C_SCK_HI;
  I2C_WAIT;
  //Read should be done on this line. Not using it yet so not doing read.
  I2C_SCK_LO;
  I2C_WAIT;
}

void showDigits(byte value) {
  i2c_start();
  i2c_writebyte(0xA); // doble de 0x5, por que?
  i2c_writebyte(value);
  i2c_stop();
}

void init_i2c(void) {
  I2C_DDR &= ~(SCK | SDA);
}
