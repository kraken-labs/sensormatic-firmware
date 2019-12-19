#include "i2c.c"
#include "onewire.c"
#include "onewire.h"
#define WIRELESS 1

#ifdef WIRELESS
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#define CE_PIN PIN_B1
#define CSN_PIN PIN_A3
#endif

#define DS18B20_PIN 2 // PIN_B2

#include "PinChangeInterrupt.h"

#define PIN_PULSADOR_A PIN_A2
#define PIN_PULSADOR_B PIN_A1
#define PIN_PULSADOR_C PIN_A0

#define PRESS_STATE_IDLE 0
#define PRESS_STATE_STARTED 1

#define DETECT_SHORT_MS 5
#define DETECT_LONG_MS 1000
//#define DETECT_DOUBLE_MS 400


//volatile byte pressStateA = PRESS_STATE_IDLE;
volatile byte pressStateB = PRESS_STATE_IDLE;
volatile byte pressStateC = PRESS_STATE_IDLE;

//volatile unsigned long initPressMillisA = 0;
volatile unsigned long initPressMillisB = 0;
volatile unsigned long initPressMillisC = 0;

//volatile unsigned long prevPressMillisA = 0;
volatile unsigned long prevPressMillisB = 0;
volatile unsigned long prevPressMillisC = 0;

//unsigned long diffMillisA = 0;
unsigned long diffMillisB = 0;
unsigned long diffMillisC = 0;

//////////////

/*void setPulsadorA(void) {
  if (pressStateA == PRESS_STATE_IDLE) {
    byte pinState;
    pinState = (PINA >> PIN_PULSADOR_A)& 1;
    if (pinState == 1) {
      pressStateA = PRESS_STATE_STARTED;
      initPressMillisA = millis();
    }
  }
}*/

void setPulsadorB(void) {
  if (pressStateB == PRESS_STATE_IDLE) {
    byte pinState;
    pinState = (PINA >> PIN_PULSADOR_B)& 1;
    if (pinState == 1) {
      pressStateB = PRESS_STATE_STARTED;
      initPressMillisB = millis();
    }
  }
}

void setPulsadorC(void) {
  if (pressStateC == PRESS_STATE_IDLE) {
    //byte pinState;
    //pinState = (PINA >> PIN_PULSADOR_C)& 1;
    if (((PINA >> PIN_PULSADOR_C)& 1) == 1) {
      pressStateC = PRESS_STATE_STARTED;
      //initPressMillisC = millis();
    }
  }
}

#include <EEPROM.h>
int addr = 0;

volatile byte counter = 0;

uint32_t publishTimer = 0;

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};

#ifdef WIRELESS
RF24 radio(CE_PIN, CSN_PIN);
RF24Network network(radio);
RF24Mesh mesh(radio, network);
#endif

void storeCounter() {
  EEPROM.update(addr, counter);  
}
// uint32_t
void publishMsg(uint16_t data) {
  //debugOut(60);
  //debugOut(61);
  #ifdef WIRELESS
  if (!mesh.write(&data, 'M', sizeof(data))) {
    // If a write fails, check connectivity to the mesh network
    //debugOut(70);
    if ( !mesh.checkConnection() ) {
      //refresh the network address
      //Serial.println("Renewing Address");
      //debugOut(71);
      if (!mesh.renewAddress(1000)) {
        //debugOut(72);
        //If address renewal fails, reconfigure the radio and restart the mesh
        //This allows recovery from most if not all radio errors
        mesh.begin(40, RF24_250KBPS, 1000);
      }
    } else {
      //debugOut(80);
      //Serial.println("Send fail, Test OK");
    }
  } else {
    //debugOut(69);
    //Serial.print("Send OK: ");
  }
  #endif
}

void debugOut(byte value) {
  showDigits(value);
  _delay_ms(500);
}

/////////////////////////////

uint32_t currentTemperature = 0;

void setup() {
  #ifdef WIRELESS
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  #endif
  pinMode(PIN_PULSADOR_A, INPUT);
  pinMode(PIN_PULSADOR_B, INPUT);
  pinMode(PIN_PULSADOR_C, INPUT);
  pinMode(DS18B20_PIN, INPUT);

  init_i2c(); // https://github.com/szczys/avr-i2c/tree/master/bitbang
  onewire_init(DS18B20_PIN);

  showDigits(88);

  //attachPCINT(digitalPinToPCINT(PIN_PULSADOR_A), setPulsadorA, RISING);
  attachPCINT(digitalPinToPCINT(PIN_PULSADOR_B), setPulsadorB, RISING);
  attachPCINT(digitalPinToPCINT(PIN_PULSADOR_C), setPulsadorC, RISING);

  showDigits(0);

  #ifdef WIRELESS
  radio.setPALevel(RF24_PA_MAX);
  mesh.begin(40, RF24_250KBPS, 1000);
  showDigits(1);
  #endif

  counter = EEPROM.read(addr);
  showDigits(2);
  
  /*GIMSK |= _BV(PCIE0);   // Enable Pin Change Interrupts
  //PCMSK0 |= _BV(PCINT0); // Use PA0 as interrupt pin

  PCMSK0 |= _BV(PCINT0); // PA0
  PCMSK0 |= _BV(PCINT1); // PA1
  PCMSK0 |= _BV(PCINT3); // PA3

  // MCUCR &= ~(_BV(ISC01) | _BV(ISC00));
  MCUCR |= (_BV(ISC01) | _BV(ISC00));
  
  sei(); //Enable interrupts*/
}

void loop() {
  #ifdef WIRELESS
  mesh.update();
  #endif
  
  //if (pressStateA == PRESS_STATE_STARTED) {
  //  diffMillisA = millis() - initPressMillisA;
    
  //  if (digitalRead(PIN_PULSADOR_A) == LOW) {
      /*if ((initPressMillisA - prevPressMillisA) < DETECT_DOUBLE_MS) {
        prevPressMillisA = 0;
        initPressMillisA = 0;
        pressStateA = PRESS_STATE_IDLE;

        // increaseCounter();
        // blinkT(1, 150);
        counter = 24;
      } else {*/
  //      if (diffMillisA >= DETECT_SHORT_MS && diffMillisA < DETECT_LONG_MS) {
  //        prevPressMillisA = initPressMillisA;

  //        initPressMillisA = 0;
  //        pressStateA = PRESS_STATE_IDLE;

  //        counter = 24;
  //        showDigits(counter);
          // blinkFromCounter();
  //      }
      // }
  //  } else {
      /*if (diffMillis > DETECT_LONG_MS) {
        pressState = PRESS_STATE_IDLE;
        initPressMillis = 0;
        prevPressMillis = 0;

        resetCounter();
        blinkT(1, 500);
      }
      */
  // }
  //}

  if (pressStateB == PRESS_STATE_STARTED) {
    diffMillisB = millis() - initPressMillisB;
    
    if (digitalRead(PIN_PULSADOR_B) == LOW) {
      if (diffMillisB >= DETECT_SHORT_MS && diffMillisB < DETECT_LONG_MS) {
        prevPressMillisB = initPressMillisB;

        initPressMillisB = 0;
        pressStateB = PRESS_STATE_IDLE;

        counter--;
        showDigits(counter);
        storeCounter();
        publishMsg((uint32_t) counter);
        _delay_ms(500);
      }
    } else {
      //if (diffMillisC > DETECT_LONG_MS) {
        //pressState = PRESS_STATE_IDLE;
        //initPressMillis = 0;
        //prevPressMillis = 0;

        //resetCounter();
        //blinkT(1, 500);
      //}
    }
  }

  if (pressStateC == PRESS_STATE_STARTED) {
    //diffMillisC = millis() - initPressMillisC;
    
    if (digitalRead(PIN_PULSADOR_C) == LOW) {
      //if (diffMillisC >= DETECT_SHORT_MS && diffMillisC < DETECT_LONG_MS) {
        //prevPressMillisC = initPressMillisC;

        //initPressMillisC = 0;
        pressStateC = PRESS_STATE_IDLE;

        counter++;
        showDigits(counter);
        storeCounter();
        publishMsg((uint32_t) counter);
        _delay_ms(500);
      //}
    } else {
      /*if (diffMillis > DETECT_LONG_MS) {
        pressState = PRESS_STATE_IDLE;
        initPressMillis = 0;
        prevPressMillis = 0;

        resetCounter();
        blinkT(1, 500);
      }*/
    }
  }

  // _delay_ms(500);

  if (millis() - publishTimer >= 5000) {
    publishTimer = millis();

  uint16_t t;
	onewire_reset(); // 1-Wire reset
	onewire_write(ONEWIRE_SKIP_ROM); // to all devices on the bus
	onewire_write(0x44); // send DS18B20 command, "CONVERT T"

	onewire_reset(); // 1-Wire reset
	onewire_write(ONEWIRE_SKIP_ROM); // to all devices on the bus
	onewire_write(0xBE); // send DS18B20 command, "READ SCRATCHPAD"

	t = onewire_read(); // read temperature low byte
	t |= (uint16_t)onewire_read() << 8; // and high byte
	t = ((t >> 4) * 100 + ((t << 12) / 6553) * 10) / 100; // decode temp
  
  showDigits((byte) t);
  
  publishMsg((uint16_t) t);
  }
    // currentTemperature = (unsigned long) read_temperature();
    //showDigits((byte) currentTemperature);
    //publishMsg(currentTemperature);
  //}

  #ifdef WIRELESS
  while (network.available()) {
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header, &payload, sizeof(payload));
    //Serial.print("Received packet #");
    //Serial.print(payload.counter);
    //Serial.print(" at ");
    //Serial.println(payload.ms);
  }
  #endif
}
