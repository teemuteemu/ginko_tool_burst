/*
	"Chain-Saw" (A Tool that is also a sequence)

  Records an incomming value, then plays it back, quantised to steps.

  TR/G input;
    ...
  Input 1 & Knob 1;
    ...
  Input 2 & knob 2;
    ...
  Button A;
    ...
  Led A;
    ...

 */
#include <Arduino.h>
#include <avr/interrupt.h> //library for using interupts
#include <avr/pgmspace.h>

#include "SPI.h"

#define MOD_A_PIN A1
#define MOD_B_PIN A2
#define TRIGGER_PIN 4
#define BUT_A_PIN 2
#define BUT_B_PIN 9
#define LED_A_PIN 3
#define LED_B_PIN 5
#define SPI_SS_PIN 10

#define TIMER_MIN 20 // 500hz / 20 => 25hz = 0.04s
#define TIMER_MAX 500 // 500hz / 500 => 1hz = 1s
#define MOD_B_MIN 1
#define MOD_B_MAX 10

enum State { OFF, ON };

State state;
boolean prevTrigger;
boolean currentTrigger;
boolean prevButtonA;
boolean currentButtonA;
volatile uint16_t burstCounter;
volatile uint16_t burstVal;
uint16_t modAVal;
uint16_t modBVal;

void output(uint16_t value) {
  byte data;                              //spi deals in bytes (8 bits) chunks

  // digitalWrite(SPI_SS_PIN, LOW);       //enable writing
  PORTB &= 0b11111011;                    //same except faster

  data = highByte(value);                 //this includes the 4 most signifficant bits
  data = 0b00001111 & data;               //the rest of the byte is settings for the dac: mask those
  data = 0b00110000 | data;               //set those setting bits (see DAC data sheet or trust me)

  SPI.transfer(data);                     //actually send this out
  data = lowByte(value);                  //the 8 least signifficant bits of our value
  SPI.transfer(data);                     //send them

  // digitalWrite(SPI_SS_PIN, HIGH);      //close connection, allowing the DAC to update
  PORTB |= 0b00000100;                    //same except faster
}

void outputTrigger() {
  output(65535);
  digitalWrite(LED_A_PIN, HIGH);

  delayMicroseconds(5000);

  output(0);
  digitalWrite(LED_A_PIN, LOW);
}

void setup() {
  Serial.begin(9600);
  Serial.println("setup");

  state = OFF;
  prevTrigger = false;
  currentTrigger = false;
  prevButtonA = false;
  currentButtonA = false;
  burstCounter = 0;
  burstVal = TIMER_MAX;

	cli();							// disable global interrupts
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 124;          // 500 Hz (16000000/((124+1)*256))
  TCCR1B |= (1 << WGM12); // CTC
  TCCR1B |= (1 << CS12); // Prescaler 256
  TIMSK1 |= B00000010;  // Enable Timer COMPA Interrupt
	sei();							//re-enable global interupts

  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(BUT_A_PIN, INPUT_PULLUP);
  pinMode(BUT_B_PIN, INPUT_PULLUP);
  pinMode(SPI_SS_PIN, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
}

void loop() {
  modAVal = map(analogRead(MOD_A_PIN), 0, 1023, TIMER_MIN, TIMER_MAX);
  modBVal = map(analogRead(MOD_B_PIN), 0, 1023, MOD_B_MIN, MOD_B_MAX);
  currentTrigger = !digitalRead(TRIGGER_PIN);
  currentButtonA = !digitalRead(BUT_A_PIN);

  if ((prevTrigger == 0 && currentTrigger == 1) || (prevButtonA == 1 && currentButtonA == 1)) {
    state = ON;
    burstCounter = 0;
    burstVal = modAVal;
    outputTrigger();
  }
  
  prevTrigger = currentTrigger;
  prevButtonA = currentButtonA;
}

ISR(TIMER1_COMPA_vect) {
  if (++burstCounter >= burstVal) {
    burstCounter = 0;
    outputTrigger();
  }
}
