#include "cpu.hpp"

#include <stdio.h>
// 2400 for the authentic experience, 9600 for something more enjoyable
#define BAUD 9600
#include <util/setbaud.h>

void uart_init(void) {
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

#if USE_2X
  UCSR0A |= _BV(U2X0);
#else
  UCSR0A &= ~(_BV(U2X0));
#endif

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void setup() {
  //Initialize serial and wait for port to open:
  uart_init();
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  CPU::power();
}

void loop() {
  CPU::run();
}
