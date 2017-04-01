#include "cpu.hpp"

int v = HIGH;

void setup() {
  pinMode(13, OUTPUT);
  CPU::power();
}

void loop() {
  digitalWrite(13, v++ );
  CPU::run_frame();
  v = v % 2;
}
