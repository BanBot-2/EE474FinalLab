#include <Arduino.h>
#include "Freenove_IR_Lib_for_ESP32.h"

Freenove_ESP32_IR_Recv ir_recv(37);

void setup() {
  Serial.begin(115200);
  Serial.println("Freenove IR Receiver ready...");
}

void loop() {
  // Run the IR task to process incoming signals
  ir_recv.task();

  // If a NEC code is available, print it
  if (ir_recv.nec_available()) {
    Serial.printf("IR Protocol: %s, IR Code: %#x\n",
                  ir_recv.protocol(),
                  ir_recv.data());
  }
}
