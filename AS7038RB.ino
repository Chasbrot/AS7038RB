extern "C" {
#include "AS7038RB.h"
}

#undef AS7038_ENABLE_PIN
#define AS7038_ENABLE_PIN 10

uint16_t samples[64];

void setup() {
  Serial.begin(115200);
  if(as7038_init()){
    Serial.print("AS7038 found!");
  }else{
    Serial.print("AS7038 NOT found!");
  }
  //as7038_led_blink();
  Serial.println("Blink finished");
  Serial.println(as7038_pox_start());
  Serial.flush();
  as7038_seq_active(true);
  //memset(samples, 0, 64 * sizeof(uint16_t));
  Serial.println("finished setup");
  int k=12;
  Serial.println(k);
}

void loop() {
  uint8_t s = as7038_read_fifo(samples);
  Serial.println(s);
  for (int i = 0; i < s; i++)
  {
    Serial.println(samples[i]);
  }
  delay(2000);
}
