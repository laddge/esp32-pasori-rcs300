#include <Arduino.h>
#include "RCS300.h"

#define PIN_STATUSLED 1

RCS300 rcs300;

void setup() {
  Serial.begin(115200);
  rcs300.begin();
}

void loop() {
  if (rcs300.connected) {
    neopixelWrite(PIN_STATUSLED, 0, 10, 0);
    // Suica (systemCode = 0x0003)
    if (rcs300.polling(0x0003) == ESP_OK) {
      neopixelWrite(PIN_STATUSLED, 0, 0, 10);

      Serial.print("idm: ");
      char idm_str[17];
      for(int i = 0; i < 8; i++) {
        snprintf(idm_str + i * 2, 3, "%02X", rcs300.idm[i]);
      }
      Serial.println(idm_str);

      Serial.print("pmm: ");
      char pmm_str[17];
      for(int i = 0; i < 8; i++) {
        snprintf(pmm_str + i * 2, 3, "%02X", rcs300.pmm[i]);
      }
      Serial.println(pmm_str);
    }
  } else {
    neopixelWrite(PIN_STATUSLED, 10, 0, 0);
  }
  delay(100);
}
