#include <Wire.h>

#define SLAVE_ADDRESS 0x04

void setup() {
  Serial.begin(9600); // start serial for output
  
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("Ready!");
}

void loop() {
  delay(100);
}

void receiveEvent(int bytes) {
  int sentPoint = Wire.read();
  
  // get object's points
  if (sentPoint == 0x01 && bytes > 1) {
    int x = Wire.read();
    Serial.println("Object Found:");
    Serial.print(x);
  }else if (sentPoint == 0x02 && bytes > 1) {
    int y = Wire.read();
    Serial.print(", ");
    Serial.println(y);
  }

  // Clear all remaining bytes
  while (Wire.available()) {
    Wire.read();
  }
}

void requestEvent() {
  //Wire.write((uint8_t *)&50, sizeof(50));
  Serial.println("request");
}

