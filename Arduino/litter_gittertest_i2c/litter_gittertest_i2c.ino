#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int x;
int y;
boolean isUpdated= false;
int w = 1024/4;
int h = 768/4;
int leftTrig = w*0.2;
int rightTrig = w*0.8;

// Motor driver vars
int pinI1=8;//define I1 interface
int pinI2=11;//define I2 interface 
int speedpinA=9;//enable motor A
int pinI3=12;//define I3 interface 
int pinI4=13;//define I4 interface 
int speedpinB=10;//enable motor B
int spead =250;//define the spead of motor

void setup()
{
  Serial.begin(9600);

  // Initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // Define callbacks for i2c communication
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  // Initialize motor driver pins
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);

  Serial.println("Ready!");
}

void loop()
{
  stopm();
}

void receiveEvent(int bytes) {
  int sentPoint = Wire.read();

  // Get object's points
  if (sentPoint == 0x01 && bytes > 1) {
    int x = Wire.read();
    Serial.println("Object Found:");
    Serial.print(x);
  }
  else if (sentPoint == 0x02 && bytes > 1) {
    int y = Wire.read();
    isUpdated = true;
    Serial.print(", ");
    Serial.println(y);
  }

  // Clear all remaining bytes
  while (Wire.available()) {
    Wire.read();
  }
  // If both points are updated, get the object
  if (isUpdated){
    isUpdated = false;
    getObject();
  }
}

void requestEvent() {
  //Wire.write((uint8_t *)&50, sizeof(50));
  Serial.println("request");
}

void getObject(){
  //stop motors
  stopm();
  delay(800);
  if (x < leftTrig || x == leftTrig) {
    //turn left
    Serial.println("left");
    left();
    delay(200);
    stopm();
  } 
  else if (x > rightTrig || x == rightTrig) {
    //turn right
    Serial.println("right");
    right();
    delay(200);
    stopm();
  } 
  else if (x > leftTrig && x < rightTrig) {
    //drive forward
    Serial.println("middle");
    //see if close to capture
    forward();
    delay(200);
    stopm();
  } 
  else {
    Serial.println("No Value");
  }
}

//-----MOTOR CONTROL-----//
void forward()
{
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
  digitalWrite(pinI1,HIGH);
}
void backward()
{
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);
}
void left()
{
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);
}
void right()
{
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
  digitalWrite(pinI1,HIGH);
}
void stopm()
{
  digitalWrite(speedpinA,LOW);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
  digitalWrite(speedpinB,LOW);

}

