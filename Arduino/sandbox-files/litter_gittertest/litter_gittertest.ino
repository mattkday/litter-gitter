int x;
int y;
int w = 1024/4;
int h = 768/4;
int leftTrig = w*0.2;
int rightTrig = w*0.8;
int leftLed = 10;
int rightLed = 11;
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
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);
}
 
void serialEvent(){
  while (Serial.available()) {
    x = Serial.parseInt();
    y = Serial.parseInt();
    if (Serial.available() == 1) {
      getObject();
      Serial.println(x);
      Serial.println(y);
    }  
  }
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

void forward()
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
}
void backward()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}
void left()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}
void right()//
{
     analogWrite(speedpinA,spead);//input a simulation value to set the speed
     analogWrite(speedpinB,spead);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
     digitalWrite(pinI1,HIGH);
}
void stopm()//
{
     digitalWrite(speedpinA,LOW);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
     digitalWrite(speedpinB,LOW);
 
}

void loop()
{
  stopm();
}
