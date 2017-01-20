int x;
int y;
int w = 480;
int h = 320;
int leftTrig = w*0.4;
int rightTrig = w*0.6;
int leftLed = 10;
int rightLed = 11;

void setup(){
  Serial.begin(9600);
  pinMode(leftLed, OUTPUT);
  pinMode(rightLed, OUTPUT);
}

void loop(){
  //stop motors
  //drive and avoid obstacles
}

void serialEvent(){
  int tx, ty;
  while (Serial.available()>0) {
    int tx = Serial.parseInt();
    int ty = Serial.parseInt();
    if (tx >0 && ty >0 && Serial.available() < 2) {
      x = tx;
      y = ty;
      getObject();
      Serial.println(x);
      Serial.println(y);
    }  
  }

}

void getObject(){
  //stop motors
  digitalWrite(leftLed, LOW);
  digitalWrite(rightLed, LOW);
  if (x < leftTrig || x == leftTrig) {
    //turn left
    Serial.println("left");
    digitalWrite(leftLed, HIGH);
    delay(250);
    digitalWrite(leftLed, LOW);
    delay(250);
  } 
  else if (x > rightTrig || x == rightTrig) {
    //turn right
    Serial.println("right");
    digitalWrite(rightLed, HIGH);
    delay(250);
    digitalWrite(rightLed, LOW);
    delay(250);
  } 
  else if (x > leftTrig && x < rightTrig) {
    //drive forward
    Serial.println("middle");
    //see if close to capture
    digitalWrite(leftLed, HIGH);
    digitalWrite(rightLed, HIGH);
    delay(250);
    digitalWrite(leftLed, LOW);
    digitalWrite(rightLed, LOW);
    delay(250);
  } 
  else {
    Serial.println("No Value");
  }
}

