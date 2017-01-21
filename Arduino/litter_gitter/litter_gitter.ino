int x;
int y;
int w = 480;
int h = 320;
int leftTrig = w*0.4;
int rightTrig = w*0.6;
int leftLed = 10;
int rightLed = 11;
int speedPin_M1 = 5;     //M1 Speed Control
int speedPin_M2 = 6;     //M2 Speed Control
int directionPin_M1 = 4;     //M1 Direction Control
int directionPin_M2 = 7;     //M1 Direction Control

void setup(){
  Serial.begin(9600);
  pinMode(speedPin_M1, OUTPUT);
  pinMode(speedPjn_M2, OUTPUT);
  pinMode(directionPin_M1, OUTPUT);
  pinMode(directionPin_M2, OUTPUT);
}

void loop(){
  //stop motors
  //drive and avoid obstacles
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
  carStop();
  delay(200);
  if (x < leftTrig || x == leftTrig) {
    //turn left
    Serial.println("left");
    carTurnLeft(250,250);
  } 
  else if (x > rightTrig || x == rightTrig) {
    //turn right
    Serial.println("right");
    carTurnRight(250,250);
  } 
  else if (x > leftTrig && x < rightTrig) {
    //drive forward
    Serial.println("middle");
    //see if close to capture
    carAdvance(100,100);
  } 
  else {
    Serial.println("No Value");
  }
}

void carStop(){                 //  Motor Stop
  digitalWrite(speedPin_M2,0); 
  digitalWrite(directionPin_M1,LOW);    
  digitalWrite(speedPin_M1,0);   
  digitalWrite(directionPin_M2,LOW);    
}   

void carBack(int leftSpeed,int rightSpeed){         //Move backward
  analogWrite (speedPin_M2,leftSpeed);              //PWM Speed Control
  digitalWrite(directionPin_M1,HIGH);    
  analogWrite (speedPin_M1,rightSpeed);    
  digitalWrite(directionPin_M2,HIGH);
} 

void carAdvance(int leftSpeed,int rightSpeed){       //Move forward
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,LOW);   
  analogWrite (speedPin_M1,rightSpeed);    
  digitalWrite(directionPin_M2,LOW);
}

void carTurnLeft(int leftSpeed,int rightSpeed){      //Turn Left
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,LOW);    
  analogWrite (speedPin_M1,rightSpeed);    
  digitalWrite(directionPin_M2,HIGH);
}
void carTurnRight(int leftSpeed,int rightSpeed){      //Turn Right
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,HIGH);    
  analogWrite (speedPin_M1,rightSpeed);    
  digitalWrite(directionPin_M2,LOW);
}

