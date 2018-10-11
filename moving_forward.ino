//control servo motor with serial monitering window - by ujash patel
  //for more go to www.uu-machinetool.blogspot.com

#include <Servo.h> 

Servo servo1;
Servo servo2;
Servo servo3;
 int x;
 int y;
 int z;
 int prev_angle1=0;
 int prev_angle2=90;
 int prev_angle3=90;
 int pos;
 int pos2;
 int pos3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo1.attach(6);
  servo2.attach(9);
  servo3.attach(11);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available() == 0){
    }
  
  if (Serial.read() != -1){
    x = Serial.readStringUntil(',').toInt();
    y = Serial.readStringUntil(',').toInt();
    z = Serial.readStringUntil(',').toInt();
    Serial.println(x);
    Serial.println(y);
    Serial.println(z);
  }

  x = (x*150)/180;
  z = z-10;
  
  if(x>= prev_angle1){
    for(pos=prev_angle1; pos<=x; pos++){
      servo1.write(pos);
      Serial.println(1);
      Serial.println(pos);
      delay(15);
  }
  }

  if(x<prev_angle1){
    for(pos=prev_angle1; pos>=x; pos--){
      servo1.write(pos);
      Serial.println(1);
      Serial.println(pos);
      delay(15);
  }
  }
  prev_angle1 = x;

  if(y>= prev_angle2){
    for(pos2=prev_angle2; pos2<=y; pos2++){
      servo2.write(pos2);
      Serial.println(2);
      Serial.println(pos2);
      delay(15);
  }
  }

  if(y < prev_angle2){
    for(pos2=prev_angle2; pos2>=y; pos2--){
      servo2.write(pos2);
      Serial.println(2);
      Serial.println(pos2);
      delay(15);
  }
  }
  prev_angle2 = y;

  if(z>= prev_angle3){
    for(pos3=prev_angle3; pos3<=z; pos3++){
      servo3.write(pos3);
      Serial.println(3);
      Serial.println(pos3);
      delay(15);
  }
  }

  if(z < prev_angle3){
    for(pos3=prev_angle3; pos3>=z; pos3--){
      servo3.write(pos3);
      Serial.println(3);
      Serial.println(pos3);
      delay(15);
  }
  }
  prev_angle3 = z;
}
