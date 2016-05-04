#include <Servo.h> 
 
Servo myservo;

 
void setup() 
{ 
  myservo.attach(11);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
  myservo.write(40);
  delay(1000);
  myservo.write(140);
  delay(1000);
} 

