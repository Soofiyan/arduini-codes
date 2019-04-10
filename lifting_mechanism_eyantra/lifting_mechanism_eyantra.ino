#include <stdbool.h>
bool ir_value;
bool counter1,counter2=0,check = 1;
void drivewheel_1(long sp_vect, long l_lim, long h_lim);
void drivewheel_2(long sp_vect, long l_lim, long h_lim);
void setup() {
  // put your setup code here, to run once:
  pinMode(5,OUTPUT);//pwm
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(7,INPUT);//ir
}

void loop() {
  // put your main code here, to run repeatedly:
  ir_value = digitalRead(7);
  counter1 = (ir_value == 1)? 0:1;
  if(counter1)
  {
    if(!counter2)
    {
      drivewheel_1(100);
      counter2 = 0;
    }
    else if(counter2)
    {
      drivewheel_1(-100);
      counter2 = 1;
    }
    check = 1;
  }
  else
  {
    if(check)
    {
      counter2 ^= 1;
      check = 0;
    }
    drivewheel_1(0);
  }
}
void drivewheel_1(long sp_vect)
{  
  if (sp_vect<-15)
  {
    digitalWrite(8,HIGH);
    digitalWrite(9,LOW);
    sp_vect=(-sp_vect);
  }
  else if (sp_vect>15)
  {
    digitalWrite(9,HIGH);
    digitalWrite(8,LOW);
  }
  else
  {
    sp_vect=0;
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
  }
  analogWrite(5,sp_vect);
}
