#define S0 10
#define S1 11
#define S2 13
#define S3 12
#define sensorOut 21

#include <LiquidCrystal.h>

int left,mid,right,sir,dir;
int i = 0;
int frequency = 0;
const int rs = 25, en = 29, d4 = 33, d5 = 32, d6 = 31, d7 = 30;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  // put your setup code here, to run once:
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
pinMode(A0,INPUT);
pinMode(A5,INPUT);
pinMode(A1,INPUT);
pinMode(A2,INPUT);
pinMode(A3,INPUT);
pinMode(11,OUTPUT);
pinMode(12,OUTPUT);
pinMode(13,OUTPUT);
 lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
left = analogRead(A1);
mid = analogRead(A2);
right = analogRead(A3);
sir = analogRead(A0);
dir = digitalRead(A5);
Serial.println(left);
Serial.println(mid);
Serial.println(right);
analogWrite(12,255);

//Serial.println(dir);
//Serial.println(sir);
Serial.println();
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);
i = i+ 10;
digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  frequency = map(frequency, 25,72,255,0);
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(frequency);//printing RED color frequency
  Serial.print("  ");
  delay(100);
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  frequency = map(frequency, 30,90,255,0);
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(frequency);//printing RED color frequency
  Serial.print("  ");
  delay(100);
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  frequency = map(frequency, 25,70,255,0);
  Serial.print("B= ");//printing name
  Serial.print(frequency);
  Serial.println("  ");
  delay(100);
}
