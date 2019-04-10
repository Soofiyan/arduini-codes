#include "SoftwareSerial.h"
SoftwareSerial serial_connection(10, 11);//Create a serial connection with TX and RX on these pins
#define BUFFER_SIZE 64//This will prevent buffer overruns.
char inData[BUFFER_SIZE];//This is a character buffer where the data sent by the python script will go.
char inChar=-1;//Initialie the first character as nothing
int count=0,count1=0;//This is the number of lines sent in from the python script
int i=0;//Arduinos are not the most capable chips in the world so I just create the looping variable once
bool start = 0;
void setup()
{
  Serial.begin(9600);//Initialize communications to the serial monitor in the Arduino IDE
  serial_connection.begin(9600);//Initialize communications with the bluetooth module
  //serial_connection.println("Ready!!!");//Send something to just start comms. This will never be seen.
  Serial.println("Started");//Tell the serial monitor that the sketch has started.
}
void loop()
{
  //This will prevent bufferoverrun errors
  byte byte_count=serial_connection.available();//This gets the number of bytes that were sent by the python script
  if(byte_count)//If there are any bytes then deal with them
  {
    if(start = 1)
    {
      count1 = 220;
      serial_connection.println(String(count1));//Then send an incrmented string back to the python script
      serial_connection.println(String(count));
      count++;//Increment the line counter
      if(count > 360)
      {
        count = 0;
      }
    }
    else
    {
      int first_bytes = byte_count;
      for(i=0;i<first_bytes;i++)//Handle the number of incoming bytes
      {
      inChar=serial_connection.read();//Read one byte
      inData[i]=inChar;//Put it into a character string(array)
      }
      Serial.println(inData);//Print to the monitor what was detected
      if(inData == 100)
      {
        start = 1;
      }
    }
  }
  delay(20);//Pause for a moment 
}
