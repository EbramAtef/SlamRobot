#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>


//#include <SoftwareSerial.h>

//pin constants
const int servoFrontPin = 12;
const int sensorFrontPin = A0;
const int sensorBackPin = A1;

//Encoder Pins
const int MotorLeftPin1 = 3;
const int MotorLeftPin2 = 24;
const int MotorRightPin1 =2;
const int MotorRightPin2 =22;

//distance sensor pins
const int TrigPin = 

//MOTOR CONTROL Right
const int MotorRightENPin = 4;
const int MotorRightIN1 = 30;
const int MotorRightIN2 = 31;

//MOTOR CONTROL Left
const int MotorLeftENPin = 5;
const int MotorLeftIN1 = 32;
const int MotorLeftIN2 = 33;

const int WifiStatus = 7;
const int motorSpeed = 155;
////////////


volatile int MotorLeft = 0;
volatile int MotorRight = 0;
unsigned long previousTime;
unsigned long previousTime2;
String readString = "";
int degree = 10;
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;


/////////////////////
//initialzing Libraries
Servo servofront;  // create servo object to control a servo

void setup() {
  //Motor PINS
  pinMode(MotorRightENPin,OUTPUT);
  pinMode(MotorRightIN1,OUTPUT);
  pinMode(MotorRightIN2,OUTPUT);
  pinMode(MotorLeftENPin,OUTPUT);
  pinMode(MotorLeftIN1,OUTPUT);
  pinMode(MotorLeftIN2,OUTPUT);

  //Servo PINS
  pinMode(servoFrontPin,OUTPUT);

  //Sensor PINS
  pinMode(sensorFrontPin,INPUT);
  pinMode(sensorBackPin,INPUT);

  //encoder PINS
  pinMode(MotorLeftPin1,INPUT);
  pinMode(MotorLeftPin2,INPUT);
  pinMode(MotorRightPin1,INPUT);
  pinMode(MotorRightPin2,INPUT);

  pinMode(WifiStatus,INPUT);
  servofront.attach(servoFrontPin);
  Serial1.begin(115200);
  Serial.begin(9600);
  Wire.begin();
  delay(10);
  delay(10);
  while(digitalRead(WifiStatus) != LOW);
  //attachInterrupt(digitalPinToInterrupt(MotorLeftPin1),countL,RISING);
  //attachInterrupt(digitalPinToInterrupt(MotorRightPin1),countR,RISING);
  previousTime = millis();
  Serial.println("Setupcomplete");
}
void countL()
{
  if(MotorLeft == 32767)
    MotorLeft = 0;
  else if(MotorLeft == -32768)
    MotorLeft = 0;
  if(digitalRead(MotorLeftPin2) == LOW)
    MotorLeft++;
  else
    MotorLeft--;
}
void countR()
{
  if(MotorRight == 32767)
    MotorRight = 0;
  else if(MotorRight == -32768)
    MotorRight = 0;
  if(digitalRead(MotorRightPin2) == HIGH)
    MotorRight++;
  else
    MotorRight--;
}
void SendEncoderData ()
{
  Serial1.print("encoder:");
  Serial1.print(MotorLeft);
  Serial1.print(",");
  Serial1.print(MotorRight);
  Serial1.println("#");
}
int get_gp2d12 (int value) 
{
  if (value < 10) value = 10;
  return ((67870.0 / (value - 3.0)) - 40.0)*0.1;
}
void loop() {
  /*unsigned long currentTime = millis();
  long int rpmL = 0;
  long int rpmR = 0;
  if(currentTime - previousTime >= 50)
  {
    detachInterrupt(digitalPinToInterrupt(MotorLeftPin)); //Interrupts are disabled
    rpmL = (MotorLeft*120)/59;
    MotorLeft = 0;
    attachInterrupt(digitalPinToInterrupt(MotorLeftPin),countL,RISING);
    detachInterrupt(digitalPinToInterrupt(MotorRightPin)); //Interrupts are disabled
    rpmR = (MotorRight*120)/59;
    MotorRight = 0;
    attachInterrupt(digitalPinToInterrupt(MotorRightPin),countR,RISING);
    previousTime = currentTime;
  }*/
  recvWithStartEndMarkers();
  if(newData)
  {
    Serial.println(receivedChars);
    char value[4]; 
    value[0] = receivedChars[2];
    value[1] = receivedChars[3];
    value[2] = receivedChars[4];
    value[3] = '\0';
    int x = atoi(value);
    if(receivedChars[1] == 'R')
    {
      if(receivedChars[6] == '0')
      {
        digitalWrite(MotorRightIN1,HIGH);
        digitalWrite(MotorRightIN2,LOW);
      }
      else
      {
        digitalWrite(MotorRightIN2,HIGH);
        digitalWrite(MotorRightIN1,LOW);
      }
      analogWrite(MotorRightENPin,x);
    }
    else
    {
      if(receivedChars[6] == '0')
      {
        digitalWrite(MotorLeftIN1,HIGH);
        digitalWrite(MotorLeftIN2,LOW);
      }
      else
      {
        digitalWrite(MotorLeftIN2,HIGH);
        digitalWrite(MotorLeftIN1,LOW);
      }
      analogWrite(MotorLeftENPin,x);
    }
    newData = false;
  }
  unsigned long currentTime = millis();
  if(currentTime - previousTime >= 500)
  {
    servofront.write(degree); 
    servoback.write(degree);
    char deg[4];
    //itoa(degree,deg,10);
    snprintf(deg, 4,"%03d",degree);
    char value[3];
    int mvalue = analogRead (A3);
    int range = get_gp2d12 (mvalue);
    itoa(range,value,10);
    char data[11];
    data[0]  = '<';
    data[1]  = 'S';
    data[2]  = 'F';
    data[3]  = value[0];
    data[4]  = value[1];
    data[5]  = 'A';
    data[6]  = deg[0];
    data[7]  = deg[1];
    data[8] = deg[2];
    data[9] = '>';
    data[10] = '\0';
    Serial1.write(data);
    Serial.println(range);
    Serial.println(mvalue);
    previousTime = currentTime;
    degree += 10;
    if(degree == 170)
    {
      degree = 10;
    }
  }
}
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
