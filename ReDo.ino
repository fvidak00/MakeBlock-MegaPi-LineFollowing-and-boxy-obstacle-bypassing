#include <Wire.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include "MeMegaPi.h"

MeUltrasonicSensor ultraSensor(PORT_7);
MeUltrasonicSensor ultraSensorL(PORT_5);
MeUltrasonicSensor ultraSensorD(PORT_8);

MeMegaPiDCMotor motorD1(PORT1A);
MeMegaPiDCMotor motorD2(PORT1B);
MeMegaPiDCMotor motorL1(PORT2A);
MeMegaPiDCMotor motorL2(PORT2B);

uint8_t defaultMotorSpeed = 220;
uint8_t motorSpeed = defaultMotorSpeed;

const int DataPin = A9;
uint8_t Sensor_Data[3];

bool lastDirection = true;
//true = desno, false = lijevo

bool opMode = true;
//true = praćenje linije, false = zaobilaženje prepreke

int blockDistance = 30;
int counter = 0;

void setup(){}

void loop()
{
    //Praćenje linije
    if (opMode == true)
    {
        if (ultraSensor.distanceCm() < 15)
        {
          opMode = false;
        }
        else
        {
          
          switch (GetValue())
          {
              case 0:
                  motorD1.stop();
                  motorD2.stop();
                  motorL1.stop();
                  motorL2.stop();
                  break;
              case 63:
                  if (lastDirection)
                    TurnRight();
                  else
                    TurnLeft();
                  break;
              case 62:
              case 61:
              case 60:
                  TurnLeft();
                  break;
              case 31:
              case 47:
              case 15:
                  TurnRight();
                  break;
              default:
                  GoForward();
                  break;
          }
        }
    }

    //Zaobilaženje prepreke
    else
    {
        //Ako je zadnji put desno skrenio
        if (lastDirection == true)
        {
            AlignRight(1750);
            StopIt(100);
            while (counter < 2)
            {
                while (ultraSensorL.distanceCm() < blockDistance)
                {
                    GoForward();
                }
                GoForward(250);
                StopIt(100);
                AlignLeft(2000);
                StopIt(100);
                GoForward(850);
                ++counter;
            }
            StopIt(100);
            while(GetValue() == 63)
            {
               GoForward();
            }
            StopIt(1000);
            AlignRight(1000);
            while(GetValue()==63)
            {
              GoForward();
            }
            counter=0;
            StopIt(100);
        }
        //Ako je zadnji put livo skrenio
        else
        {
            AlignLeft(2000);
            StopIt(100);
            while(counter<2)
            {
              while(ultraSensorD.distanceCm() < blockDistance)
              {
                GoForward();
              }
              GoForward(250);
              StopIt(100);
              AlignRight(1650);
              StopIt(100);
              GoForward(850);
              ++counter;
            }
            while(GetValue()==63)
            {
              if(ultraSensorD.distanceCm() < 7)
              {
                AlignLeft(100);
              }
              else
              {
                GoForward();
              }
            }
            StopIt(100);
            AlignLeft(1000);
            while(GetValue()==63)
            {
              GoForward();
            }
            counter=0;
            StopIt(100);
        }
        opMode = true;
    }
}

void StopIt(int delayLength)
{
    motorD1.stop();
    motorD2.stop();
    motorL1.stop();
    motorL2.stop();
    delay(delayLength);
}

void AlignLeft(int delayLength)
{
    motorD1.stop();
    motorD2.stop();
    motorL1.stop();
    motorL2.stop();
    MotorD(motorSpeed/2);
    MotorL(-motorSpeed/2);
    lastDirection = false;
    delay(delayLength);
}
void AlignRight(int delayLength)
{
    motorD1.stop();
    motorD2.stop();
    motorL1.stop();
    motorL2.stop();
    MotorL(motorSpeed/2);
    MotorD(-motorSpeed/2);
    lastDirection = true;
    delay(delayLength);
}

void GoForward()
{
    MotorD(motorSpeed);
    MotorL(motorSpeed);
}
void GoForward(int delayLength)
{
    MotorD(motorSpeed);
    MotorL(motorSpeed);
    delay(delayLength);
}

void TurnLeft()
{
    motorD1.stop();
    motorD2.stop();
    motorL1.stop();
    motorL2.stop();
    MotorD(motorSpeed/2);
    MotorL(-motorSpeed/2);
    lastDirection = false;
}
void TurnRight()
{
    motorD1.stop();
    motorD2.stop();
    motorL1.stop();
    motorL2.stop();
    MotorL(motorSpeed/2);
    MotorD(-motorSpeed/2);
    lastDirection = true;
}


void MotorD(int mSpeed)
{
    motorD1.run(-mSpeed); /* value: between -255 and 255. */
    motorD2.run(-mSpeed); /* value: between -255 and 255. */
}
void MotorL(int mSpeed)
{
    motorL1.run(mSpeed);
    motorL2.run(mSpeed);
}


uint8_t GetValue()
{  
    long time_out_flag = 0;
    pinMode(DataPin, OUTPUT);
    digitalWrite(DataPin, LOW);
    delayMicroseconds(980);
    digitalWrite(DataPin, HIGH);
    delayMicroseconds(40);
    pinMode(DataPin, INPUT_PULLUP);
    delayMicroseconds(50); 
    time_out_flag = millis();
    while((digitalRead(DataPin) == 0)&&((millis() - time_out_flag) < 6)); 
    time_out_flag = millis();
    while((digitalRead(DataPin) == 1)&&((millis() - time_out_flag) < 6));
    for(uint8_t k=0; k<3; k++)
    {
        Sensor_Data[k] = 0x00;
        for(uint8_t i=0;i<8;i++)
        {
            time_out_flag = millis(); 
            while(digitalRead(DataPin) == 0&&((millis() - time_out_flag) < 6));
            uint32_t HIGH_level_read_time = micros();
            time_out_flag = millis(); 
            while(digitalRead(DataPin) == 1&&((millis() - time_out_flag) < 6));
            HIGH_level_read_time = micros() - HIGH_level_read_time;
           if(HIGH_level_read_time > 50 && HIGH_level_read_time < 100)  
            {
                Sensor_Data[k] |= (0x80 >> i);
            }
        }
    }
    if (Sensor_Data[1] == (uint8_t)(~(uint8_t)Sensor_Data[0]))
    {
       return Sensor_Data[0];
    }
}
