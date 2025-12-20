#include <Servo.h>

Servo servo_1;  // create servo object to control a servo
Servo servo_2;  // create servo object to control a servo
Servo servo_3;  // create servo object to control a servo
Servo servo_4;  // create servo object to control a servo
// twelve servo objects can be created on most boards

void setup() {
  Serial.begin(9600);
  servo_1.attach(3);  // attaches the servo on pin 9 to the servo object
  servo_2.attach(5);
  servo_3.attach(6);
  servo_4.attach(10);
  servo_1.write(1);
  servo_2.write(179);
  servo_3.write(1);
  servo_4.write(179);
}

String comdata = "";
int numdata[4] = {0};
void loop(){
  if (Serial.available() > 0)
  {
    int j = 0;
    while (Serial.available() > 0)
    {
      comdata += char(Serial.read());
      delay(2);
    }
    Serial.println(comdata);
    Serial.println(comdata.length());
    if (comdata[0] == '#')
    {
      for (int i = 1; i < comdata.length(); i++)
      {
        if(comdata[i] == ',')
        {
          j++;  
        }  
        else
        {
          numdata[j] = numdata[j] * 10 + (comdata[i] - '0');
        }
      }
      comdata = String("");
      delay(20);
      if(numdata[0] > 70)
      {
        numdata[0] = 70;
      }
      if(numdata[1] > 90)
      {
        numdata[1] = 90;
      }
      if(numdata[2] > 70)
      {
        numdata[2] = 70;
      }
      if(numdata[3] > 90)
      {
        numdata[3] = 90;
      }
//      Serial.println(numdata[0]);
//      Serial.println(numdata[1]);
//      Serial.println(numdata[2]);
//      Serial.println(numdata[3]);
      servo_1.write(numdata[0]);
      servo_2.write(179-numdata[1]);
      servo_3.write(numdata[2]);
      servo_4.write(179-numdata[3]);
      delay(10);
      numdata[0] = 0;
      numdata[1] = 0;
      numdata[2] = 0;
      numdata[3] = 0;
    }
    else
    {
      comdata = String("");
    }
  }
  delay(10);
}
