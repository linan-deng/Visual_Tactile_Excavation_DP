String comdata = "";

#define DIRPIN_z 8 //改变电机z方向
#define DIRPIN_y 9 //改变电机y方向
#define DIRPIN_x 10 //改变电机x方向

#define STEPPIN_z 5 //电机z脉冲
#define STEPPIN_y 6 //电机z脉冲
#define STEPPIN_x 7 //电机z脉冲

#define FLAG_z 2 //电机z归零初始化标志
#define FLAG_y 3 //电机z归零初始化标志
#define FLAG_x 4 //电机z归零初始化标志

void setup(){
    pinMode(FLAG_x, INPUT_PULLUP);
    pinMode(STEPPIN_x, OUTPUT);
    pinMode(DIRPIN_x, OUTPUT);
    pinMode(FLAG_y, INPUT_PULLUP);
    pinMode(STEPPIN_y, OUTPUT);
    pinMode(DIRPIN_y, OUTPUT);
    pinMode(FLAG_z, INPUT_PULLUP);
    pinMode(STEPPIN_z, OUTPUT);
    pinMode(DIRPIN_z, OUTPUT);
    Serial.begin(9600);
    int init_time_delay = 10;
//    pinMode(STOP, OUTPUT);
//    attachInterrupt(0,onchange,CHANGE);

    while(true){
      int flag_z =  digitalRead(FLAG_z);
      if (flag_z == 1){
        digitalWrite(DIRPIN_z,LOW);
        digitalWrite(STEPPIN_z,HIGH);
        delayMicroseconds(init_time_delay);
        digitalWrite(STEPPIN_z,LOW);
        delayMicroseconds(init_time_delay);
      }
      if (flag_z == 0){
        digitalWrite(DIRPIN_z,HIGH);
        for (long i = 0 ; i < 40000; ++i)//1000个脉冲一圈，滑块走1mm
        {
          digitalWrite(STEPPIN_z,HIGH);
          delayMicroseconds(init_time_delay);
          digitalWrite(STEPPIN_z,LOW);
          delayMicroseconds(init_time_delay);
        }
        break;
      }
    }
    // delayMicroseconds(500);
    // while(true){
    //   int flag_x =  digitalRead(FLAG_x);
    //   if (flag_x == 1){
    //     digitalWrite(DIRPIN_x,LOW);
    //     digitalWrite(STEPPIN_x,HIGH);
    //     delayMicroseconds(init_time_delay);
    //     digitalWrite(STEPPIN_x,LOW);
    //     delayMicroseconds(init_time_delay);
    //   }
    //   if (flag_x == 0){
    //     digitalWrite(DIRPIN_x,HIGH);
    //     for (long i = 0 ; i < 50000; ++i)//1000个脉冲一圈，滑块走1mm
    //     {
    //       digitalWrite(STEPPIN_x,HIGH);
    //       delayMicroseconds(init_time_delay);
    //       digitalWrite(STEPPIN_x,LOW);
    //       delayMicroseconds(init_time_delay);
    //     }
    //     break;
    //   }
    // }
    // delayMicroseconds(500);
    // while(true){
    //   int flag_y =  digitalRead(FLAG_y);
    //   if (flag_y == 1){
    //     digitalWrite(DIRPIN_y,LOW);
    //     digitalWrite(STEPPIN_y,HIGH);
    //     delayMicroseconds(init_time_delay);
    //     digitalWrite(STEPPIN_y,LOW);
    //     delayMicroseconds(init_time_delay);
    //   }
    //   if (flag_y == 0){
    //     digitalWrite(DIRPIN_y,HIGH);
    //     for (long i = 0 ; i < 50000; ++i)//1000个脉冲一圈，滑块走1mm
    //     {
    //       digitalWrite(STEPPIN_y,HIGH);
    //       delayMicroseconds(init_time_delay);
    //       digitalWrite(STEPPIN_y,LOW);
    //       delayMicroseconds(init_time_delay);
    //     }
    //     break;
    //   }
    // }
  }

void loop()
    {
      int j = 0;
      long numdata[9] = {0};
      int mark = 0;
      int flag_x =  digitalRead(FLAG_x);
      int flag_y =  digitalRead(FLAG_y);
      int flag_z =  digitalRead(FLAG_z);
      int init_time_delay = 10;
      
      while (Serial.available() > 0 && flag_z==1)
      {
        comdata += char(Serial.read());  
        delay(2);    
        mark = 1;
      }
      if(mark == 1)
      {
        for(int i = 0; i < (comdata.length()) ; i++)
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
        Serial.print(comdata);
        comdata = String("");
        // direction_x = numdata[0];
        // pulse_x = numdata[1];
        // delaytime_x = numdata[2];
        // direct_y = numdata[3];
        // pulse_y = numdata[4];
        // delaytime_y = numdata[5];
        // direct_z = numdata[6];
        // pulse_z = numdata[7];
        // delaytime_z = numdata[8];
        // Serial.print('\n');
        // Serial.print(direction_x);
        // Serial.print('\n');
        // Serial.print(pulse_x);
        // Serial.print('\n');
        // Serial.print(delaytime_x);
        
      //  if (numdata[0] == 0)//x轴靠近电机的方向
      //  {
      //   digitalWrite(DIRPIN_x,LOW);
      //   for (long i = 0 ; i < numdata[1]; ++i) 
      //   {
      //     digitalWrite(STEPPIN_x,HIGH);
      //     delayMicroseconds(numdata[2]);
      //     digitalWrite(STEPPIN_x,LOW);
      //     delayMicroseconds(numdata[2]);
      //     flag_x =  digitalRead(FLAG_x);
      //     if (flag_x == 0){
      //       digitalWrite(DIRPIN_x,HIGH);
      //       for (long i = 0 ; i < 100000; ++i)//1000个脉冲一圈，滑块走1mm
      //       {
      //         digitalWrite(STEPPIN_x,HIGH);
      //         delayMicroseconds(init_time_delay);
      //         digitalWrite(STEPPIN_x,LOW);
      //         delayMicroseconds(init_time_delay);
      //       }
      //       break;
      //     }
      //   }
      //  }
      //  if (numdata[0] == 1)
      //  {
      //   digitalWrite(DIRPIN_x,HIGH);
      //   for (long i = 0 ; i < numdata[1]; ++i) 
      //   {
      //     digitalWrite(STEPPIN_x,HIGH);
      //     delayMicroseconds(numdata[2]);
      //     digitalWrite(STEPPIN_x,LOW);
      //     delayMicroseconds(numdata[2]);
      //     flag_x =  digitalRead(FLAG_x);
      //     if (flag_x == 0){
      //       digitalWrite(DIRPIN_x,HIGH);
      //       for (long i = 0 ; i < 100000; ++i)//1000个脉冲一圈，滑块走1mm
      //       {
      //         digitalWrite(STEPPIN_x,HIGH);
      //         delayMicroseconds(init_time_delay);
      //         digitalWrite(STEPPIN_x,LOW);
      //         delayMicroseconds(init_time_delay);
      //       }
      //       break;
      //     }
      //   }
      //  }
       
      //  if (numdata[3] == 0)//y轴靠近电机的方向
      //  {
      //   digitalWrite(DIRPIN_y,LOW);
      //   for (long i = 0 ; i < numdata[4]; ++i) 
      //   {
      //     digitalWrite(STEPPIN_y,HIGH);
      //     delayMicroseconds(numdata[5]);
      //     digitalWrite(STEPPIN_y,LOW);
      //     delayMicroseconds(numdata[5]);
      //     flag_y =  digitalRead(FLAG_y);
      //     if (flag_y == 0){
      //       digitalWrite(DIRPIN_y,HIGH);
      //       for (long i = 0 ; i < 100000; ++i)//1000个脉冲一圈，滑块走1mm
      //       {
      //         digitalWrite(STEPPIN_y,HIGH);
      //         delayMicroseconds(init_time_delay);
      //         digitalWrite(STEPPIN_y,LOW);
      //         delayMicroseconds(init_time_delay);
      //       }
      //       break;
      //     }
      //   }
      //  }
      //  if (numdata[3] == 1)
      //  {
      //   digitalWrite(DIRPIN_y,HIGH);
      //   for (long i = 0 ; i < numdata[4]; ++i) 
      //   {
      //     digitalWrite(STEPPIN_y,HIGH);
      //     delayMicroseconds(numdata[5]);
      //     digitalWrite(STEPPIN_y,LOW);
      //     delayMicroseconds(numdata[5]);
      //     flag_y =  digitalRead(FLAG_y);
      //     if (flag_y == 0){
      //       digitalWrite(DIRPIN_y,HIGH);
      //       for (long i = 0 ; i < 100000; ++i)//1000个脉冲一圈，滑块走1mm
      //       {
      //         digitalWrite(STEPPIN_y,HIGH);
      //         delayMicroseconds(init_time_delay);
      //         digitalWrite(STEPPIN_y,LOW);
      //         delayMicroseconds(init_time_delay);
      //       }
      //       break;
      //     }
      //   }
      //  }

       if (numdata[6] == 0)//z轴靠近电机的方向
       {
        digitalWrite(DIRPIN_z,HIGH);
        for (long i = 0 ; i < numdata[7]; ++i) 
        {
          digitalWrite(STEPPIN_z,HIGH);
          delayMicroseconds(numdata[8]);
          digitalWrite(STEPPIN_z,LOW);
          delayMicroseconds(numdata[8]);
          flag_z =  digitalRead(FLAG_z);
          if (flag_z == 0){
            digitalWrite(DIRPIN_z,LOW);
            for (long i = 0 ; i < 100000; ++i)//1000个脉冲一圈，滑块走1mm
            {
              digitalWrite(STEPPIN_z,HIGH);
              delayMicroseconds(init_time_delay);
              digitalWrite(STEPPIN_z,LOW);
              delayMicroseconds(init_time_delay);
            }
            break;
            }
        }
       }
       if (numdata[6] == 1)
       {
        digitalWrite(DIRPIN_z,LOW);
        for (long i = 0 ; i < numdata[7]; ++i) 
        {
          digitalWrite(STEPPIN_z,HIGH);
          delayMicroseconds(numdata[8]);
          digitalWrite(STEPPIN_z,LOW);
          delayMicroseconds(numdata[8]);
          flag_z =  digitalRead(FLAG_z);
          if (flag_z == 0){
            digitalWrite(DIRPIN_z,HIGH);
            for (long i = 0 ; i < 100000; ++i)//1000个脉冲一圈，滑块走1mm
            {
              digitalWrite(STEPPIN_z,HIGH);
              delayMicroseconds(init_time_delay);
              digitalWrite(STEPPIN_z,LOW);
              delayMicroseconds(init_time_delay);
            }
            break;
          }
        }
       }
        mark = 0;
      }
    }
