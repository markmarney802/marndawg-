#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

Servo ESC ;   

//I have 6 servos
Servo serv[6] ;

MPU6050 sensor ;

int potValue; //for the potentiometer controlling the ESC

//Sensor values
int16_t ax, ay, az ;
int16_t gx, gy, gz ;

//Transformation values for motors, Mark please double check these... 
//I saw some transformations were off, so I assume giving 0 also means no transformation...
int16_t angle_min_x[6]={200, 0,   0,   0,   0,   90};
int16_t angle_max_x[6]={90,  120, 60,  90,  90,  0};
int16_t angle_min_y[6]={200, 0,   0,   90,  0,   0};
int16_t angle_max_y[6]={90,  0,   90,  0,   0,   90};

void setup() {
  for (int i=0; i<6; i++)
  {
    serv[i].attach(9-i); //attach Servo one to pin 9(D9)
    //I am sorry I didnt see how to format text in one println command... this print statement is hacky
    Serial.print("Attaching Servo ");
    Serial.print(i);
    Serial.print(" to pin D");
    Serial.print(9-i);
    Serial.print("\n");
  }
  ESC.attach(10,1000,2000) ;  //(pin, min pulse width, max pulse width in microseconds)
  
  Wire.begin();
  
  Serial.begin(115200) ;
  Serial.println("Initialising the sensor");
  
  sensor.initialize();

  Serial.println (sensor.testConnection()? "IMU Successfully Connected" : "IMU Connection Failed");

  delay (1000);

  Serial.println("Taking Values from IMU");

  delay(1000);
}

void loop() {

   potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
   potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
   ESC.write(potValue);    // Send the signal to the ESC

   sensor.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
   for (int i=0; i<6; i++)
   {
     int16_t x = map(ax, -17000,17000,angle_min_x[i],angle_max_x[i]);
     int16_t y = map(ay, -17000,17000,angle_min_y[i],angle_min_y[i]);
     int16_t val1 = min(x,y);
     //Unneeded printstatements will slow down this code, possibly turn these off when you are done debugging
     Serial.println(i);
     Serial.println(x);
     Serial.println(y);
     serv[i].write(val1);
   }

}
