#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

Servo ESC ;   

Servo serv1 ;
Servo serv2 ;
Servo serv3 ;
Servo serv4 ;
Servo serv5 ;
Servo serv6 ;

MPU6050 sensor ;

int potValue; //for the potentiometer controlling the ESC

int16_t ax, ay, az ;
int16_t ax1, ay1, az1 ;
int16_t ax2, ay2, az2 ;
int16_t ax3, ay3, az3 ;
int16_t ax4, ay4, az4 ;
int16_t ax5, ay5, az5 ;
int16_t ax6, ay6, az6 ;

int16_t gx, gy, gz ;
int16_t gx1, gy1, gz1 ;
int16_t gx2, gy2, gz2 ;
int16_t gx3, gy3, gz3 ;
int16_t gx4, gy4, gz4 ;
int16_t gx5, gy5, gz5 ;
int16_t gx6, gy6, gz6 ;

int16_t val1 ;
int16_t val2 ;
int16_t val3 ;
int16_t val4 ;
int16_t val5 ;
int16_t val6 ;


void setup() {
  serv1.attach(9) ; //attach Servo one to pin 9(D9)
  serv2.attach(8);
  serv3.attach(7);
  serv4.attach(6);
  serv5.attach(5);
  serv6.attach(4);

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


//SERVO ONE
sensor.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax1 = map(ax, -17000,17000,200,90);
ay1 = map(ay, -17000,17000,200,90);
val1 = min(ax1,ay1);
Serial.println(ax);
Serial.println(ay);
serv1.write(val1);


//SERVO TWO
sensor.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax2 = map(ax, -17000,17000,0,120);
//ay2 = map(ay, -17000,17000,0,120);  //removed the y value as I dont think i need it? will inspect again next week
//val2 = max (ax2,ay2);
serv2.write(ax2);

//SERVO THREE
sensor.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax3 = map(ax, -17000,17000,0,60);
ay3 = map (ay,-17000,17000,0,90);
val3 = min (ax3,ay3);
serv3.write(val3);


//SERVO FOUR
sensor.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax4 = map(ax, -17000,17000,0,90);
ay4 = map (ay,-17000,17000,90,0);
val4 = max (ax4,ay4);
serv4.write(val4);

//SERVO FIVE
sensor.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax5 = map(ax, -17000,17000,0,90);
//ay5 = map (ay,-17000,17000,0,90);
//val5 = min (ax5,ay5);
serv5.write(ax5);

//SERVO SIX
sensor.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax6 = map(ax, -17000,17000,90,0);
ay6 = map (ay,-17000,17000,0,90);
val6 = max (ax6,ay6);
serv6.write(val6);


}
