#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
MPU6050 mpu ;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[128]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//interrupt check
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

Servo ESC ;   
Servo serv1 ;
Servo serv2 ;
Servo serv3 ;
Servo serv4 ;
Servo serv5 ;
Servo serv6 ;


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
  Serial.println("Initialising the mpu");
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  
Serial.println(F("Type any character to begin: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    devStatus = mpu.dmpInitialize();

    // gyro offsets, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

   

}


void loop() {


      // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize){
          //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) 
      fifoCount = mpu.getFIFOCount();
  }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
  while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

       
    }

potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
ESC.write(potValue);    // Send the signal to the ESC


//SERVO ONE
mpu.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax1 = map(ax, -17000,17000,200,90);
ay1 = map(ay, -17000,17000,200,90);
val1 = min(ax1,ay1);    //CWsens = min ( CWsens, min(CWsens1, CWsens2) ) ; member to use this for multiple inputs with min
Serial.println(ax);
Serial.println(ay);
serv1.write(val1);


//SERVO TWO
mpu.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax2 = map(ax, -17000,17000,0,120);
//ay2 = map(ay, -17000,17000,0,120);  //removed the y value as I dont think i need it? will inspect again next week
//val2 = max (ax2,ay2);
serv2.write(ax2);

//SERVO THREE
mpu.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax3 = map(ax, -17000,17000,0,60);
ay3 = map (ay,-17000,17000,0,120);
val3 = min (ax3,ay3);
serv3.write(val3);


//SERVO FOUR
mpu.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax4 = map(ax, -17000,17000,0,90);
ay4 = map (ay,-17000,17000,90,0);
val4 = max (ax4,ay4);
serv4.write(val4);

//SERVO FIVE
mpu.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax5 = map(ax, -17000,17000,0,90);
//ay5 = map (ay,-17000,17000,0,90);
//val5 = min (ax5,ay5);
serv5.write(ax5);

//SERVO SIX
mpu.getMotion6 (&ax,&ay,&az,&gx,&gy,&gz);
ax6 = map(ax, -17000,17000,90,0);
ay6 = map (ay,-17000,17000,0,90);
val6 = max (ax6,ay6);
serv6.write(val6);


}
