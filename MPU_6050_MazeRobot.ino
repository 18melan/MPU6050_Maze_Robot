#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "PID_v1.h"
#include <NewPing.h>
#include <SparkFun_TB6612.h>

#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9
#define STBY 6
Motor leftMotor = Motor(AIN1, AIN2, PWMA, -1, STBY); 
Motor rightMotor = Motor(BIN1, BIN2, PWMB, -1, STBY);

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 300
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int pingSpeed = 22; //ultrasonic refresh rate in ms
unsigned long pingTimer;
double distance = 300;

#define INTERRUPT_PIN 2
#define LED_PIN 13

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float startYaw, startPitch, startRoll;
float currentYaw;

// interrupt detection routine
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

double Setpoint, Input, Output;

double Kp=18, Ki=0, Kd=4.6;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int maze[] = {90, 90, 90, -90, -90, 90, 90};
int turnDistance = 22;
int segment = 0;
bool turning = true;

long previousMillis = 0;  
long interval = 1000;

void setup() {
    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(115200);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-80);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(-28);
    mpu.setXAccelOffset(-72);
    mpu.setYAccelOffset(599);
    mpu.setZAccelOffset(685);
 
    if (devStatus == 0) { // make sure it worked (returns 0 if so)
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else { //Error
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    blink(3);
    Serial.println("Waiting for offsets to calculate...");
    for(int i = 0; i < 1850; i++) {
      while (!mpuInterrupt && fifoCount < packetSize);
      updateMPU6050();
    }
    Serial.println("Done!");
    
    startYaw = ypr[0] * 180/M_PI;
    startPitch = ypr[1] * 180/M_PI;
    startRoll = ypr[2] * 180/M_PI;
    
    pinMode(LED_PIN, OUTPUT);

    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255);
    myPID.SetSampleTime(4); // looptime in ms (4ms = 250hz)
    
    

    blink(3);
    unsigned long t = millis();
    while(millis() < t + 3000) {
      while (!mpuInterrupt && fifoCount < packetSize);
      updateMPU6050();
    }
    blink(1);
    
    digitalWrite(LED_PIN, true);
    pingTimer = millis();
}

void loop() {
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    
    Input = (int) (currentYaw + 360) % 360;
    while(Input - Setpoint < -180) {
        Input += 360;
    }
    while(Input - Setpoint > 180) {
        Input -= 360;
    }

    myPID.Compute();

    //Serial.println(getSpeed(distance));


//    unsigned long currentMillis = millis();
//    if(currentMillis - previousMillis > interval) {
//      previousMillis = currentMillis; 
//      Setpoint += 90;
//      
//    }
    

    drive(Output, getSpeed(distance)); //190);
    Serial.println((ypr[2] * 180/M_PI) - startRoll);
    if(!turning) {
      if(distance < turnDistance) {
        int lastSegment = sizeof(maze);
        if(segment == lastSegment) {
          
        }
        Setpoint += maze[segment];
        segment++;
        turning = true;
      }
    }
    if(distance > turnDistance + 10) { //10
      turning = false;
    }
  }

    if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
      pingTimer += pingSpeed;      // Set the next ping time.
      sonar.ping_timer(updateSonar); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
    }
 
  updateMPU6050();
}

void updateMPU6050() {
  mpuInterrupt = false; // reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount(); // get current FIFO count

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check for overflow
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) { // otherwise, check for DMP data ready interrupt (this should happen frequently)
    
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); // wait for correct available data length
      
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize; // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      currentYaw = (ypr[0] * 180/M_PI) - startYaw;
  }
}

void updateSonar() {
  if(sonar.check_timer()) {
    distance = sonar.ping_result / US_ROUNDTRIP_CM;
  }
}

double getSpeed(double distance) {
  float s = 1.8*distance + 30;
  if(s > 255 || distance > 42) { //distance > 35
    return 255;
  }
  
  return s;
}

void drive(float turnAmt, float speed) { //turnAmt from -255 to 255, speed from 0 to 255
  float left, right;
  if(speed < 0) {
    speed = 0;
  }
  
  turnAmt = map(turnAmt, -255, 255, -1, 1); //convert turnAmt to percentage
  if(turnAmt > 0) {
    left = speed + turnAmt*(255-speed);
    right = speed - (turnAmt * speed);
  }
  else {
    right = speed + -turnAmt*(255-speed);
    left = speed - (-turnAmt * speed);
  }
  leftMotor.drive(left);
  rightMotor.drive(right);
}

void blink(int times) {
  for(int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, true);
    delay(200);
    digitalWrite(LED_PIN, false);
    delay(200); 
  }
}

