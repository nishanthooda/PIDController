#include <PID.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// The value received over serial communication from other bluetooth device
char blueToothVal;

// Used for tracking speed of all ESCs.
// It is initially set to 48 as the motors begin spinning at a value of approximately 49
double speed = 48;

// Used for tracking the current state of the MPU
double currentYaw = 0, prevYaw = -100000;
double currentPitch = 0;
double currentRoll = 0;

// For tracking if the yaw is stable yet
bool setProperYaw = false;

// The servo variables for each ESC. Note: the number is the pin to which the ESC is attached on the arduino
Servo ESC3;
Servo ESC5; 
Servo ESC6;
Servo ESC11;

// The PIDs for each axis
PID * yawPID = new PID(32, 0.02, 16, 0);
PID * pitchPID = new PID(32, 0.02, 16, 0);
PID * rollPID = new PID(32, 0.02, 16, 0);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // Set true if DMP init was successful
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// O  rientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// Indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

/**
 * Writes a speed to all of the ESCs
 * @param speed an integer speed from 1 to 100
 */
void SetSpeed(double speed) {
  int angle = MapDouble(speed, 0, 100, 0, 2300); //Sets servo positions to different speeds
  
  ESC3.writeMicroseconds(angle);
  ESC5.writeMicroseconds(angle);
  ESC6.writeMicroseconds(angle);
  ESC11.writeMicroseconds(angle);
}

/**
 * Writes speed to ESC3
 * @param speed an integer speed from 1 to 100
 */
void SetSpeedESC3(double speed) {
  int angle = MapDouble(speed, 0, 100, 0, 2300);
  ESC3.writeMicroseconds(angle);
}

/**
 * Writes speed to ESC5
 * @param speed an integer speed from 1 to 100
 */
void SetSpeedESC5(double speed) {
  int angle = MapDouble(speed, 0, 100, 0, 2300);
  ESC5.writeMicroseconds(angle);
}

/**
 * Writes speed to ESC6
 * @param speed an integer speed from 1 to 100
 */
void SetSpeedESC6(double speed) {
  int angle = MapDouble(speed, 0, 100, 0, 2300);
  ESC6.writeMicroseconds(angle);
}

/**
 * Writes speed to ESC11
 * @param speed an integer speed from 1 to 100
 */
void SetSpeedESC11(double speed) {
  int angle = MapDouble(speed, 0, 100, 0, 2300);
  ESC11.writeMicroseconds(angle);
}

/**
 * Custom map function that can map doubles
 * @param x The value to be mapped
 * @param in_min the lowest value that x can be as input to the function
 * @param in_max the highest value that x can be as input to the function
 * @param out_min the lowest value that the output can be
 * @param out_max the highest value that the output can be
 * @return The mapped value
 */
double MapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Clamps the output of the speed for an ESC to the range 48-60
 * @param speed The current speed value
 * @return A clamped value
 */
double clampESCSpeed(double speed) {
  if (speed > 60) return 60;
  if (speed < 48) return 48;
  return speed;
}

void setup() {

    // Join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();

        // 400kHz I2C clock (200kHz if CPU is 8MHz)
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // Initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // Verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // Make sure everything worked
    if (devStatus == 0) {
        // Turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // Prepare all of the ESCs 
    ESC3.attach(3); 
    ESC5.attach(5);
    ESC6.attach(6);
    ESC11.attach(11);
    
    // Make sure that the PIDs don't try to do too much work
    yawPID->SetOutputLimits(-1, 1);
    pitchPID->SetOutputLimits(-1, 1);
    rollPID->SetOutputLimits(-1, 1);
}

void loop() {
    // If  MPU programming failed, don't try to do anything
    if (!dmpReady) return;

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // Otherwise, check for DMP data ready interrupt
    } else if (mpuIntStatus & 0x02) {
        // Wait for correct available data length
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get the YPR from the MPU
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        currentYaw = ypr[0] * 180/M_PI;
        currentPitch = ypr[1] * 180/M_PI;
        currentRoll = ypr[2] * 180/M_PI;

        // Blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    
    // Update the input given to the PID
    yawPID->UpdateInput(currentYaw);
    pitchPID->UpdateInput(currentPitch);
    rollPID->UpdateInput(currentRoll);

    // Run the compute function on all of the PIDs
    yawPID->Compute();
    pitchPID->Compute();
    rollPID->Compute();

    // If there is data to read
    if (Serial.available()){
      blueToothVal = Serial.read(); 
    }
    // 'i' increases the base speed of the motors
    if (blueToothVal=='i' && speed < 100) {
      blueToothVal='n';
      speed+= 0.1;

    // 'd' decreases the base speed of the motors
    } else if (blueToothVal=='d' && speed > 1) {
      blueToothVal='n';
      speed-= 0.1;

    // 'k' should 'kill' the motors. For some reason this can cause unexpected behaviour in the ESCs. REFRAIN FROM USING THIS
    } else if (blueToothVal=='k') {
      while (1) {
        SetSpeedESC3(0);
        SetSpeedESC5(0);
        SetSpeedESC6(0);
        SetSpeedESC11(0);
      }
    }
    
    // Now use the PID values to adjust the flight
    // ONLY IF we have decided to start taking off (i.e. speed is greater than 48)
    if (speed > 48) {
      // Make sure that the PID's aren't trying to immediatly trying to correct the yaw
      if (!setProperYaw) {
        yawPID->SetSetPoint(currentYaw);
        setProperYaw = true;
      }
      double yawPIDOutput = yawPID->GetOutput();
      double pitchPIDOutput = pitchPID->GetOutput();
      double rollPIDOutput = rollPID->GetOutput();

      /**
       * YAW CORRECTION: COUNTER-CLOCKWISE IS NEGATIVE
       *  If quad is rotating counter-clockwise, PID output will be positive 
       *  (as PID calculates setpoint - input, which is 0 - (some negative value), which is positive)
       *  in order to make the quad rotate clock-wise now, we need to increase the speed of counter clockwise motors
       *  (i.e. 3 and 6)
       * PITCH CORRECTION: MOTOR THREE AND FIVE DOWN IS POSITIVE
       *  If quad pitch is positive, that means the PID output will be negative. This means that we need to subtract the
       *  PID value from 3 and 5 which are currently down.
       * ROLL CORRECTION: MOTOR THREE AND ELEVEN DOWN IS POSITIVE
       *  If quad roll is positive, that means the PID output will be negative. This means that we need to subtract the 
       *  PID value from 3 and 11 which are currently down.
       */
      double ESC3Speed = speed + yawPIDOutput - pitchPIDOutput - rollPIDOutput;
      double ESC5Speed = speed - yawPIDOutput - pitchPIDOutput + rollPIDOutput;
      double ESC6Speed = speed + yawPIDOutput + pitchPIDOutput + rollPIDOutput;
      double ESC11Speed = speed - yawPIDOutput + pitchPIDOutput - rollPIDOutput;

      SetSpeedESC3(clampESCSpeed(ESC3Speed));
      SetSpeedESC5(clampESCSpeed(ESC5Speed));
      SetSpeedESC6(clampESCSpeed(ESC6Speed));
      SetSpeedESC11(clampESCSpeed(ESC11Speed));
    }
}


