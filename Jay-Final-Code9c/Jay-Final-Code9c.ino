/* ME366J - Team Jay Leno - Final Robot Code 9
 * Author: Matthew McDermott, Daniel Smarda (w/ inspiration from Miguel Grinberg)

ARDUINO (MEGA) SETUP:
=======================
Ultrasonic Sensors: 5V, GND, trig/echo: #1 (22,23); #2 (26,27), #3 (30,31), #4 (34,35)
Encoders: 5V, GND, Left (18, 19) and Right (2,3)
AMG88xx IR Camera: 5V, GND, SCL (21), SDA (20)
Motor Shield (L298N): 12V, GND (from battery), ENA-9, IN1-8, IN1-9, ENB-6, IN1-5, IN2-4
*/

#define ENABLE_L298N_MOTOR_DRIVER
#define ENABLE_NEWPING_DISTANCE_SENSOR_DRIVER
#define ENABLE_SAINSMART_ENCODER_SENSOR_DRIVER
#define ENABLE_ADAFRUIT_IR_CAMERA_DRIVER
#define LOGGING

#define RUN_TIME 10000000 //sec
// distance that triggers obstacle avoidance movement
#define TOO_CLOSE 20 // cm
// When checking vitals, total length robot will travel before turning if no obstacle is sensed
#define MAX_DISTANCE 200 // cm
// Amount of time robot reverses before turning when it sees an obstacle
#define REVERSE_TIME 1000 // ms

#define WHEEL_CIRCUMFRENCE 25.933847 // cm
#define CPR 64 //counts per revolution
// Internal gear ratio of gears in motor encoder
#define GEAR_RATIO 29 // 29:1 gear ratio
#define MSTOSEC 1000 //1000 ms in a second
// Target cruising speed when travelling in straight line
#define TARGET_SPEED 35 // cm/s
// If wheels are travelling slower than this velocity, robot is considered stuck
#define MIN_NOT_STUCK_VEL 3 // cm/s
// Robot will reverse and turn if it is stuck for this long
#define MAX_ALLOWED_STUCK_TIME 3000 // ms

// For IR 8x8 pixel frame processing to find animal.
// Uses sliding frame algorithm.
// Increasing FRAME_WIDTH increases both distance the target can be sensed from,
// and the likelihood the robot stops prematurely due to noise.
#define FRAME_WIDTH 1 // pixels
// Temp before sounding alarm
#define THRESHOLD_TEMP 29 // degrees C

#define BUZZ_PIN 12

#include "moving_average.h"
// in distance_sensor library
#include "logging.h"

#ifdef ENABLE_L298N_MOTOR_DRIVER
#include <L298N.h>
#include "L298N_motor_driver.h"
#define LEFT_MOTOR_INIT 9,7,8
#define RIGHT_MOTOR_INIT 4,6,5
#endif

#ifdef ENABLE_NEWPING_DISTANCE_SENSOR_DRIVER
// in NewPing library
#include <NewPing.h>
// in distance_sensor library
#include "newping_distance_sensor.h"
#define DISTANCE_SENSOR_INIT_1 22,23,MAX_DISTANCE
#define DISTANCE_SENSOR_INIT_2 26,27,MAX_DISTANCE
#define DISTANCE_SENSOR_INIT_3 30,31,MAX_DISTANCE
#define DISTANCE_SENSOR_INIT_4 34,35,MAX_DISTANCE
#endif


#ifdef ENABLE_SAINSMART_ENCODER_SENSOR_DRIVER
// in Encoder-master library
#include <Encoder.h>
// in encoder_sensor library
#include "SainSmart_encoder_sensor.h"
#define ENCODER_SENSOR_INIT_1 18,19
#define ENCODER_SENSOR_INIT_2 2,3
#endif

#ifdef ENABLE_ADAFRUIT_IR_CAMERA_DRIVER
#include <Adafruit_AMG88xx.h>
#endif

namespace Jay
{
  class Robot
  {
    public:

      Robot() 
          : leftMotor(LEFT_MOTOR_INIT), rightMotor(RIGHT_MOTOR_INIT), 
            ultrasonic1(DISTANCE_SENSOR_INIT_1), ultrasonic2(DISTANCE_SENSOR_INIT_2),
            ultrasonic3(DISTANCE_SENSOR_INIT_3), ultrasonic4(DISTANCE_SENSOR_INIT_4),
            distanceAverage1(MAX_DISTANCE), distanceAverage2(MAX_DISTANCE),
            distanceAverage3(MAX_DISTANCE), distanceAverage4(MAX_DISTANCE),
            leftEncoder(ENCODER_SENSOR_INIT_1), rightEncoder(ENCODER_SENSOR_INIT_2),
            IR()      
      {}
      // will be used to find the velocity of the wheels
      long oldCurrentTime;

      void initialize()
      {
        pinMode(BUZZ_PIN, OUTPUT);
        digitalWrite(BUZZ_PIN, LOW);
        IR.begin();
        currentRightSpeed = 60;

        delay(10000);

        oldCurrentTime = millis();
        endTime = millis() + RUN_TIME;

        move(TARGET_SPEED);  
      }

      // Pointer help from the following two links: \
      1d: https://stackoverflow.com/questions/3473438/return-array-in-a-function \
      2d: https://stackoverflow.com/questions/8617683/return-a-2d-array-from-a-function

      // Used example at this link for passing: \
      http://www.cppforschool.com/tutorial/array2.html

      bool petFound(float* arr) {
        // Below line removed because Thermistor shown to be unreliable
        //float roomTemp = amg.readThermistor();
        int thresholdTemp = THRESHOLD_TEMP;
        //Serial.print("Room Temp = ");
        //Serial.println(roomTemp);
        int frameWidth = FRAME_WIDTH;
        
        // Build 2D Array for processing from read IR camera 1x64 array
        float pixels2D[8][8] = {0};
        for (int r = 0; r < 8; r++) {
          for (int c = 0; c < 8; c++) {
            pixels2D[r][c] = arr[r*8 + c];
            Serial.print(pixels2D[r][c]);
            Serial.print(", ");
            if (c == 7) 
              Serial.println();
          }
        }
      
        // Processing
        // Return true if the average temp for any frame for the previously defined frameWidth
        // exceeds the previously defined thresholdTemp, i.e. if target is found
        for (int r = 0; r < 8 - frameWidth + 1; r++) {
          for (int c = 0; c < 8 - frameWidth + 1; c++) {
            float frameAvg = 0;
            float frameSum = 0;
            for (int R = r; R < r + frameWidth; R++) {
              for (int C = c; C < c + frameWidth; C++) {
                frameSum = frameSum + pixels2D[R][C];
              }
            }
            frameAvg = frameSum / (frameWidth * frameWidth);
            // Commented lines below found unuseful when thermistor determined to be unreliable
            //float animalTempDiff = frameAvg - roomTemp;
            //if (animalTempDiff > maxTempDiff || animalTempDiff < -maxTempDiff) {
              //return true;
            if (frameAvg >= thresholdTemp)
              return true;
          }
        }
        return false; 
      }  


      void run()
      {
        if (stopped())
          return;

        int distance1 = distanceAverage1.add(ultrasonic1.getDistance());
        int distance2 = distanceAverage2.add(ultrasonic2.getDistance());
        int distance3 = distanceAverage3.add(ultrasonic3.getDistance());
        int distance4 = distanceAverage4.add(ultrasonic4.getDistance());
        long leftPosition = leftEncoder.readEncoder();
        long rightPosition = rightEncoder.readEncoder();
        long currentTime = millis();

        int cpr = CPR;
        int msToSec = MSTOSEC;
        int gear_ratio = GEAR_RATIO;
        int circum = WHEEL_CIRCUMFRENCE;

        // Using encoder and conversion factors, calculates velocity of the wheel
        // cm/s
        leftVelocity = ((circum*msToSec)/(gear_ratio*CPR))*(leftPosition - oldLeftPosition)/(currentTime - oldCurrentTime);
        rightVelocity = ((circum*msToSec)/(gear_ratio*CPR))*(rightPosition - oldRightPosition)/(currentTime - oldCurrentTime);

        Serial.print("Left Velocity =");
        Serial.println(leftVelocity);
        Serial.print("Right Velocity =");
        Serial.println(rightVelocity);

        // Makes greater time interval (50ms) to ensure higher accuracy of velocity calculations
        if (currentTime - oldCurrentTime > 50) {
          timeStuck = timeStuck + currentTime-oldCurrentTime;
          leftVelocity = ((circum*msToSec)/(gear_ratio*CPR))*(leftPosition - oldLeftPosition)/(currentTime - oldCurrentTime);
          rightVelocity = ((circum*msToSec)/(gear_ratio*CPR))*(rightPosition - oldRightPosition)/(currentTime - oldCurrentTime);
          oldLeftPosition = leftPosition;
          oldRightPosition = rightPosition;
          oldCurrentTime = currentTime;
          if ((abs(leftVelocity) > MIN_NOT_STUCK_VEL && abs(rightVelocity) > MIN_NOT_STUCK_VEL))
            timeStuck = 0;
        }

        IR.readPixels(pixels);
        bool petisFound = petFound(pixels);

        log("state: %d, currentTime: %lu, distance1: %u, distance2: %u, distance3: %u, distance4: %u, leftPosition: %ld, rightPosition: %ld\n",
            state, currentTime, distance1, distance2, distance3, distance4, leftPosition, rightPosition);
        Serial.println();
        Serial.print("Pet found status:");
        Serial.println(petisFound);

          //Use this code if you don't want to reverse when see obstacle
//        if (doneRunning(currentTime) || petisFound) {
//          if (petisFound) buzz();
//          stop();
//        }
//        
//        else if (moving()) {
//          if (obstacleAhead(distance1, distance2))
//             turn(currentTime);
//          else
//            move(TARGET_SPEED);
//          }
//        else if (turning()) {
//          if (doneTurning(currentTime, distance1, distance2))
//            move(TARGET_SPEED);
//          }

        // Use this if statement if you want to reverse for a bit before turning
        if (doneRunning(currentTime) || petisFound) {
          if (petisFound) 
            buzz();
          stop();
          }
        else if (moving()) {
          if (obstacleAhead(distance1, distance2, distance3, distance4) || timeStuck > MAX_ALLOWED_STUCK_TIME)
             currentRightSpeed = reverse(currentTime, currentRightSpeed);
          else
            move(TARGET_SPEED);
          }
        else if (reversing()) {
          if (doneReversing(currentTime)) 
            turn(currentTime);
          else
            reverse(currentTime, currentRightSpeed);
          }
        else if (turning()) {
          if (doneTurning(currentTime, distance1, distance2, distance3, distance4))
            move(TARGET_SPEED);
          }
        }

    protected:
      
      bool moving() { return (state == stateMoving); }
      bool turning() { return (state == stateTurning); }
      bool stopped() { return (state == stateStopped); }
      bool reversing() { return (state == stateReversing); }
      
      void move(float targetSpeed)
      {
        if (state != stateMoving){
          leftSpeedOut = 75;
          rightSpeedOut = 75;
        }

        Serial.print("Left speed=");
        Serial.println(leftSpeedOut);
        Serial.print("Right speed=");
        Serial.println(rightSpeedOut);

        // Gradual increases in velocity magnitude
        // ensure that wheels don't slip
        // and ensure that torque on motor shaft is not excessive which would shear gear
        if(((targetSpeed - leftVelocity) > 0) && (leftSpeedOut < 250))
          leftSpeedOut = leftSpeedOut + 5;
        else if(((targetSpeed - leftVelocity) < 0) && (leftSpeedOut > -250))
          leftSpeedOut = leftSpeedOut - 5;
        
        if(((targetSpeed - rightVelocity) > 0) && (rightSpeedOut < 250))
          rightSpeedOut = rightSpeedOut + 5;
        else if(((targetSpeed - rightVelocity) < 0) && (rightSpeedOut > -250))
          rightSpeedOut = rightSpeedOut - 5;
 
        leftMotor.setSpeed(leftSpeedOut);
        rightMotor.setSpeed(rightSpeedOut);

        state = stateMoving;
      }

      bool turn(long currentTime)
      {
        // random statement determines whether robot turns left or right
        // when it sees an obstacle.
        // RandomSeed statement ensures these decisions are more truly random
        // each time the robot runs
        randomSeed(analogRead(0));
        if (random(2) == 0){
          // Motor speeds are relative out of 256;
          // Due to friction and other factors, speeds below
          // cause approximately same wheel turning velocity
          leftMotor.setSpeed(-115);
          rightMotor.setSpeed(140);
        }
        else {
          leftMotor.setSpeed(115);
          rightMotor.setSpeed(-140);
        }
        state = stateTurning;
        randomSeed(analogRead(0));
        // Instructs robot to turn for random amount of time
        endStateTime = currentTime + random(800, 2500);
      }

      int reverse(long currentTime, int currentRightSpeed)
      {
        if (state != stateReversing){
          currentRightSpeed = -60;
          endStateTime = currentTime + REVERSE_TIME;
        }
        leftMotor.setSpeed(currentRightSpeed);
        // Difference in speed command values due to friction and other factors
        rightMotor.setSpeed(currentRightSpeed-40);
        state = stateReversing;
        // Iterative speed increase to prevent excessive torque
        currentRightSpeed = currentRightSpeed - 25;
        return currentRightSpeed;
      }
      void stop()
      {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            state = stateStopped;
            delay(15000);
            digitalWrite(BUZZ_PIN, LOW);
      }


      void buzz()
      {
        digitalWrite(BUZZ_PIN, HIGH);

      }
      

      bool doneRunning(long currentTime)
      {
        return (currentTime >= endTime);
      }

      bool doneTurning(long currentTime, unsigned int distance1, unsigned int distance2, unsigned int distance3, unsigned int distance4)
      {
        if (currentTime >= endStateTime)
          return (distance1 > TOO_CLOSE || distance2 > TOO_CLOSE || distance3 > TOO_CLOSE || distance4 > TOO_CLOSE);
        return false;
      }

      bool doneReversing(long currentTime)
      {
     
        return (currentTime >= endStateTime);
      }

      bool obstacleAhead(unsigned int distance1, unsigned int distance2, unsigned int distance3, unsigned int distance4)
      {
        return (distance1 <= TOO_CLOSE || distance2 <= TOO_CLOSE || distance3 <= TOO_CLOSE || distance4 <= TOO_CLOSE);
      }

 
    private:
      Motor leftMotor;
      Motor rightMotor;
      DistanceSensor ultrasonic1;
      DistanceSensor ultrasonic2;
      DistanceSensor ultrasonic3;
      DistanceSensor ultrasonic4;
      MovingAverage<unsigned int, 3> distanceAverage1;
      MovingAverage<unsigned int, 3> distanceAverage2;
      MovingAverage<unsigned int, 3> distanceAverage3;
      MovingAverage<unsigned int, 3> distanceAverage4;
      EncoderSensor leftEncoder;
      EncoderSensor rightEncoder;
      Adafruit_AMG88xx IR;
      // Line below might need to start with **float instead of float**
      // Unnecessary since we're doing processing in one function
      // float** pixels2d = create2dArray[AMG88xx_PIXEL_ARRAY_SIZE/8][AMG88xx_PIXEL_ARRAY_SIZE/8]
      float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
      long endTime;
      long endStateTime;
      long oldLeftPosition = 0;
      long oldRightPosition = 0;
      float leftVelocity;
      float rightVelocity;
      int currentRightSpeed = -60;
      float timeStuck = 0;
      int leftSpeedOut = 70;
      int rightSpeedOut = 70;
      enum state_t { stateStopped, stateMoving, stateTurning, stateReversing };
      state_t state;
  };
};

Jay::Robot robot;

void setup() {
  Serial.begin(115200);
  robot.initialize();
}

void loop() {
  robot.run();
}

