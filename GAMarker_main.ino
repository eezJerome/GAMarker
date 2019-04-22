/*

  Implementation of Gyroscope-Accelerometer to Track Motion in a Projector-Based Display for Low-Cost Intelligent Teaching Systems
  A paper by Jerome Ariola
  The goals of this project are defined here: https://docs.google.com/document/d/1yRyuVYNtGEeND7_DPktd76tyu4C6lmXf-QXMJcSXFFs/edit?usp=sharing


  Just some notes:
  Gravity by the MPU6050 is read by the 8bit registers as 16384 (2^14); 1g = 9.81 m/s/s

                (**REFER TO DATASHEET IF CONFUSED**)
  The value of measured acceleration on an axis is therefore defined by this constant:
      rawvalue / 1671.133
  (this number is derived from 16384 / 9.81)

  The value of measured (gyro data) 1 deg/s is 250;
    rawvalue / 0.6944
  (this number is derived from 250 / 360)


  Accleromter data is measured in m/s²
  Gyroscope data is measure in degrees/s

  Double integration of acceleration should give displacement but this is a difficult thing to do....

  The maths for mapping out a surface for a given display resolution should be easy...but still need to figure out something

  How this will be done: the gyroscope serves as an accel/gyro smartpen. The pen has the sensor integrated into the main body
  and its motion across the X and Y axes is captured. This is mapped onto the screen of the user's computer; when projected the mouse cursor moves
  with the motion of the pen. Since the surface being used as the projector screen is essentially from the computer, mapping of a coordinate grid to
  acquire the pen's position should be easy...right?

  Acceleration on the X and Y axes can be used to derive displacement; this value can be used to draw a line, hence the purpose of the pen :)
    --for example, if the velocityy is read as 0.1 m/s and the pen was "on" (i.e. the user has the button that defines "write" is pressed) for 0.25s
      then the displacement on that axes is 0.1m / 0.25s = 0.4m or 40cm. Easier said than done

  The Z axis isn't so necessary, and even still it won't be so accurate with this sensor anyway, however consider implementation if need be.

  Take from forum that I asked from:
  https://forum.arduino.cc/index.php?topic=605186.0

  1.  The MPU6050 sensor does the following jobs:
  (1)  Gyro measurements (change in angular velocity = change in rotational angle/unit time).
  (2)  Acceleration measurements.
  (3)  Temperature measurement.

  2.  MPU6050 sensor is an I2C Bus compatible sensor. Therefore, you must include the Wire.h Library in your Arduino IDE.

  3.  You can operate MPU6050 sensor in two ways:
  (1) Use the functions/methods of the Wire.h Library to prepare codes to --
  (a) Perform read/write operations on the registers of the sensor data;
  (b) Compute information based on the values of the above registers.
  (c)  Simple Example -- Temperature Measurement (tested in UNO

  (2) Use the functions/methods of the MPU6050.h Library (it calls/uses Wire.h Library) to --
  (a) Perform read/write operations on the registers of the sensor data;
  (b) Compute information based on the values of the above registers.
  (c)  Simple Example -- Temperature Measurement (tested in UNO)

  Generic Program Sequence:
  INITIALIZATION
  1. initialize all functions and check if sensor and RX/TX are live
     --> the sensor, with correct offsets should do just fine; no additional calibration needed; otherwise, consider this of interest
  2. ask for calibration by some signal. This will be done by choosing a starting point, which will be used to reference the position of the sensor relative
     to it. This is what makes the pen so unique and special -- it (should) uses the double integral of * acceleration * to find * displacement *
     --> the movement of the sensor on an axis contains a * velocity * ; maybe this can also be used to find displacement of the body.
       ===> noise filtering is a must here! Consistency of the velocities and all measured data is important for industry-level function
  MAIN LOOP
  1. if "write" button is pressed
     --> get velocity/acceleration/measured force on the axes that require measurement
     --> calculate displacement
     --> transmit to RX how far the cursor was to the reference point when capture was taken
     --> draw function to display what was drawn
       ===> essentially the cursor is tracked relative to the reference starting point; from there the mouse on the pc is just doing on/off
     --> end
  2. check if other data is present, if button is still being pressed, etc
  3. increment timer
  4. go to start of loop function

  Functions and their explanations:
  getaccel(x, y);
    -- Gets measured acceleration when called
    -- Converts the raw 8bit
*/

/*
  CHANGELOG
  2019.03.22 -- created code
  2019.03.25 -- trying to figure out what constants are necessary
            -- took note of this:
                 AngleX = atan(X/(√X² + Z²))
                 AngleY = atan(X/(√Y² + Z²))
            -- simple inclined plane vector math can be used here too..
  2019.03.26 -- added some notes on how the code should be arranged.
  2019.03.27 -- added offsets from a previous test
                   mpu.setXGyroOffset(48); //this was at 220
                   mpu.setYGyroOffset(-40); //76
                   mpu.setZGyroOffset(-17); //-85
                   //these are ones I put
                   mpu.setXAccelOffset(-4318);
                   mpu.setYAccelOffset(-3424);
  2019.03.29 -- added conversion function from raw to m/s²
  2019.04.02 -- changed variables to be read to float type instead of int
             -- actually MADE THE READING OF VALUES WORK so yay :)
             -- At this stage of the project only the gyro is being read;
                --> pro tip: get accel values next :)
                    -->> use DOUBLE INTEGRATION to find displacement in centimeters
  2019.04.04 -- adding gyroscope function to get acceleration of moving object
             -- added accel Z offset value
             -- Today I also copied and played around with Excel/Sheets and the values. Filtering is needed
             -- still trying to figure out the gyro constant to show o/s
             -- BEEN TRYING TO CONFIGURE THE GYRO BUT IT SEEMS ALL I NEED IS VELOCITY
             -- some notes to remember for next edit:
                --> implement the single acceleration of the velocity or the double integration of acceleration (depends which one is
                    being output by the accelerometer). Then do mouse.move().
  2019.04.08 -- attempting to add integration of data using 1/2 bh dx
             -- added commas to data because data splitting can be done on Sheets
             -- found out that with a single stream of data (i.e. one axis) there are about 410-420 data points in a second
                data stream with 3 accelerometer axes gives about 250 readings in one second
             -- ran Get_Offset_Values_MPU_Calibration and averaged the offsets in 5 trials
                --> new offsets
                    (reading as gx, gy, gz, ax, ay, az)
                    -4172.5  -3436  1473.5  50  -39  -18.5
             -- during testing the sensor was TILTED on the breadboard....retesting...
  2019.04.12 -- added terminal clear in loop()
  2019.04.13 -- added low-pass filter via DLPF
             -- really considering trapezoidal area to integrate acceleration
  2019.04.15 -- updated offsets because i didn't make sure the sensor was leveled the last time
                new offsets:
                *******NOTE THIS IS ONLY FOR MY CURRENT BREADBOARD SETUP
                (reading as gx, gy, gz, ax, ay, az)
                -4280  -3452 1475  51  -40 -19
             -- goals for today:
                --> write some code to find rate of change per axis (deltaX, deltaY, deltaZ)
  2019.04.22 -- learned how to integrate (thanks organic tutor <3 )
             -- trying to implement the intergrate() algorithm
*/
#include <I2Cdev.h>
#include <MPU6050.h>

//some constants that I had no idea what to put so i copied the constants from electronoobs site

// declaring sensor module
MPU6050 mpu;

// defining vars for raw values
const float g = 9.80665;
int16_t
rawaccx, rawaccy, rawaccz,
         rawgyrx, rawgyry, rawgyrz;

// real angle
// "u" is theta here (i.e. angle)
float
uaccx, uaccy,
       ugyrx, ugyry,
       usumx, usumy;

//float
//time, prevtime, currenttime;
//float currenttime = micros()/1000000.0f;
//float prevtime = micros()/1000000.0f;

int i;
float pi = 180 / 3.141592654;

//acceleration conversion factor; raw_value / 1671 = m/s2 value
const int convraw = 1671;

//gyro conversion factor
// gyro might not be so necessary
const int convdeg = 131;   //0.000031;
// this is taken from Joop Brokking video (MPU-6050 6dof IMU tutorial for auto-leveling quadcopters with Arduino source code at 7:48)
// i.e. 1 / 250 / 131 (in my sensor it is 131 because of the FS_SEL=0 in the datasheet)


//real values
float
realaccx, realaccy, realaccz;

float
realdegx, realdegy, realdegz;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  mpu.initialize();

  //  51  -40 -19 -4280  -3452 1475
  mpu.setXGyroOffset(51); //this was at 220
  mpu.setYGyroOffset(40); //76
  mpu.setZGyroOffset(-19); //-85
  //these are ones I put
  mpu.setXAccelOffset(-4280);
  mpu.setYAccelOffset(-3452);
  mpu.setZAccelOffset(1475);

  mpu.setDLPFMode(MPU6050_DLPF_BW_42); // 42hz digital low pass filter

  //  Serial.println("This program converts the raw MPU6050 values in terms of m/s/s/");
  //  Serial.println("3");
  //  delay(1000);
  //  Serial.println("2");
  //  delay(1000);
  //  Serial.println("1");
  //  delay(1000);
}

void loop() {
  //  while(millis() <= 1001){ // this is to test individual axes for data collecting/analysis; comment as you wish
  mpu.getMotion6(&rawaccx, &rawaccy, &rawaccz, &rawgyrx, &rawgyry, &rawgyrz);
  getaccel(rawaccx, rawaccy, rawaccz);  // just to print this function
  getgyr(rawgyrx, rawgyry, rawgyrz);
  // getangle(); // still don't know what to put here....I'll need to look into datasheet or something
  // gettime();

  // the data output can then be pasted onto a spreadsheet. Split data for individual axix; use space as the separator
  //  currenttime = millis();

  //  Serial.print(realaccx); Serial.print(" \t");
  //  Serial.print(realaccy); Serial.print("      \t");
  //  Serial.print(realaccz); Serial.print("      \t");

    accel_getDelta();
  //Serial.print((currenttime - 1000) / 1000); Serial.print("         \t");
  //  Serial.println(" ");

  //  } // comment this if no while loop
  //  prevtime = currenttime; // this is for finding the time difference
}



//this is pretty much the whole pseudocode because I don't really know how to implement the libraries
// converts raw accelerometer data to m/s/s
float getaccel(float x, float y, float z) {
  realaccx = x / convraw;
  realaccy = y / convraw;
  realaccz = z / convraw;
}

float getgyr(float x, float y, float z) {
  realdegx = x / convdeg;
  realdegy = y / convdeg;
  realdegz = z / convdeg;
}

void getangle() {

}
// AngleY = PrevAngleY + GyroDataY * elapsedTime
// AngleX = PrevAngleX + GyroDataX * elapsedTime

int gettime() {
  //  // im not so keen on making timers so I copied this
  //  timeprev = time;  //the previous time is stored before the actual time read
  //  time = millis();  // actual time read
  //
  //  elapsedtime = (time - timeprev) / 1000;
}

float integrate() {
  /*
     this uses the area of a triangle to basically integrate the area under the curve (i.e. the sensor readings)
     integrate each axis for usable data
     take note of sample time
     algorithm in a nutshell:
        -- take current time
        -- measure 1/2bh dx, where "b" is half of the time (measured in s), h is the y-value, and not sure how dx falls into all of this...
        -- if i can learn how to really integrate, then error can be removed
           **** 1/2 (a+b) h ***
              it would make sense to reason the following
              -- idk i really think now that it won't matter....
    NO, USE TRAPEZOID INSTEAD OF TRIANGLE!
  */


}

// not sure if i'll do it right
// The intent of this function is to get how much acceleration changes
// hence "getDelta"
// time, prevtime, currenttime

int sampletime = 50; 

//float accel_getDelta() {
//  currenttime = micros();
//  float x1 = realaccx;
//  float y1 = realaccy;
//  float z1 = realaccz;
//
//  float deltat = (currenttime - prevtime);
//  if (micros()  > sampletime + currenttime) {
//    float x2 = realaccx - x1;
//    float y2 = realaccy - y1;
//    float z2 = realaccz - z1;   
//    Serial.print(x2); Serial.print("\t  ");
//    Serial.print(y2); Serial.print("\t  ");
//    Serial.print(z2); Serial.print("\t");
//    Serial.println(currenttime); Serial.print("\t");
//    prevtime = currenttime;
//  }
//}
//
//void mouse_test(){
//  Mouse.move(readAxis(0), readAxis(1), 0);
//
//}
//
unsigned long prevtime = 0;

long accel_getDelta(){
  static long x1=0, y1=0, z1=0;
  unsigned long currenttime = micros();//1000000.0f;
  float deltat = (currenttime - prevtime);

  if (deltat > sampletime){
    prevtime = currenttime;
    
    Serial.print(realaccx - x1); Serial.print("\t ");  
    Serial.print(realaccy - y1); Serial.print("\t ");
    Serial.print(realaccz - z1); Serial.print("\t ");
    Serial.println(currenttime); Serial.print("\t ");
    Serial.println(" ");

    x1 = realaccx;
    y1 = realaccy;
    z1 = realaccz;
  }
}
