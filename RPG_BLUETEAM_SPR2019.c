//*** BLUE TEAM - ROCKET POWERED GLIDER PROJECT ***
// WRITTEN BY MATT AVNY
// CONTRIBUTORS: LUCAS LIDNER, ADAM POSSET, BREANNE STICHLER, BRAD TONER
// THANK YOU: JORDAN STREET AND JAKE COVINGTON, and UCF

#include <CurieIMU.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <math.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

/* Servo Setup */
Servo elevator;                                                                          // Define our servos
Servo rudder;

DFRobot_QMC5883 compass;                                                                 // Turn on compass and pressure/temp sensors
Adafruit_BMP3XX bmp;

#define dt 0.005                                                                         // Time step of 200 Hz for sensors
#define arrval 10
#define alpha 0.98

int orPinNum = 12;                                                                        // Digital pin number for the override switch (from tx/rx)
int elPinNum = 8;
int rudPinNum = 10;                                                                      // Digital pin number for the elevator and rudder commands (from tx/rx)

bool manualOverride = false;                                                             // Global override flag

static int orPwmThresh = 1500;                                                           // Override time threshold
static int interval = 1500;                                                              // LED interval
volatile int orStartTime = 0;
volatile int orPwmValue = 0;

int ledState = LOW;
unsigned long ledTime = 0;

int ch1, ch2;                                                                            // Servo manual control readers
int counter1 = 0;

unsigned long timer1;                                                                    // This is the 'time since rocket fired' timer
unsigned long timer2;                                                                    // Stall timer
unsigned long timer3;                                                                    // Magnetometer warmup timer

//float initialAltitude;

float PsiSum[arrval];
float PsiFinal;
int j = 0;
float TargetHeading;

static bool timeTrigger1 = false;                                                        // Triggers for magnetometer
static bool timeTrigger2 = false;

/* F U N C T I O N S */

float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 4) / 32768.0;                                                        // Converts raw data to g
  return a;
}

float convertRawGyro(int gRaw) {
  float g = (gRaw * 250) / 32768.0;                                                       // Converts raw data to deg/sec
  return g;
}

void orRising() {
  orStartTime = micros();
  attachInterrupt(digitalPinToInterrupt(orPinNum), orFalling, FALLING);
}

void orFalling() {
  orPwmValue = micros() - orStartTime;

  if (orPwmValue > orPwmThresh) {
    if (!manualOverride) {
      manualOverride = true;
    }
  }
  attachInterrupt(digitalPinToInterrupt(orPinNum), orRising, RISING);
}

/* Define PID Variables */

// Longitudinal Rate
long LongRateKp = -0.02192;
long LongRateKi = -1.04329;
// Longitudinal Angle
long LongAngleKp = 3.5077;
long LongAngleKi = 0.1751;
// Lateral Angle 1
long LatAngle1Kp = 1.05511;
long LatAngle1Ki = 0.01351;
// Lateral Angle 2
long LatAngle2Kp = 3.19193;                                                             // -0.19193, 3.19193(worked kinda)
long LatAngle2Ki = 2.00379;                                                              // -0.00379, 2.00379(worked kinda)

void setup() {
  
                                                                                        // initialize device
  CurieIMU.begin();
  compass.begin();
  bmp.begin();

  initialAltitude = bmp.readAltitude(bmp.readPressure());                            // Gets original altitude of glider on the launch rail

  /* Configuration of sensors */
  CurieIMU.setAccelerometerRange(4);     // +/- 4g
  CurieIMU.setGyroRange(250);            // +/- 250 deg/sec
  CurieIMU.setGyroRate(200);             // Hz
  CurieIMU.setAccelerometerRate(200);    // Hz

  CurieIMU.setGyroOffset(X_AXIS, -0.61);    // Set gyro offsets. This was calibrated beforehand
  CurieIMU.setGyroOffset(Y_AXIS, 0.67);
  CurieIMU.setGyroOffset(Z_AXIS, -0.37);

  CurieIMU.setAccelerometerOffset(X_AXIS, -23.40);                                // Set accelerometer offsets. Also calibrated beforehand.
  CurieIMU.setAccelerometerOffset(Y_AXIS, 39.00);
  CurieIMU.setAccelerometerOffset(Z_AXIS, -19.50);

  compass.setRange(QMC5883_RANGE_2GA);
  compass.setMeasurementMode(QMC5883_CONTINOUS);
  compass.setDataRate(QMC5883_DATARATE_50HZ);
  compass.setSamples(QMC5883_SAMPLES_8);

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);                                  // Optimal rates
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); 

  pinMode(LED_BUILTIN, OUTPUT);                                                     // Initialize LED
  pinMode(orPinNum, INPUT);                                                  // Sets the commands from rec as inputs
  pinMode(elPinNum, INPUT);
  pinMode(rudPinNum, INPUT);

  /* Assigning Servo Pins to PWM on CurieNano */
  elevator.attach(6);
  rudder.attach(3);

  // Sets rudder and elevator to mid point
  rudder.writeMicroseconds(1430);   // servo 1
  elevator.writeMicroseconds(1585); //servos 2 and 3

  attachInterrupt(digitalPinToInterrupt(orPinNum), orRising, RISING);

}

void loop() {

  if (manualOverride == true) {     // Will ALWAYS operate while manualOverride = true
    ch1 = pulseIn(elPinNum, HIGH);  // Measures pulses for the rudder and elevator
    ch2 = pulseIn(rudPinNum, HIGH);

    int ch1Fixed = ch1;
    ch1Fixed = map(ch1Fixed, 1023, 1920, 1100, 2000); // Shifts elevator midpoint value up so servo is centered(1585ms)

    elevator.writeMicroseconds(ch1Fixed);
    rudder.writeMicroseconds(ch2);

    digitalWrite(LED_BUILTIN, HIGH); // LED always on for manual override
     //Serial.println("Manual Override Achieved");   //DEBUG
  }

  else {                                                // AUTOPILOT

    float altitude = bmp.readAltitude(bmp.readPressure());

    int axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    float ax, ay, az, gx, gy, gz;
    int i;

    unsigned long time = millis();

    noInterrupts();
    CurieIMU.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);
    interrupts();

    /* Convert Values */
    ax = convertRawAcceleration(axRaw);
    ay = convertRawAcceleration(ayRaw);
    az = convertRawAcceleration(azRaw);
    gx = convertRawGyro(gxRaw);  // Equivalent to 'p' or rotation rate about x-axis
    gy = convertRawGyro(gyRaw);  // Equivalent to 'q' or rotation rate about y-axis
    gz = convertRawGyro(gzRaw);  // Equivalent to 'r' or rotation rate about z-axis

    /* Complementary Filter Start */

    float Phi, Theta, Psi;                                              // Roll, Pitch, Yaw initials

    Phi = atan2(-ax, az) * 180 / PI;                                    // Calculate ( was in radians, converted ! )
    Theta = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

    /* Compass and Psi stuff */
    Vector mag = compass.readNormalize();

    float heading = atan2(mag.YAxis, mag.XAxis);                      // Not tilt corrected

    if (heading < 0 ) {                                                   // Correcting for heading <0 deg and >360 deg
      heading += 2 * PI;
    }

    if (heading > 2 * PI) {     // SPIN BEFORE LAUNCH
      heading -= 2 * PI;
    }

    float declinationAngle = (-4 + (37.0 / 60.0)) / (180 / PI);          //Roughly around UCF http://magnetic-declination.com/
    heading += declinationAngle;

    float headingDegrees = heading * 180 / PI;

    if (j == 10) {
      j = 0;
    }

    PsiSum[j] = headingDegrees;
    j++;

    for (int m = 0; m < arrval; m++) {
      PsiFinal += PsiSum[m];
    }

    PsiFinal = PsiFinal / arrval;

    Psi = PsiFinal;                                        // Initial set to magnetometer reading

    /* Convert Gyro Data from Body Frame to Earth Frame */

    double junk = cos(Theta);

    float PhiDotMatrix[3] = {gx, (sin(Phi) * tan(Theta)) * gy, (cos(Phi) * tan(Theta)) * gz}; // Euler Conversion gx = p, gy = q, gz = r
    float ThetaDotMatrix[3] = {0, (cos(Phi)) * gy, (-sin(Phi)) * gz};
    float PsiDotMatrix[3] = {0, (sin(Phi) * pow(junk, -1)) * gy, (cos(Phi) * pow(junk, -1)) * gz};

    float PhiDot = 0, ThetaDot = 0, PsiDot = 0;

    for (i = 0; i < 3; i++) {                                            // Sums matrix after multiplication
      PhiDot += PhiDotMatrix[i];
      ThetaDot += ThetaDotMatrix[i];
      PsiDot += PsiDotMatrix[i];
    }

    /* Sensor Fusion */                                                 // 98%:2% rule

    Theta = alpha * (Theta + (ThetaDot * dt)) + (1 - alpha) * (atan2(ay, az));
    Psi =  alpha * (Psi + (PsiDot * dt)) + (1 - alpha) * (atan2(ax, ay));

    // L A U N C H  M O D E //

  if (counter1 == 0) {
    elevator.writeMicroseconds(1585);
    rudder.writeMicroseconds(1430);
  }


    if (ay > 3) {                   // Start timer at launch, under these conditions
      timer1 = time;
    }

    if (counter1 == 0 && timer1 > 1) {
      elevator.writeMicroseconds(1585);                       // Launch trim for elevators
      rudder.writeMicroseconds(1430);   // off abit          // Launch trim for rudder (zeroed out)

      if ((time - timer1 > 2500) && (az > 0.7)) {    // Condition for entering glide mode, and never going back to launch mode
        counter1++;
      }
    }

    if (timeTrigger1 == false) {   // Starts a timer for 12 seconds
      timeTrigger1 = true;
      timer3 = micros();
    }

    if ((timeTrigger2 == false) && (micros() - timer3 > 12000)) {  // Heading calculated after 12 seconds
      timeTrigger2 = true;
      TargetHeading = PsiFinal;     // Initial heading that will try to be maintained
    }

    // G L I D E   M O D E //
    if (counter1 == 1) {

      if (Theta < -9.0 || Theta > - 7.0) {  // Target theta of -8 degrees with +/- 1.0 deg tolerance

        double LongAngleInput = -8 - Theta;  // Input loop 1
        double LongAngleOutput;     // Output loop 1 -thetadot
        double LongAngleSetpoint = -8;        // Setpoint loop 1

        PID longAngle(&LongAngleInput, &LongAngleOutput, &LongAngleSetpoint, LongAngleKp, LongAngleKi, 0, DIRECT);    // Outer PI loop
        longAngle.SetMode(AUTOMATIC);
        longAngle.SetOutputLimits(-120, 120);
        longAngle.Compute();

        double LongRateInput = LongAngleOutput - ThetaDot;
        double LongRateOutput;        // Elevator deflection in degrees
        double LongRateSetpoint = 0;

        PID longRate(&LongRateInput, &LongRateOutput, &LongRateSetpoint, LongRateKp, LongRateKi, 0, REVERSE);  // Inner PI Loop
        longRate.SetMode(AUTOMATIC);
        longRate.SetOutputLimits(-250, 250); // fill this out to not overload servos
        longRate.Compute();
        
        float elevatorCommand = LongRateOutput;
        elevatorCommand = map(elevatorCommand, -250, 250, 1150, 1930);
        elevator.writeMicroseconds(elevatorCommand);  // Servo instruction

      }

      if (Psi > (TargetHeading + 3.5) || Psi < (TargetHeading - 3.5)) { // Target psi, 7 degree deadzone

        double LatAngle1Input = (0 - Psi);
        double LatAngle1Output;       // psidot
        double LatAngle1Setpoint = TargetHeading;

        PID latAngle1(&LatAngle1Input, &LatAngle1Output, &LatAngle1Setpoint, LatAngle1Kp, LatAngle1Ki, 0, DIRECT);
        latAngle1.SetMode(AUTOMATIC);
        latAngle1.SetOutputLimits(-120, 120);
        latAngle1.Compute();

        double LatAngle2Input = LatAngle1Output - Psi;
        double LatAngle2Output;
        double LatAngle2Setpoint = 0;

        PID latAngle2(&LatAngle2Input, &LatAngle2Output, &LatAngle2Setpoint, LatAngle2Kp, LatAngle2Ki, 0, REVERSE);
        latAngle2.SetMode(AUTOMATIC);
        latAngle2.SetOutputLimits(-250, 250);
        latAngle2.Compute();

        float rudderCommand = LatAngle2Output;
        rudderCommand = map(rudderCommand, -250, 250, 1130, 1730);
        rudder.writeMicroseconds(rudderCommand);
      }

      else {
        rudder.writeMicroseconds(1430); // off a bit
      }
    }

    // L A N D I N G  M O D E //

    if ((altitude <= initialAltitude + 2) && (counter1 == 1)) {  // Once the glider is about 2m above from where it started, it'll go into this mode
      counter1++;                           // Prevents it from going back to the previous modes
      timer2 = time;
    }

    if (timer2 + 5000 >= time) {
      rudder.writeMicroseconds(1430);
      elevator.writeMicroseconds(1950); // Stall aircraft to land softly with max elevator deflection
    } 

    /* LED FUNCTION */

    if (time - ledTime >= interval) {                // Shine LED intervals of 1.5sec for AUTOPILOT ON
      ledTime = time;

      if (ledState == LOW) {
        ledState = HIGH;
      }
      else {
        ledState = LOW;
      }
    }
    digitalWrite(LED_BUILTIN, ledState);
  }
}