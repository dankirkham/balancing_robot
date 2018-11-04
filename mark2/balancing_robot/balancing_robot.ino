#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Stepper.h>
#include <Wire.h>
#include <math.h>

const double P = 2;
const double I = 0;
const double D = 0;

const double FILTER_CUTOFF_HERTZ = 15;
const double FILTER_RC = 1 / (2 * M_PI * FILTER_CUTOFF_HERTZ);
const double SET_POINT = 0;
const int STEPS_PER_REVOLUTION = 200;
const double MICROSECONDS_PER_SECOND = 1000000.0;
const double SECONDS_PER_MINUTE = 60.0;
const double MAX_PV = 60.0; // in degrees per seconds
const double DIAGNOSTICS_HERTZ = 5.0;
const double DPS_PER_RPM = 6;

Adafruit_LSM9DS1 imu;
Stepper left_stepper(STEPS_PER_REVOLUTION, 4, 5, 6, 7);
Stepper right_stepper(STEPS_PER_REVOLUTION, 8, 9, 10, 11);

double integral = 0;
double last_error = 0;
long last_time;
long last_diagnostics_time;
double steps = 0;

void setup() {
  Serial.begin(115200);

  imu = Adafruit_LSM9DS1();

  Serial.println("Starting IMU");
  if (!imu.begin()) {
    Serial.println("IMU could not be started.");
    while(1);
  }
  Serial.println("IMU Started");

  imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
  imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  left_stepper.setSpeed(100);
  right_stepper.setSpeed(100);

  // TODO: Send configuration to host

  last_time = micros();
}

void loop() {
  // Find time delta
  long current_time = micros();
  double delta = double(current_time - last_time) / MICROSECONDS_PER_SECOND;
  last_time = current_time;

  // Find Error
  sensors_event_t accel, mag, gyro, temp;
  imu.getEvent(&accel, &mag, &gyro, &temp);
  double x = accel.acceleration.x;
  double z = accel.acceleration.z;
  // Simple infinite impulse response filter
  double alpha = delta / (FILTER_RC + delta);
  double raw_error = SET_POINT - (atan2(-x, -z) * 180 / M_PI);
  double error = (alpha * raw_error) + (1 - alpha) * last_error;

  // Find integral and derivative
  integral = integral + (error * delta);
  double derivative = error - last_error;
  last_error = error;

  // Calculate Process Value with gains. Process Value is velocity!
  double process_value = (P * error) + (I * integral) + (D * derivative);
  // Cap PV at 60
  if (process_value > MAX_PV) {
    process_value = MAX_PV;
  } else if (process_value < -MAX_PV) {
    process_value = -MAX_PV;
  }

//  // Set speed of motors
//  if (int(process_value) > 0) {
//    left_stepper.setSpeed(fabs(int(process_value)));
//    right_stepper.setSpeed(fabs(int(process_value)));
//  }

  // Find out how many steps need to be taken for this time slice
  steps = steps + (process_value * STEPS_PER_REVOLUTION / SECONDS_PER_MINUTE * delta);

  // Seperate fractional from integral part.
  // Fractional part is rolled over to next iteration.
  double steps_to_take_right_now;
  steps = modf(steps, &steps_to_take_right_now);

  // Diagnostics
  if ((double(current_time - last_diagnostics_time) / MICROSECONDS_PER_SECOND) > (1 / DIAGNOSTICS_HERTZ)) {
    last_diagnostics_time = current_time;
    Serial.print(raw_error);
    Serial.print(' ');
    Serial.print(error);
    Serial.print(' ');
    Serial.print(delta);
    Serial.print(' ');
    Serial.print(integral);
    Serial.print(' ');
    Serial.print(derivative);
    Serial.print(' ');
    Serial.print(steps);
    Serial.print(' ');
    Serial.print(steps_to_take_right_now);
    Serial.print(' ');
    Serial.println(process_value);
  }

  // Move the motors
  while (steps_to_take_right_now != 0) {
    if (steps_to_take_right_now > 0) {
      left_stepper.step(-1);
      right_stepper.step(1);
      steps_to_take_right_now--;
    } else {
      left_stepper.step(1);
      right_stepper.step(-1);
      steps_to_take_right_now++;
    }
  }
}
