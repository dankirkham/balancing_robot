#include <Adafruit_MotorShield.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <math.h>

const double P = 13;
const double I = 4;
const double D = 40;

const double FILTER_CUTOFF_HERTZ = 1;
const double FILTER_RC = 1 / (2 * M_PI * FILTER_CUTOFF_HERTZ);
const double SET_POINT = 0;
const int STEPS_PER_REVOLUTION = 200;
const double MICROSECONDS_PER_SECOND = 1000000.0;
const double SECONDS_PER_MINUTE = 60.0;
const double MAX_PV = 400.0; // in degrees per seconds
const double DIAGNOSTICS_HERTZ = 5.0;
const double DPS_PER_RPM = 6;
const double GYRO_GAIN = 24;
const double MOVE_THRESHOLD = 5;
const double INTEGRAL_MAX = 10;

Adafruit_LSM9DS1 imu;

Adafruit_MotorShield afms = Adafruit_MotorShield();
Adafruit_StepperMotor *left_stepper = afms.getStepper(STEPS_PER_REVOLUTION, 1);
Adafruit_StepperMotor *right_stepper = afms.getStepper(STEPS_PER_REVOLUTION, 2);

double integral = 0;
double last_error = 0;
double last_accelerometer_angle = 0;
double gyro_accumulator = 0;
long last_time;
long last_diagnostics_time;
double steps = 0;
int runs = 0;

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

  left_stepper->setSpeed(120);
  right_stepper->setSpeed(120);

  // TODO: Send configuration to host

  last_time = micros();

  afms.begin();

  // Change the i2c clock to 400KHz, this speeds up motor control
  TWBR = ((F_CPU /400000l) - 16) / 2; 
}

void loop() {
  // Find time delta
  long current_time = micros();
  double delta = double(current_time - last_time) / MICROSECONDS_PER_SECOND;
  last_time = current_time;
  runs += 1;

  // Find Error
  sensors_event_t accel, mag, gyro, temp;
  imu.getEvent(&accel, &mag, &gyro, &temp);

  // Accelerometer Angle
  double x = accel.acceleration.x;
  double z = accel.acceleration.z;
  // Simple infinite impulse response filter
  double alpha = delta / (FILTER_RC + delta);
  double raw_accelerometer_angle = SET_POINT - (atan2(-x, -z) * 180 / M_PI);
  double accelerometer_angle = (alpha * raw_accelerometer_angle) + (1 - alpha) * last_accelerometer_angle;
  last_accelerometer_angle = raw_accelerometer_angle;

  // Gyroscope Angle
  gyro_accumulator -= gyro.gyro.y * delta * GYRO_GAIN;
  double error = gyro_accumulator * 0.9 + accelerometer_angle * 0.1;
  gyro_accumulator = error;
//  double error = gyro_accumulator;

  // Find integral and derivative
  integral = integral + (error * delta);
  double derivative = error - last_error;
  // Integral reset
//  if (
//    error < 0 && last_error > 0 ||
//    error > 0 && last_error < 0
//  ) {
//    integral = 0;
//  }
  // Integral cap
//  if (integral > INTEGRAL_MAX) {
//    integral = INTEGRAL_MAX;
//  } else if (integral < -INTEGRAL_MAX) {
//    integral = -INTEGRAL_MAX;
//  }
  last_error = error;

  // Calculate Process Value with gains. Process Value is velocity!
  double p = P * error;
  double i = I * integral;
  double d = D * derivative;
  double process_value = p + i + d;
  // Cap PV
  if (process_value > MAX_PV) {
    process_value = MAX_PV;
  } else if (process_value < -MAX_PV) {
    process_value = -MAX_PV;
  }

  // Find out how many steps need to be taken for this time slice
  steps = steps + (process_value * STEPS_PER_REVOLUTION / SECONDS_PER_MINUTE * delta);

  // Seperate fractional from integral part.
  // Fractional part is rolled over to next iteration.
  double steps_to_take_right_now;
  steps = modf(steps, &steps_to_take_right_now);

  // Diagnostics
  if ((double(current_time - last_diagnostics_time) / MICROSECONDS_PER_SECOND) > (1 / DIAGNOSTICS_HERTZ)) {
    last_diagnostics_time = current_time;
//    Serial.print(gyro_accumulator);
//    Serial.print(' ');
//    Serial.print(accelerometer_angle);
//    Serial.print(' ');
    Serial.print(p);
    Serial.print(' ');
    Serial.print(i);
    Serial.print(' ');
    Serial.print(d);
//    Serial.print(' ');
//    Serial.print(steps);
//    Serial.print(' ');
//    Serial.print(steps_to_take_right_now);
    Serial.print(' ');
    Serial.print(process_value);
    Serial.print(' ');
    Serial.print(runs);
    Serial.print('\n');
    runs = 0;
  }

//   Move the motors
  while (steps_to_take_right_now != 0) {
    if (steps_to_take_right_now >= 32) {
      left_stepper->onestep(BACKWARD, DOUBLE);
      right_stepper->onestep(FORWARD, DOUBLE);
      steps_to_take_right_now -= 32;
    } else if (steps_to_take_right_now >= 16) {
      left_stepper->onestep(BACKWARD, SINGLE);
      right_stepper->onestep(FORWARD, SINGLE);
      steps_to_take_right_now -= 16;
    } else if (steps_to_take_right_now >= 8) {
      left_stepper->onestep(BACKWARD, INTERLEAVE);
      right_stepper->onestep(FORWARD, INTERLEAVE);
      steps_to_take_right_now -= 8;
    } else if (steps_to_take_right_now >= 1) {
      left_stepper->onestep(BACKWARD, MICROSTEP);
      right_stepper->onestep(FORWARD, MICROSTEP);
      steps_to_take_right_now -= 1;
    } else if (steps_to_take_right_now <= -32) {
      left_stepper->onestep(FORWARD, DOUBLE);
      right_stepper->onestep(BACKWARD, DOUBLE);
      steps_to_take_right_now += 32;
    } else if (steps_to_take_right_now <= -16) {
      left_stepper->onestep(FORWARD, SINGLE);
      right_stepper->onestep(BACKWARD, SINGLE);
      steps_to_take_right_now += 16;
    } else if (steps_to_take_right_now <= -8) {
      left_stepper->onestep(FORWARD, INTERLEAVE);
      right_stepper->onestep(BACKWARD, INTERLEAVE);
      steps_to_take_right_now += 8;
    } else if (steps_to_take_right_now <= -1) {
      left_stepper->onestep(FORWARD, MICROSTEP);
      right_stepper->onestep(BACKWARD, MICROSTEP);
      steps_to_take_right_now += 1;
    }
  }
}
