# Balancing Robot
A balancing robot that uses a PID control loop with accelerometer feedback to drive stepper motors.

## Mark 1

### Bill of Materials

#### Purchased Parts
- 2 x NEMA 17 Stepper
- 1 x Arduino Uno  
- 2 x Adafruit TB6612 1.2A DC/Stepper Motor Driver Breakout Board
- 1 x Adafruit Triple-Axis Accelerometer - ±2/4/8g @ 14-bit - MMA8451
- 1 x Protoboard
- 1 x 3S 11.1 V 1300 mAh LiPo Battery with XT-60 Connector
- 1 x XT-60 Male Connector
- Hookup Wire
- 3M Dual Lock to hold battery

#### 3D Printed Parts
- 1 x Chassis
- 1 x Circuit Mount
- 2 x Wheels

### Lessons Learned
#### Robot Center-of-Gravity
The center of gravity of the robot is too low. It can be balanced when the motors are unpowered. The robot should have been more top-heavy so that steady-state balance is much more difficult.

#### Reliance on Accelerometer
The robot relies on an accelerometer as its sole instrument for which to measure tilt angle. There are two major issues with this.

1. The stepper motors vibrate violently enough to rattle the accelerometer. When this happens, the accelerometer sees random accelerations greater than 1g. This causes angle measurements to become unreliable. However, if a low-pass filter is applied to the measurement data, this problem seems to go away. When determining a cutoff frequency for the filter, there is a trade-off between vibration noise and the disturbance response of the robot. If the cutoff frequency is too low, the robot will not right it self quickly enough when it is bumped.

2. The angle measurement is not reliable when the robot is under a translational acceleration. If the robot accelerates forward while it is not tilted, the control loop will think its tilted and try to compensate.

A future robot should make use of a gyroscope in addition to the accelerometer. A gyroscope measures rate of change of angle, so it must be integrated over time. This results in angle measurements with more instantaneous accuracy, however the gyroscope will drift over time and must be corrected by the accelerometer.

#### Traction Issues
The robot slipped on smooth surfaces. It seemed to work best on carpet. Future designs should have wheels that provide much better traction.

#### Step Angle
The stepper motors moved 1.8° per step. This resulted in very discrete movements from the robot. The wheels could be seen moving like the second-hand on a quartz clock. A future design might make use of a microstepping driver to make wheel movement more continuous.

## Mark 2

### Improvements Over Mark 1

#### Taller Robot
The robot is now taller and more top heavy. The old robot could sometimes balance by itself with no motor input. This version should be much less stable and require more active control input.

#### Sensors
In addition to an accelerometer, this version uses a gyroscope. The gyroscope provides angular rate data which is integrated over time to provide positional feedback to the control loop.

The sensor has been moved to be on the same axis of rotation as the wheels. I noticed that with the previous version the robot behaved differently when tilted to one side than the other. This is likely because the sensor was not mounted above/below the center of mass. I expect the sensor to perform best when mounted both under the center of mass and on the axis of rotation.

The sensor is now has some vibration damping. This should improve the accelerometer noise that was a problem on the previous robot. I opted to use double-sided foam because it was easy. I may go down the rabbit hole and experiment with other isolation/damping methods in the future.

#### Filtering
In order to combine the accelerometer data and the gyroscope data, this robot makes use of a Kalman filter. This is a statistical filter that models the dynamics of the robot to predict its state. It then uses the measurement data to correct the prediction. By taking into account the noise of the system and the measurements, it provides a very reliable prediction of the tilt angle of the robot.
