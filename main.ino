#include <L298N.h>
#include <Encoder.h>
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB

// gyro = degrees/sec
// rotation of the sensor clockwise (correct direction of text is upwards) results in postive gyroZ

// 480 ticks = 1 rev



// RING CALCULATIONS --------------------------------------------------------
// TORQUE                 (ring) = 0.049Nm
// INERTIA                (ring) = 0.0002447217kgm^2
// ANGULAR ACCELERATION   (ring) = 200.227442029 rad/s^2
// RING CALCULATIONS --------------------------------------------------------

MPU9250_DMP imu;
float degrees = 0;

//pin definition
#define EN 9
#define IN1 8
#define IN2 10
#define I_ROCKET 10     // i dont even know what units
#define I_RING 4        // i dont even know what units
#define DPS_MIN 0
#define DPS_MAX 2000
#define MOTOR_MIN 60     // 0, but can it actually spin at 0?
#define MOTOR_MAX 255

int curr_time = 0;
bool calibrated = false;
bool calibration_sequence = true;
float gyroZ_calibration;
int zero_count = 0;

//create a motor instance
L298N motor(EN, IN1, IN2);
Encoder encoder(11,12);

void setup() {

  //used for display information
  SerialPort.begin(115200);

  motor.setSpeed(255); // an integer between 0 and 255

  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  SerialPort.println("test");

  imu.setSensors(INV_XYZ_GYRO); // Enable gyroscope only
  imu.setGyroFSR(DPS_MAX); // Set gyro to 2000 dps

  imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
              DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
              10);                   // Set DMP rate to 10 Hz

//  delay(5000);
//  calibrated = detectCalibration();
}

float av_old  = 0;
float gyroZ;
long oldPosition  = -999;

// function declarations
bool detectCalibration();
float calcCorrection(float& av_old);
float performCorrection(float correction_av);
float getPosition();
float getGyroZ(bool log);


void loop() {
    if (!calibrated)
      calibrated = detectCalibration();
//    
    if (calibrated) {
      if (calibration_sequence) {
        calibration_sequence = false;
      }

      
      if (imu.fifoAvailable() ) {
//      // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
      if (imu.dmpUpdateFifo() == INV_SUCCESS) {

        float curr_gyro_z = getGyroZ(false);
        float rocket_angular_radian = deg_to_rad(abs(curr_gyro_z));
        float motor_duty = neededMotorDuty(rocket_angular_radian);
        motor.setSpeed(motor_duty);
        if (curr_gyro_z < 0) {
          // rotate the motor in the opposite direction, based on the value of the gyroZ
            motor.forward();
            delay(10);
        } else {
            motor.backward();
            delay(10);
        }
        
//        SerialPort.print(calcCorrection(av_old));
//        float correction_av = calcCorrection(av_old);
//        performCorrection(correction_av);
      }
    }
//
//    delay(10);
//
    }
    
}

float neededMotorDuty(float rocket_angular_radian) {
  float constant = 1.0;
  float motor_angular_radian = constant * rocket_angular_radian;

  float motor_angular_revolutions = motor_angular_radian / (2 * PI);

  // calculating motor voltage
  float numerator = log(motor_angular_revolutions) - log(0.327);
  float motor_duty = numerator / (0.01275);

  if (motor_angular_revolutions <= 0.33) {
      motor_duty = 0;
  }

  SerialPort.print(millis());
  SerialPort.print("   Motor duty: ");
  SerialPort.println(motor_duty);

  return motor_duty + 20;

}

float getPosition() {
  long newPosition = encoder.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    unsigned long time = millis();
    SerialPort.print(time);
    SerialPort.print(",");
    SerialPort.println(newPosition);
  }
  
}

// return radians per second
float deg_to_rad(float degree) {
    float numerator = degree*2*PI;
    return numerator / 360;         // radians/sec
}

// reads gyroZ from the sensor and logs it to the serial 
float getGyroZ(bool log) {
  float gyroZ = imu.calcGyro(imu.gz);
  
  if (log) {
    SerialPort.print("Gyro Z: ");
    SerialPort.println(gyroZ);
  }

  return gyroZ;
}

// takes old angular velocity (av_old)
// samples the new angular velocity (av_new)
// performs calculation
// updates old angular velocity
// returns the correction for the control wheel

float calcCorrection(float& av_old)
{  
  float av_new = getGyroZ(true);
  
  //  perform calculation
  float delta_av = av_new - av_old;
  float ring_av = (I_ROCKET / I_RING) * (delta_av);
  av_old = av_new;

  return ring_av;
}

float performCorrection(float correction_av) {
  float motor_av_t = map(abs(correction_av), DPS_MIN, DPS_MAX, MOTOR_MIN, MOTOR_MAX);
  motor.setSpeed(motor_av_t); // set to computed gyroscope angular velocity
  // rotate the motor faster and faster until the speed read by the ... isn't 0
  if (correction_av > 0) {
    // while (motor_av_r < motor_av_t)
    motor.backward();        // continue moving forwards while motor_av_r(eal) != motor_av_t(heoretical)
  } else {
    // while (motor_av_r < motor_av_t)
    motor.forward();       // continue moving backwards while motor_av_r(eal) != motor_av_t(heoretical)
  }
  // verify that the motor has reached that angular velocity
  // control system needs to be implemented here  
}

bool detectCalibration(){
  
  // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    
  while(zero_count < 20) {
    if (imu.fifoAvailable() ) {
      if (imu.dmpUpdateFifo() == INV_SUCCESS) {
        gyroZ_calibration = imu.calcGyro(imu.gz);
        SerialPort.println(gyroZ_calibration);
        SerialPort.println(zero_count);
        if (gyroZ_calibration < 0.3 && gyroZ_calibration > -0.2) {
          zero_count += 1;
        }
      }
    }
  }

  return true;
  
}
