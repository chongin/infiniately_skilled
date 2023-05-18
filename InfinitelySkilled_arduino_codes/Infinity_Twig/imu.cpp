
#include <Arduino.h>
#include <Wire.h>
#include <time.h>
#include "SparkFun_ISM330DHCX.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include "Fusion.h" // absolute lifesaver!
#include "imu.h"

#define IMU_SAMPLE_RATE (100) // lie about sample rate for nicer filtering

static SparkFun_ISM330DHCX myIMU;
static SFE_MMC5983MA myMag;
static FusionEuler euler;
static FusionQuaternion quaternion;
static FusionAhrs ahrs;

// Structs for X,Y,Z data
sfe_ism_data_t accel;
sfe_ism_data_t gyro;
uint32_t magX = 0;
uint32_t magY = 0;
uint32_t magZ = 0;
double normalizedX = 0;
double normalizedY = 0;
double normalizedZ = 0;
double heading = 0;
static clock_t previousTimestamp;

void imu_update(struct twig_struct *twig_data)
{
  const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp

  // Note: We will use interrupt counters for generally event handling + counters
  // so this code will come back along with some twig_struct event variables

  // if (tap_interrupt_counter > 0 && millis() - tap_time_previous > TAP_TIMEOUT) {
  //   twig_data->tap_event_counter++;
  //   twig_data->tap_event = tap_interrupt_counter;
  //   tap_interrupt_counter = 0;
  // }
  myIMU.getAccel(&accel); // units = milli-g's
  myIMU.getGyro(&gyro);   // units = milli-g's
  myMag.getMeasurementXYZ(&magX, &magY, &magZ);

  // Unit Conversions
  // -----------------------------------------------
  // Convert milli-g's into g's
  accel.xData = accel.xData / 1000.0f;
  accel.yData = accel.yData / 1000.0f;
  accel.zData = accel.zData / 1000.0f;

  // Convert milli-degrees-per-second into degrees-per-second
  gyro.xData = gyro.xData / 1000.0f;
  gyro.yData = gyro.yData / 1000.0f;
  gyro.zData = gyro.zData / 1000.0f;

  // The magnetic field values are 18-bit unsigned. The zero (mid) point is 2^17 (131072).
  // Normalize each field to +/- 1.0
  normalizedX = (double)magX - 131072.0;
  normalizedX /= 131072.0;
  normalizedY = (double)magY - 131072.0;
  normalizedY /= 131072.0;
  normalizedZ = (double)magZ - 131072.0;
  normalizedZ /= 131072.0;

  // Magnetic north is oriented with the Y axis
  // Convert the X and Y fields into heading using atan2 (Arc Tangent 2)
  heading = atan2(normalizedX, 0 - normalizedY);

  // atan2 returns a value between +PI and -PI
  // Convert to degrees
  heading /= PI;
  heading *= 180;
  heading += 180;

  // The magnetometer full scale is +/- 8 Gauss
  // Multiply the normalized values by 8 to convert to Gauss
  magX = normalizedX * 8;
  magY = normalizedY * 8;
  magZ = normalizedZ * 8;

  // -----------------------------------------------

  // replace this with actual gyroscope timestamp
  const FusionVector accelerometer = {accel.xData, accel.yData, accel.zData}; // accelerometer data in g
  const FusionVector gyroscope = {gyro.xData, gyro.yData, gyro.zData};        // gyroscope data in degrees/s
  const FusionVector magnetometer = {(float)magX, (float)magY, (float)magZ};  // magnetometer data in arbitrary units - Gauss

  // Apply calibration
  // gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  // accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  // magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

  // Update gyroscope offset correction algorithm
  // gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // Calculate delta time (in seconds) to account for gyroscope sample clock error
  const float deltaTime = (float)(timestamp - previousTimestamp) / (float)CLOCKS_PER_SEC;
  previousTimestamp = timestamp;

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
  quaternion = FusionAhrsGetQuaternion(&ahrs);
  euler = FusionQuaternionToEuler(quaternion);

  twig_data->yaw = constrain(map(euler.angle.yaw + 180, 0, 360, 0, 256), 0, 255);
  twig_data->pitch = constrain(map(euler.angle.pitch + 180, 0, 360, 0, 256), 0, 255);
  twig_data->roll = constrain(map(euler.angle.roll + 180, 0, 360, 0, 256), 0, 255);

  // This data is only really needed for training the MLC/FSM
  // although conceptually we should be able to do the same
  // with the yaw, pitch, roll changes over time?
  twig_data->accel[0] = accel.xData;
  twig_data->accel[1] = accel.yData;
  twig_data->accel[2] = accel.zData;

  twig_data->gyro[0] = gyro.xData;
  twig_data->gyro[1] = gyro.yData;
  twig_data->gyro[2] = gyro.zData;

  twig_data->mag[0] = magX;
  twig_data->mag[1] = magY;
  twig_data->mag[2] = magZ;

  twig_data->quat[0] = quaternion.element.x;
  twig_data->quat[1] = quaternion.element.y;
  twig_data->quat[2] = quaternion.element.z;
  twig_data->quat[3] = quaternion.element.w;

  twig_data->heading = heading;
  twig_data->ts_micro = ((float)timestamp / (float)CLOCKS_PER_SEC) * 1000000;

  // Note: I think we want this to determine earth based acceleration for integration
  // const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

  // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
  //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
  //        earth.axis.x, earth.axis.y, earth.axis.z);

  // We aren't doing step counts, but I think we will use FSM feature of the IMX MLC core
  // to store these counts in the registers and then read them back out to keep the event
  // handling code simpler ;)
  // myIMU.readRegister(&(twig_data->step_count), LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
}

void imu_print(struct twig_struct *twig_data)
{
  // Serial.print("Battery Level %: ");
  // Serial.println(level);
  // Serial.print("Accel: ");
  // Serial.print("x: ");
  // Serial.print(twig_data->accel[0]);
  // Serial.print(" ");
  // Serial.print("y: ");
  // Serial.print(twig_data->accel[1]);
  // Serial.print(" ");
  // Serial.print("z: ");
  // Serial.print(twig_data->accel[2]);
  // Serial.println(" ");
  // Serial.print("Gyro: ");
  // Serial.print("x: ");
  // Serial.print(twig_data->gyro[0]);
  // Serial.print(" ");
  // Serial.print("y: ");
  // Serial.print(twig_data->gyro[1]);
  // Serial.print(" ");
  // Serial.print("z: ");
  // Serial.print(twig_data->gyro[2]);
  // Serial.println(" ");
  // Serial.print("Mag: ");
  // Serial.print("x: ");
  // Serial.print(twig_data->mag[0]);
  // Serial.print(" ");
  // Serial.print("y: ");
  // Serial.print(twig_data->mag[1]);
  // Serial.print(" ");
  // Serial.print("z: ");
  // Serial.print(twig_data->mag[2]);
  // Serial.println(" ");
  // Serial.print("Heading: ");
  // Serial.print("degrees: ");
  // Serial.print(twig_data->heading);
  // Serial.println(" ");
  // Serial.print("Quaternion: ");
  // Serial.print("x: ");
  // Serial.print(twig_data->quat[0]);
  // Serial.print(" ");
  // Serial.print("y: ");
  // Serial.print(twig_data->quat[1]);
  // Serial.print(" ");
  // Serial.print("z: ");
  // Serial.print(twig_data->quat[2]);
  // Serial.print(" ");
  // Serial.print("w: ");
  // Serial.print(twig_data->quat[3]);
  // Serial.println(" ");
}

void imu_print_motion_cal(struct twig_struct *twig_data)
{
  // 'Raw' values to match expectation of MotionCal
  Serial.print("Raw:");
  Serial.print(int(twig_data->accel[0] * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(twig_data->accel[1] * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(twig_data->accel[2] * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(twig_data->gyro[0] * DEGREES_PER_RADIAN * 16));
  Serial.print(",");
  Serial.print(int(twig_data->gyro[1] * DEGREES_PER_RADIAN * 16));
  Serial.print(",");
  Serial.print(int(twig_data->gyro[2] * DEGREES_PER_RADIAN * 16));
  Serial.print(",");
  Serial.print(int(twig_data->mag[0] * 10));
  Serial.print(",");
  Serial.print(int(twig_data->mag[1] * 10));
  Serial.print(",");
  Serial.print(int(twig_data->mag[2] * 10));
  Serial.println("");

  // unified data
  Serial.print("Uni:");
  Serial.print(twig_data->accel[0]);
  Serial.print(",");
  Serial.print(twig_data->accel[1]);
  Serial.print(",");
  Serial.print(twig_data->accel[2]);
  Serial.print(",");
  Serial.print(twig_data->gyro[0], 4);
  Serial.print(",");
  Serial.print(twig_data->gyro[1], 4);
  Serial.print(",");
  Serial.print(twig_data->gyro[2], 4);
  Serial.print(",");
  Serial.print(twig_data->mag[0]);
  Serial.print(",");
  Serial.print(twig_data->mag[1]);
  Serial.print(",");
  Serial.print(twig_data->mag[2]);
  Serial.println("");
}

void imu_print_x_imu3_gui(struct twig_struct *twig_data)
{
  // https://x-io.co.uk/downloads/x-IMU3-User-Manual-v1.0.pdf (page #23-24)

  /*
  The following message examples are for a timestamp of 1 second (1,000,000 microseconds) and argument
  values of:
    1. Gyroscope X axis = 0
    2. Gyroscope Y axis = 0
    3. Gyroscope Z axis = 0
    4. Accelerometer X axis = 0
    5. Accelerometer Y axis = 0
    6. Accelerometer Z axis = 1

    ASCII example: I,1000000,0.0000,0.0000,0.0000,0.0000,0.0000,1.0000\r\n
  */
  Serial.print("I");
  Serial.print(",");
  Serial.print(twig_data->ts_micro);
  Serial.print(",");
  Serial.print(twig_data->gyro[0], 4);
  Serial.print(",");
  Serial.print(twig_data->gyro[1], 4);
  Serial.print(",");
  Serial.print(twig_data->gyro[2], 4);
  Serial.print(",");
  Serial.print(twig_data->accel[0], 4);
  Serial.print(",");
  Serial.print(twig_data->accel[1], 4);
  Serial.print(",");
  Serial.print(twig_data->accel[2], 4);
  Serial.println("");

  /*
    The following message examples are for a timestamp of 1 second (1,000,000 microseconds) and argument
    values of:
      1. Magnetometer X axis = 1
      2. Magnetometer Y axis = 0
      3. Magnetometer Z axis = 0

      ASCII example: M,1000000,1.0000,0.0000,0.0000\r\n
  */
  Serial.print("M");
  Serial.print(",");
  Serial.print(twig_data->ts_micro);
  Serial.print(",");
  Serial.print(twig_data->mag[0], 4);
  Serial.print(",");
  Serial.print(twig_data->mag[1], 4);
  Serial.print(",");
  Serial.print(twig_data->mag[2], 4);
  Serial.println("");

  /*
    The following message examples are for a timestamp of 1 second (1,000,000 microseconds) and argument
    values of:
      1. Quaternion W element = 1
      2. Quaternion X element = 0
      3. Quaternion Y element = 0
      4. Quaternion Z element = 0
    ASCII example: Q,1000000,1.0000,0.0000,0.0000,0.0000\r\n
  */
  Serial.print("Q");
  Serial.print(",");
  Serial.print(twig_data->ts_micro);
  Serial.print(",");
  Serial.print(twig_data->quat[3], 4);
  Serial.print(",");
  Serial.print(twig_data->quat[0], 4);
  Serial.print(",");
  Serial.print(twig_data->quat[1], 4);
  Serial.print(",");
  Serial.print(twig_data->quat[2], 4);
  Serial.println("");
}

void imu_init(void)
{
  // Define calibration (replace with actual calibration data)
  // const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  // const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
  // const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
  // const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  // const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
  // const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
  // const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  // const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

  // Initialise algorithms
  // FusionOffset offset;

  // FusionOffsetInitialise(&offset, IMU_SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
      .gain = 0.5f,                            // This also needs to be validated
      .accelerationRejection = 10.0f,          // This might need to be changed for slapshots!
      .magneticRejection = 20.0f,              // This also needs to be validated - we are using a magnetometer
      .rejectionTimeout = 5 * IMU_SAMPLE_RATE, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);

  Wire.begin();
  Wire.setClock(400000); // max i2c bus speed = 400KHz
  //  Serial.begin(115200);

  if (!myIMU.begin())
  {
    Serial.println("IMU: did not begin.");
    while (1)
      ;
  }

  // Note: THIS SHOULD ONLY BE DONE DURING DEBUGGING!!!! Causes first readings to be whack
  // https://github.com/sparkfun/SparkFun_6DoF_ISM330DHCX_Arduino_Library/issues/7
  // -----------------------------------------------------------------------------
  // Reset the device to default settings. This if helpful is you're doing multiple
  // uploads testing different settings.
  myIMU.deviceReset();

  // Wait for it to finish reseting
  while (!myIMU.getDeviceReset())
  {
    delay(1);
  }
  // -----------------------------------------------------------------------------

  Serial.println("IMU: Reseting.");
  Serial.println("IMU: Applying settings.");
  delay(100);

  myIMU.setDeviceConfig();
  myIMU.setBlockDataUpdate();

  // Set the output data rate and precision of the accelerometer
  myIMU.setAccelDataRate(ISM_XL_ODR_104Hz);
  myIMU.setAccelFullScale(ISM_2g);

  // Set the output data rate and precision of the gyroscope
  myIMU.setGyroDataRate(ISM_GY_ODR_104Hz);
  myIMU.setGyroFullScale(ISM_250dps);

  // Turn on the accelerometer's filter and apply settings.
  myIMU.setAccelFilterLP2();
  myIMU.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

  // Turn on the gyroscope's filter and apply settings.
  myIMU.setGyroFilterLP1();
  myIMU.setGyroLP1Bandwidth(ISM_MEDIUM);

  if (myMag.begin() == false)
  {
    Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
    while (true)
      ;
  }

  myMag.softReset();
  Serial.println("MMC5983MA connected");

  // @TODO - we also need to configure the imu/mag settings for resolution, filters etc...
  // which most definitely will need to be tweaked to get the best performance out of
  // the imu specific for our use case of measuring hockey stick movements and the
  // sensitivity of the particular imu and mag chips we are using (STM)

  // we will need to plug into the interrupts for event handling of MLC core and FSM
  // state management / count processing
  // pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
  // attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), tap_isr, RISING);

  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);         // INTERRUPTS_ENABLE, SLOPE_FDS
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);         // enable pedometer
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0b11000100); // tap threshold
  // // myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F); //tap duraction
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0b01000000); // single tap route to int
}
