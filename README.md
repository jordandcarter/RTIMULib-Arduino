# RTIMULib-Arduino - a versatile 9-dof and 10-dof IMU library for the Arduino

RTIMULib-Arduino is the simplest way to connect a 9-dof or 10-dof IMU to an Arduino (Uno or Mega) and obtain fully fused quaternion or Euler angle pose data.

## Please note that this library is no longer supported.

## Features

RTIMULib-Arduino currently supports the following IMUs via I2C:

* InvenSense MPU-9150 single chip IMU.
* InvenSense MPU-6050 plus HMC5883 magnetometer on MPU-6050's aux bus (handled by the MPU-9150 driver).
* InvenSense MPU-6050 gyros + acclerometers. Treated as MPU-9150 without magnetometers.
* InvenSense MPU-9250 single chip IMU
* STM LSM9DS0 single chip IMU
* L3GD20H + LSM303D (optionally with the LPS25H) as used on the Pololu AltIMU-10 v4.
* L3GD20 + LSM303DLHC as used on the Adafruit 9-dof (older version with GD20 gyro) IMU. 
* L3GD20H + LSM303DLHC (optionally with BMP180) as used on the new Adafruit 10-dof IMU.
* Bosch BNO055 9-dof IMU with onchip fusion (see notes below).

Pressure/temperature sensing is supported for the following pressure sensors:

* BMP180
* LPS25H
* MS5611

Select the IMU in use by editing libraries/RTIMULib/RTIMULibDefs.h and uncommenting one of the supported IMUs like this:

	#define MPU9150_68                      // MPU9150 at address 0x68
	//#define MPU9150_69                      // MPU9150 at address 0x69
	//#define MPU9250_68                      // MPU9250 at address 0x68
	//#define MPU9250_69                      // MPU9250 at address 0x69
	//#define LSM9DS0_6a                      // LSM9DS0 at address 0x6a
	//#define LSM9DS0_6b                      // LSM9DS0 at address 0x6b
	//#define GD20HM303D_6a                   // GD20H + M303D at address 0x6a
	//#define GD20HM303D_6b                   // GD20H + M303D at address 0x6b
	//#define GD20M303DLHC_6a                 // GD20 + M303DLHC at address 0x6a
	//#define GD20M303DLHC_6b                 // GD20 + M303DLHC at address 0x6b
	//#define GD20HM303DLHC_6a                // GD20H + M303DLHC at address 0x6a
	//#define GD20HM303DLHC_6b                // GD20H + M303DLHC at address 0x6b
     //#define BNO055_28                       // BNO055 at address 0x28
     //#define BNO055_29                       // BNO055 at address 0x29

Once this has been done, all example sketches will build for the selected IMU.

To enable a pressure sensor, uncomment one of the following lines in libraries/RTIMULib/RTIMULibDefs.h:

	//#define BMP180                          // BMP180
	//#define LPS25H_5c                       // LPS25H at standard address
	//#define LPS25H_5d                       // LPS25H at option address
	//#define MS5611_76                       // MS5611 at standard address
	//#define MS5611_77                       // MS5611 at option address


The actual RTIMULib and support libraries are in the library directory. The other top level directories contain example sketches.

*** Important note ***
It is essential to calibrate the magnetometers (except for the BNO055 IMU) or else very poor results will obtained, especially with the MPU-9150 and MPU-9250. If odd results are being obtained, suspect the magnetometer calibration! 

### Special notes for the BNO055

The Bosch BNO055 can perform onchip fusion and also handles magnetometer calibration. Therefore, ArduinoMagCal need not be used. If the ArduinoIMU sketch is used, RTFusion RTQF performs the fusion using the BNO055's sensors. If the ArduinoBNO055 sketch is used, the BNO055's onchip fusion results are used. This results in a small flash memory footprint of approximately 11.5k bytes.

## The Example Sketches

### Build and run

To build and run the example sketches, start the Arduino IDE and use File --> Preferences and then set the sketchbook location to:

	.../RTIMULib-Arduino

where "..." represents the path to the RTIMULib-Arduino directory. The directory is set up so that there's no need to copy the libraries into the main Arduino libraries directory although this can be done if desired.

### ArduinoMagCal

This sketch can be used to calibrate the magnetometers and should be run before trying to generate fused pose data. It also needs to be rerun at any time that the configuration is changed (such as different IMU or different IMU reference orientation). Load the sketch and waggle the IMU around, making sure all axes reach their minima and maxima. The display will stop updating when this occurs. Then, enter 's' followed by enter into the IDE serial monitor to save the data.

### ArduinoIMU

ArduinoIMU is the main demo sketch. It configures the IMU based on settings in RTIMUSettings.cpp. Change these to alter any of the parameters. The display is updated only 3 times per second regardless of IMU sample rate.

Note that, prior to version 2.2.0, the gyro bias is being calculated during the first 5 seconds. If the IMU is moved during this period, the bias calculation may be incorrect and the code will need to be restarted. Starting at version 2.2.0 this is no longer a problem and gyro bias will be reported as valid after the required number of stable samples have been obtained.

If using this sketch with the BNO055, RTFusionRTQF performs the fusion and the BNO055's internal fusion results are not used. Magnetometer calibration data, if present, is also not used as the BNO055 performs this onchip.

### ArduinoBNO055

This is a special version of ArduinoIMU for the BNO055 that uses the IMU's internal fusion results. It is still necessary to uncomment the correct BNO055 IMU address option in RTIMULibDefs.h. No magnetometer calibration is required as this is performed by the BNO055.

### ArduinoIMU10

This is exactly the same as ArduinoIMU except that it adds support for a pressure sensor. One of the pressure sensors in libraries/RTIMULib/RTIMULibDefs.h must be uncommented for this sketch to run. It will display the current pressure and height above standard sea level in addition to pose information from the IMU.

### ArduinoAccel

This is similar to ArduinoIMU except that it subtracts the rotated gravity vector from the accelerometer outputs in order to obtain the residual accelerations - i.e. those not attributable to gravity.

### RTArduLinkIMU

This sketch sends the fused data from the IMU over the Arduino's USB serial link to a host computer running either RTHostIMU or RTHostIMUGL (whcih can be found in the main RTIMULib repo). Basically just build and download the sketch and that's all that needs to be done. Magnetometer calibration can be performed either on the Arduino or within RTHostIMU/RTHostIMUGL.

