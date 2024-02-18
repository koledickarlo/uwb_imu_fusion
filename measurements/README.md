# UWB measurements with OptiTrack ground truth

These are are measurement of Ultrax diagnostic hardware performed in laboratory
conditions with OptiTrack as a reference device. Rigid body marker was placed
on the mobile tag that contains UWB and IMU sensors. Offset from centre of the
rigid body and UWB antenna was measured to be 38 mm. Static markers were placed
on poles that were equipped with two anchors. One on height of **15 cm** from
the ground and another 161 cm from the ground. Static marker was placed on the
top of the pole and their offset from centre is **10 mm**. Odd anchor nodes
were placed closer to the ground while even anchors on the top of the poles.
**Also markers were placed on the nodes itself with offset of 10 mm**

**Three** measurement were made. One with L shape movement, another with
forward-backward type movement **and lastly jumps in the air**. Every
measurement has its own corresponding folder name for data from Ultrax
diagnostic mobile tag and "optitrack" for data produced by OptiTrack system.
All datasets
are CSV format which will be explained.

## Ultrax diagnostic CSV data format

There are two files that are being produced by the mobile tag. "uwb.csv" file
contains RTOF (Round Trip of Flight) measurements from every stationary anchors
placed on the poles while "imu.csv" contains data from the 9-DOF inertial
sensor.

### IMU.CSV

On first line IMU data contains following string:
"TH,TL,s,x/q0,y/q1,z/q2,bx/q4,by/acc_heading,bz,accuracy"

Since there are multiple types of sensor data sources columns have shared
names.

 - "TH" and "TL" represent upper 32 and lower 32 bits of time stamp on which
   data event was recorded. Time stamp is in microseconds and value are in HEX.

 - "s" represents code of the sensor type. More on sensor types can be found in
   ICM-20948 datasheet or example code. There are following types:

	INV_SENSOR_TYPE_RESERVED                     = 0 ,  /**< Reserved ID: do not use */
	INV_SENSOR_TYPE_ACCELEROMETER                = 1 ,  /**< Accelerometer */
	INV_SENSOR_TYPE_MAGNETOMETER                 = 2 ,  /**< Magnetic field */
	INV_SENSOR_TYPE_ORIENTATION                  = 3 ,  /**< Deprecated orientation */
	INV_SENSOR_TYPE_GYROSCOPE                    = 4 ,  /**< Gyroscope */
	INV_SENSOR_TYPE_LIGHT                        = 5 ,  /**< Ambient light sensor */
	INV_SENSOR_TYPE_PRESSURE                     = 6 ,  /**< Barometer */
	INV_SENSOR_TYPE_TEMPERATURE                  = 7 ,  /**< Temperature */
	INV_SENSOR_TYPE_PROXIMITY                    = 8 ,  /**< Proximity */
	INV_SENSOR_TYPE_GRAVITY                      = 9 ,  /**< Gravity */
	INV_SENSOR_TYPE_LINEAR_ACCELERATION          = 10,  /**< Linear acceleration */
	INV_SENSOR_TYPE_ROTATION_VECTOR              = 11,  /**< Rotation vector */
	INV_SENSOR_TYPE_HUMIDITY                     = 12,  /**< Relative humidity */
	INV_SENSOR_TYPE_AMBIENT_TEMPERATURE          = 13,  /**< Ambient temperature */
	INV_SENSOR_TYPE_UNCAL_MAGNETOMETER           = 14,  /**< Uncalibrated magnetic field */
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR         = 15,  /**< Game rotation vector */
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE              = 16,  /**< Uncalibrated gyroscope */
	INV_SENSOR_TYPE_SMD                          = 17,  /**< Significant motion detection */
	INV_SENSOR_TYPE_STEP_DETECTOR                = 18,  /**< Step detector */
	INV_SENSOR_TYPE_STEP_COUNTER                 = 19,  /**< Step counter */
	INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR       = 20,  /**< Geomagnetic rotation vector */
	INV_SENSOR_TYPE_HEART_RATE                   = 21,  /**< Heart rate */
	INV_SENSOR_TYPE_TILT_DETECTOR                = 22,  /**< Tilt detector */
	INV_SENSOR_TYPE_WAKE_GESTURE                 = 23,  /**< Wake-up gesture  */
	INV_SENSOR_TYPE_GLANCE_GESTURE               = 24,  /**< Glance gesture  */
	INV_SENSOR_TYPE_PICK_UP_GESTURE              = 25,  /**< Pick-up gesture */
	INV_SENSOR_TYPE_BAC                          = 26,  /**< Basic Activity Classifier */
	INV_SENSOR_TYPE_PDR                          = 27,  /**< Pedestrian Dead Reckoning */
	INV_SENSOR_TYPE_B2S                          = 28,  /**< Bring to see */
	INV_SENSOR_TYPE_3AXIS                        = 29,  /**< 3 Axis sensor */
	INV_SENSOR_TYPE_EIS                          = 30,  /**< Electronic Image Stabilization */
	INV_SENSOR_TYPE_OIS                          = 31,  /**< Optical Image Stabilization */
	INV_SENSOR_TYPE_RAW_ACCELEROMETER            = 32,  /**< Raw accelerometer */
	INV_SENSOR_TYPE_RAW_GYROSCOPE                = 33,  /**< Raw gyroscope */
	INV_SENSOR_TYPE_RAW_MAGNETOMETER             = 34,  /**< Raw magnetometer */
	INV_SENSOR_TYPE_RAW_TEMPERATURE              = 35,  /**< Raw temperature */
	INV_SENSOR_TYPE_CUSTOM_PRESSURE              = 36,  /**< Custom Pressure Sensor */
	INV_SENSOR_TYPE_MIC                          = 37,  /**< Stream audio from microphone */
	INV_SENSOR_TYPE_TSIMU                        = 38,  /**< TS-IMU */
	INV_SENSOR_TYPE_RAW_PPG                      = 39,  /**< Raw Photoplethysmogram */
	INV_SENSOR_TYPE_HRV                          = 40,  /**< Heart rate variability */
	INV_SENSOR_TYPE_SLEEP_ANALYSIS               = 41,  /**< Sleep analysis */
	INV_SENSOR_TYPE_BAC_EXTENDED                 = 42,  /**< Basic Activity Classifier Extended */
	INV_SENSOR_TYPE_BAC_STATISTICS               = 43,  /**< Basic Activity Classifier Statistics */
	INV_SENSOR_TYPE_FLOOR_CLIMB_COUNTER          = 44,  /**< Floor Climbed Counter */
	INV_SENSOR_TYPE_ENERGY_EXPENDITURE           = 45,  /**< Energy Expenditure */
	INV_SENSOR_TYPE_DISTANCE                     = 46,  /**< Distance */
	INV_SENSOR_TYPE_SHAKE                        = 47,  /**< Shake Gesture */
	INV_SENSOR_TYPE_DOUBLE_TAP                   = 48,  /**< Double Tap */

 - depending on data next six values "x/q0,y/q1,z/q2,bx/q4,by/acc_heading,bz"
   either represent data in x,y,z axis or quaternion format of orientation. All
   data ends with last entry that is accuracy flag which has value from 0-3.
   Higher number corresponds to better accuracy. Data format is as it follows:

		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			raw values in LSB format for x,y,z axis

		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
			values of calibrated acceleration expressed in mg, where g is 9.81 m/s^2 for
			x,y,z axis.
			in case of accelerometer biases for x,y,z axis are expressed. Later
			it was determined that biases are always 0.

		case INV_SENSOR_TYPE_GYROSCOPE:
			values of calibrated rotation speed expressed in mdps for x,y,z axis.

		case INV_SENSOR_TYPE_MAGNETOMETER:
			values of magnetometer expressed in nT for x,y,z axis.

		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			values of uncalibrated rotation speed expressed in mdps for x,y,z
			axis. Next are three values are calculated biases from gyroscope
			for x,y,z axis

		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			values of uncalibrated magnetometer expressed in nT for x,y,z
			axis. Next are three values are calculated biases from gyroscope
			for x,y,z axis

		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		    orientation written in quaternion format.
		    INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR uses 9-DOF while
		    INV_SENSOR_TYPE_GAME_ROTATION_VECTOR uses 6-DOF. Additionally, fith
		    number corresponsd to estimator accuracy in degrees

		case INV_SENSOR_TYPE_ORIENTATION:
		    orientation written in degrees in x,y,z axis

### UWB.CSV

Similarly, "UWB.csv" data contains string on the first line which describes
written data: "TH,TL,#,a,tempvbat,T1H,T1L,T2H,T2L,T3H,T3L,T4H,T4L"

 - "TH" and "TL" represent upper 32 and lower 32 bits of time stamp on which data
event was recorded. Time stamp is in microseconds and value are in HEX.

 - "#" represents frame sequence number which goes from 0 to 255. After each
   RTOF measurement sequence number is increased

 - "a" is the address of the anchor that is being used for RTOF measurements.
   Addresses in these measurements are going from 1 to 6. In reference
   measurements anchor poles are noted as N02N01 where numbers correspond to
   anchors

 - "tempvbat" is value of temperature and battery voltage on DW1000 expressed
   in on HEX number. To get separate values you need to do following:

    voltage = (0.0057 * (x & 0xFF)) + 2.3
    temperature = (1.13 * ((x>>8) & 0xFF)) - 113.0

 - pairs "T1H,T1L", "T2H,T2L", "T3H,T3L", "T4H,T4L" are time stamps of DW1000
   during RTOF measurements expressed in HEX format. Note that register size is
   2^40 so overflows are possible. Each bit represents 15.6 * 10^-12 seconds.
   To get RTOF distance one must do following:

    RTOF = ((T2 - T1) - (T4 - T3)) / 2

## OptiTrack measurements

Data is as well in CSV format and it is self-explanatory. Only thing that needs
to be noted is that measured values are for Rigid Body structure that was
placed on the mobile tag, corresponding markers that form this rigid body and
measurements of the markers that were placed on the top of the pole.

**Additionally rigid body was created for the poles. Since optitrack software
automatically named markers markers on the Top of the pole and on the nodes
don't have unique names. The one on the top has to have largest value for
height.**
