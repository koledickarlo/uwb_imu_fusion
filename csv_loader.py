import pandas as pd
import math

_inv_sensor_type_to_string = {
    0: "RESERVED",  # Reserved ID: do not use */
    1: "ACCELEROMETER",  # Accelerometer */
    2: "MAGNETOMETER",  # Magnetic field */
    3: "ORIENTATION",  # Deprecated orientation */
    4: "GYROSCOPE",  # Gyroscope */
    5: "LIGHT",  # Ambient light sensor */
    6: "PRESSURE",  # Barometer */
    7: "TEMPERATURE",  # Temperature */
    8: "PROXIMITY",  # Proximity */
    9: "GRAVITY",  # Gravity */
    10: "LINEAR_ACCELERATION",  # Linear acceleration */
    11: "ROTATION_VECTOR",  # Rotation vector */
    12: "HUMIDITY",  # Relative humidity */
    13: "AMBIENT_TEMPERATURE",  # Ambient temperature */
    14: "UNCAL_MAGNETOMETER",  # Uncalibrated magnetic field */
    15: "GAME_ROTATION_VECTOR",  # Game rotation vector */
    16: "UNCAL_GYROSCOPE",  # Uncalibrated gyroscope */
    17: "SMD",  # Significant motion detection */
    18: "STEP_DETECTOR",  # Step detector */
    19: "STEP_COUNTER",  # Step counter */
    20: "GEOMAG_ROTATION_VECTOR",  # Geomagnetic rotation vector */
    21: "HEART_RATE",  # Heart rate */
    22: "TILT_DETECTOR",  # Tilt detector */
    23: "WAKE_GESTURE",  # Wake-up gesture  */
    24: "GLANCE_GESTURE",  # Glance gesture  */
    25: "PICK_UP_GESTURE",  # Pick-up gesture */
    26: "BAC",  # Basic Activity Classifier */
    27: "PDR",  # Pedestrian Dead Reckoning */
    28: "B2S",  # Bring to see */
    29: "3AXIS",  # 3 Axis sensor */
    30: "EIS",  # Electronic Image Stabilization */
    31: "OIS",  # Optical Image Stabilization */
    32: "RAW_ACCELEROMETER",  # Raw accelerometer */
    33: "RAW_GYROSCOPE",  # Raw gyroscope */
    34: "RAW_MAGNETOMETER",  # Raw magnetometer */
    35: "RAW_TEMPERATURE",  # Raw temperature */
    36: "CUSTOM_PRESSURE",  # Custom Pressure Sensor */
    37: "MIC",  # Stream audio from microphone */
    38: "TSIMU",  # TS-IMU */
    39: "RAW_PPG",  # Raw Photoplethysmogram */
    40: "HRV",  # Heart rate variability */
    41: "SLEEP_ANALYSIS",  # Sleep analysis */
    42: "BAC_EXTENDED",  # Basic Activity Classifier Extended */
    43: "BAC_STATISTICS",  # Basic Activity Classifier Statistics */
    44: "FLOOR_CLIMB_COUNTER",  # Floor Climbed Counter */
    45: "ENERGY_EXPENDITURE",  # Energy Expenditure */
    46: "DISTANCE",  # Distance */
    47: "SHAKE",  # Shake Gesture */
    48: "DOUBLE_TAP",  # Double Tap */
}


def loadIMUCSVReturnFormatedSensorDataFrame(sensor, filename):
    """Reads data from diagnostic setup CSV input file and returns seperate pandas
    DataFrame per selected sensor.

    Parameters
    ----------
    sensor : str
        Name of the sensor data that needs to be filtered out. Possible values
        case-insensitive values are:
            - "ACCELEROMETER"
            - "MAGNETOMETER"
            - "ORIENTATION"
            - "GYROSCOPE"
            - "GRAVITY"
            - "LINEAR_ACCELERATION"
            - "ROTATION_VECTOR"
            - "UNCAL_MAGNETOMETER"
            - "GAME_ROTATION_VECTOR"
            - "UNCAL_GYROSCOPE"
            - "STEP_DETECTOR"
            - "STEP_COUNTER"
            - "GEOMAG_ROTATION_VECTOR"
            - "RAW_ACCELEROMETER"
            - "RAW_GYROSCOPE"
            - "RAW_MAGNETOMETER"

    filename : str
        Path to the IMU.csv provided by the diagnostic hardware

    Returns
    -------
    DataFrame
    """
    data = pd.read_csv(filename)

    # Format time
    data["TL"] = data["TL"].apply(int, base=16)
    data["time (us)"] = data["TH"].apply(lambda x: (x << 32)) + data["TL"]
    data.drop(labels="TL", axis=1, inplace=True)
    data.drop(labels="TH", axis=1, inplace=True)

    # Find out id of the string
    sensor_id = -1
    for key, value in _inv_sensor_type_to_string.items():
        if value.lower() == sensor.lower():
            sensor_id = key

    # If there is no such string return empty data frame
    if sensor_id == -1:
        return pd.DataFrame()

    # Parse data per sensor and make a copy
    sensor_data = data.loc[data["s"] == sensor_id].copy()

    # Format the data per IMU specification if there is data in frame
    if sensor_data.empty is False:
        if (
            _inv_sensor_type_to_string[sensor_id] == "LINEAR_ACCELERATION"
            or _inv_sensor_type_to_string[sensor_id] == "ACCELEROMETER"
            or _inv_sensor_type_to_string[sensor_id] == "GRAVITY"
        ):

            sensor_data.rename(
                inplace=True,
                columns={"x/q0": "x (g)", "y/q1": "y (g)", "z/q2": "z (g)"},
            )

            # Scale the data to g
            sensor_data["x (g)"] = sensor_data["x (g)"].div(1000)
            sensor_data["y (g)"] = sensor_data["y (g)"].div(1000)
            sensor_data["z (g)"] = sensor_data["z (g)"].div(1000)

            if _inv_sensor_type_to_string[sensor_id] == "ACCELEROMETER":
                sensor_data.rename(
                    inplace=True,
                    columns={
                        "bx/q4": "bias_x (g)",
                        "by/acc_heading": "bias_y (g)",
                        "bz": "bias_z (g)",
                    },
                )

                # Scale the data to g
                sensor_data["bias_x (g)"] = sensor_data["bias_x (g)"].div(1000)
                sensor_data["bias_y (g)"] = sensor_data["bias_y (g)"].div(1000)
                sensor_data["bias_z (g)"] = sensor_data["bias_z (g)"].div(1000)
            else:
                sensor_data.drop(labels="bx/q4", axis=1, inplace=True)
                sensor_data.drop(labels="by/acc_heading", axis=1, inplace=True)
                sensor_data.drop(labels="bz", axis=1, inplace=True)

        elif (
            _inv_sensor_type_to_string[sensor_id] == "GYROSCOPE"
            or _inv_sensor_type_to_string[sensor_id] == "UNCAL_GYROSCOPE"
        ):

            sensor_data.rename(
                inplace=True,
                columns={"x/q0": "x (dps)", "y/q1": "y (dps)", "z/q2": "z (dps)"},
            )

            # Scale the data to g
            sensor_data["x (dps)"] = sensor_data["x (dps)"].div(1000)
            sensor_data["y (dps)"] = sensor_data["y (dps)"].div(1000)
            sensor_data["z (dps)"] = sensor_data["z (dps)"].div(1000)

            if _inv_sensor_type_to_string[sensor_id] == "UNCAL_GYROSCOPE":
                sensor_data.rename(
                    inplace=True,
                    columns={
                        "bx/q4": "bias_x (dps)",
                        "by/acc_heading": "bias_y (dps)",
                        "bz": "bias_z (dps)",
                    },
                )

                # Scale the data to g
                sensor_data["bias_x (dps)"] = sensor_data["bias_x (dps)"].div(1000)
                sensor_data["bias_y (dps)"] = sensor_data["bias_y (dps)"].div(1000)
                sensor_data["bias_z (dps)"] = sensor_data["bias_z (dps)"].div(1000)
            else:
                sensor_data.drop(labels="bx/q4", axis=1, inplace=True)
                sensor_data.drop(labels="by/acc_heading", axis=1, inplace=True)
                sensor_data.drop(labels="bz", axis=1, inplace=True)

        elif (
            _inv_sensor_type_to_string[sensor_id] == "MAGNETOMETER"
            or _inv_sensor_type_to_string[sensor_id] == "UNCAL_MAGNETOMETER"
        ):

            sensor_data.rename(
                inplace=True,
                columns={"x/q0": "x (T)", "y/q1": "y (T)", "z/q2": "z (T)"},
            )

            # Scale the data to g
            sensor_data["x (T)"] = sensor_data["x (T)"].div(1000)
            sensor_data["y (T)"] = sensor_data["y (T)"].div(1000)
            sensor_data["z (T)"] = sensor_data["z (T)"].div(1000)

            if _inv_sensor_type_to_string[sensor_id] == "UNCAL_MAGNETOMETER":
                sensor_data.rename(
                    inplace=True,
                    columns={
                        "bx/q4": "bias_x (T)",
                        "by/acc_heading": "bias_y (T)",
                        "bz": "bias_z (T)",
                    },
                )

                # Scale the data to g
                sensor_data["bias_x (T)"] = sensor_data["bias_x (T)"].div(1000)
                sensor_data["bias_y (T)"] = sensor_data["bias_y (T)"].div(1000)
                sensor_data["bias_z (T)"] = sensor_data["bias_z (T)"].div(1000)
            else:
                sensor_data.drop(labels="bx/q4", axis=1, inplace=True)
                sensor_data.drop(labels="by/acc_heading", axis=1, inplace=True)
                sensor_data.drop(labels="bz", axis=1, inplace=True)

        elif (
            _inv_sensor_type_to_string[sensor_id] == "GAME_ROTATION_VECTOR"
            or _inv_sensor_type_to_string[sensor_id] == "ROTATION_VECTOR"
            or _inv_sensor_type_to_string[sensor_id] == "GEOMAG_ROTATION_VECTOR"
        ):

            sensor_data.rename(
                inplace=True,
                columns={
                    "x/q0": "q0",
                    "y/q1": "q1",
                    "z/q2": "q2",
                    "bx/q4": "q3",
                    "by/acc_heading": "accuracy (deg)",
                },
            )

            # Scale the data to g
            sensor_data["q0"] = sensor_data["q0"].div(1000)
            sensor_data["q1"] = sensor_data["q1"].div(1000)
            sensor_data["q2"] = sensor_data["q2"].div(1000)
            sensor_data["q3"] = sensor_data["q3"].div(1000)

            if _inv_sensor_type_to_string[sensor_id] != "GAME_ROTATION_VECTOR":
                sensor_data.drop(labels="accuracy", axis=1, inplace=True)

            sensor_data.drop(labels="bz", axis=1, inplace=True)

        elif _inv_sensor_type_to_string[sensor_id] == "ORIENTATION":

            sensor_data.rename(
                inplace=True,
                columns={"x/q0": "x (deg)", "y/q1": "y (deg)", "z/q2": "z (deg)"},
            )

            # Scale the data to g
            # Scale the data to g
            sensor_data["x (deg)"] = sensor_data["x (deg)"].div(1000)
            sensor_data["y (deg)"] = sensor_data["y (deg)"].div(1000)
            sensor_data["z (deg)"] = sensor_data["z (deg)"].div(1000)

            sensor_data.drop(labels="bx/q4", axis=1, inplace=True)
            sensor_data.drop(labels="by/acc_heading", axis=1, inplace=True)
            sensor_data.drop(labels="bz", axis=1, inplace=True)

    return sensor_data


def loadUWBCSVReturnFormatedDataFrame(filename):
    """Reads UWB data from diagnostic setup CSV input file and returns formated
    pandas DataFrame which contains combined timestamp values (High and low in one
    value), temperature, voltage, deltas and distance.

    Note: internal DW1000 timestamps are 40-bit registers with LSB = 15.6 * 10**-12
    seconds.

    Parameters
    ----------
    filename : str
        Path to the UWB.csv provided by the diagnostic hardware

    Returns
    -------
    DataFrame
        pandas DataFrame object
    """
    data = pd.read_csv(filename)

    # Format time
    data["TL"] = data["TL"].apply(int, base=16)
    data["time(us)"] = data["TH"].apply(lambda x: (x << 32)) + data["TL"]
    data.drop(labels="TL", axis=1, inplace=True)
    data.drop(labels="TH", axis=1, inplace=True)

    # Temperature and voltage
    data["voltage"] = (0.0057 * data["tempvbat"].apply(lambda x: x & 0xFF)) + 2.3
    data["temperature"] = (
        1.13 * data["tempvbat"].apply(lambda x: (x >> 8) & 0xFF)
    ) - 113.0
    data.drop(labels="tempvbat", axis=1, inplace=True)

    # Timestamps
    data["T1L"] = data["T1L"].apply(int, base=16)
    data["T1H"] = data["T1H"].apply(int, base=16)
    data["local_sent"] = data["T1H"].apply(lambda x: (x << 32)) + data["T1L"]
    data.drop(labels="T1L", axis=1, inplace=True)
    data.drop(labels="T1H", axis=1, inplace=True)

    data["T2L"] = data["T2L"].apply(int, base=16)
    data["T2H"] = data["T2H"].apply(int, base=16)
    data["local_receive"] = data["T2H"].apply(lambda x: (x << 32)) + data["T2L"]
    data.drop(labels="T2L", axis=1, inplace=True)
    data.drop(labels="T2H", axis=1, inplace=True)

    data["T3L"] = data["T3L"].apply(int, base=16)
    data["T3H"] = data["T3H"].apply(int, base=16)
    data["remote_sent"] = data["T3H"].apply(lambda x: (x << 32)) + data["T3L"]
    data.drop(labels="T3L", axis=1, inplace=True)
    data.drop(labels="T3H", axis=1, inplace=True)

    data["T4L"] = data["T4L"].apply(int, base=16)
    data["T4H"] = data["T4H"].apply(int, base=16)
    data["remote_receive"] = data["T4H"].apply(lambda x: (x << 32)) + data["T4L"]
    data.drop(labels="T4L", axis=1, inplace=True)
    data.drop(labels="T4H", axis=1, inplace=True)

    # Calculate differences
    data.loc[(data["local_receive"] - data["local_sent"]) > 0, "local_delta"] = (
        data["local_receive"] - data["local_sent"]
    )
    data.loc[(data["local_receive"] - data["local_sent"]) < 0, "local_delta"] = (
        data["local_receive"] - data["local_sent"] + 2**40
    )

    data.loc[(data["remote_sent"] - data["remote_receive"]) > 0, "remote_delta"] = (
        data["remote_sent"] - data["remote_receive"]
    )
    data.loc[(data["remote_sent"] - data["remote_receive"]) < 0, "remote_delta"] = (
        data["remote_sent"] - data["remote_receive"] + 2**40
    )

    # Calculate delta
    data.loc[(data["local_delta"] - data["remote_delta"]) > 0, "delta"] = (
        data["local_delta"] - data["remote_delta"]
    )
    data.loc[(data["local_delta"] - data["remote_delta"]) < 0, "delta"] = (
        data["remote_delta"] - data["local_delta"]
    )

    # Calculate distance
    # Convert from DW1000 timestamp to seconds (15.6 *10 **-12) and multiply
    # with speed of light to get distance in meters
    data["distance (m)"] = (data["delta"] / 2) * 15.6 * 10**-12 * 299792458

    return data


def calculateAntennaDelay(data):
    """Returns calibration values for antenna delay from 5.01 m distance
    measurements

    Parameters
    ----------
    data : DataFrame
        pandas DataFrame object generated from UWB.csv

    Returns
    -------
    list
        Three values (total antenna delay, tx antenna delay, rx antenna delay)
    """
    # Drop 0 data
    data = data[data["local_receive"] != 0]

    # Delta corresponds to the RTOF and needs to be divided by 2
    tof = data["delta"].mean() / 2

    # Real distance
    time = 5.01 / 299792458 / 15.6 / 10**-12

    # Antenna delay
    antenna_delay = tof - time

    # Per Application Note APS014 TX is 44% and RX is 56%
    return (
        math.ceil(antenna_delay),
        math.ceil(0.44 * antenna_delay),
        math.ceil(0.56 * antenna_delay),
    )


if __name__ == "__main__":
    data = loadUWBCSVReturnFormatedDataFrame(
        "./mjerenja_kalibracija/calibration_501m/ultrasonic/UWB_M1.CSV"
    )

    print(calculateAntennaDelay(data))
