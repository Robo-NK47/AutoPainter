import serial
import time
import numpy as np
from tqdm.auto import tqdm


def get_stable_distance(min_value, max_value, test_amount, arduino, stdev_limit, sensor_type):
    measurements = np.array([])
    while True:
        for _ in tqdm(range(test_amount)):
            if sensor_type == 'distance':
                measurement = get_single_measurement(arduino)
            if sensor_type == 'force':
                measurement = get_single_analog_measurement(arduino)

            if min_value < measurement < max_value:
                measurements = np.append(measurements, measurement)
        if measurements.std() < stdev_limit:
            value = measurements.mean()
            if sensor_type == 'force':
                value = (value / 1023) * 5
                print(measurements.std())
            return value
        else:
            stdev_limit += 0.5
            print(f'STDev is: {measurements.std():.2f}, stdev_limit raised to: {stdev_limit:.2f}')


def get_single_measurement(arduino):
    arduino.write(b'd')
    return float(arduino.readline().decode().rstrip())


def get_single_analog_measurement(arduino):
    arduino.write(b't')
    return int(arduino.readline().decode().rstrip())


def get_arduino():
    return serial.Serial('/dev/ttyACM0', 57600)


if __name__ == "__main__":
    ser = get_arduino()
    time.sleep(2)
    while True:
        # distance = get_stable_distance(min_value=0, max_value=1000, test_amount=500,
        #                                arduino=ser, stdev_limit=1.0, sensor_type='distance')
        # print(f"Distance: {distance:.2f} [mm]")
        force = get_single_analog_measurement(ser)
        print(f"force: {force:.2f} [V]")
        time.sleep(0.1)

    ser.close()  # Close the serial connection
