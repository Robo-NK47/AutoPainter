import serial
import time
import numpy as np
from tqdm.auto import tqdm


def get_stable_distance(min_value, max_value, test_amount, arduino, stdev_limit):
    measurements = np.array([])
    while True:
        for _ in tqdm(range(test_amount)):
            measurement = get_single_measurement(arduino)
            if min_value < measurement < max_value:
                measurements = np.append(measurements, measurement)
        if measurements.std() < stdev_limit:
            return measurements.mean()
        else:
            stdev_limit += 0.1
            print(f'stdev_limit raised to: {stdev_limit:.2f}')


def get_single_measurement(arduino):
    arduino.write(b'r')
    return float(arduino.readline().decode().rstrip())


if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 57600)
    time.sleep(2)
    while True:
        distance = get_stable_distance(min_value=0, max_value=1000, test_amount=500, arduino=ser, stdev_limit=1.0)
        print(f"Distance: {distance:.2f} [mm]")

    ser.close()  # Close the serial connection
