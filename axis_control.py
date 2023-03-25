import RPi.GPIO as GPIO
import time
from tqdm.auto import tqdm
import math
import threading

# Set up GPIO mode
GPIO.setmode(GPIO.BOARD)


class Motor:
    def __init__(self, name, step_pin, direction_pin, edge_sensor_i_pin,
                 edge_sensor_f_pin, resolution):
        self.direction_key = {'up': 1, 'down': 0, 'right': 1, 'left': 0, 'forward': 1, 'backward': 0}
        self.position = 0
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.name = name
        self.edge_sensor_i_pin = edge_sensor_i_pin
        self.edge_sensor_f_pin = edge_sensor_f_pin
        self.resolution = resolution

        self.setup_motor()

        self.edge_sensor_i_pin_state = edge_sensor_i_pin
        self.edge_sensor_f_pin_state = edge_sensor_f_pin

        self.update_edge_sensors()

    def setup_motor(self):
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.edge_sensor_i_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.edge_sensor_f_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def update_edge_sensors(self):
        i_state = 0
        f_state = 0
        checks = 5
        for _ in range(checks):
            i_state += GPIO.input(self.edge_sensor_i_pin)
            f_state += GPIO.input(self.edge_sensor_f_pin)
            time.sleep(0.001)
        if i_state == 0:
            self.edge_sensor_i_pin_state = 0
        else:
            self.edge_sensor_i_pin_state = 1

        if f_state == 0:
            self.edge_sensor_f_pin_state = 0
        else:
            self.edge_sensor_f_pin_state = 1

    def motor_single_step(self, velocity):
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(velocity / 2)
        GPIO.output(self.step_pin, GPIO.LOW)
        time.sleep(velocity / 2)

    def set_direction(self, direction):
        _dir = self.direction_key[direction]
        if _dir:
            return GPIO.HIGH
        else:
            return GPIO.LOW

    def update_position(self, direction):
        if not isinstance(direction, int):
            _dir = self.direction_key[direction]
        else:
            _dir = direction

        if _dir:
            self.position += self.resolution
        else:
            self.position -= self.resolution

    def motor_for_loop(self, velocity, distance, direction):
        velocity = 1 / velocity
        distance = int(distance / self.resolution)
        GPIO.output(self.direction_pin, self.set_direction(direction))
        for step in tqdm(range(distance)):
            self.update_edge_sensors()
            # if (self.direction_key[direction] and not self.edge_sensor_i_pin_state) or (
            #         not self.direction_key[direction] and not self.edge_sensor_f_pin_state):
            self.motor_single_step(velocity)
            self.update_position(direction)
            # print(self.edge_sensor_i_pin_state, self.edge_sensor_f_pin_state)

    def motor_while_loop(self, velocity, direction):
        velocity = 1 / velocity
        GPIO.output(self.direction_pin, self.set_direction(direction))
        while (not self.direction_key[direction] and self.edge_sensor_i_pin_state) or (
                self.direction_key[direction] and self.edge_sensor_f_pin_state):
            self.update_edge_sensors()
            # print('i: ', self.edge_sensor_i_pin_state, '     f: ', self.edge_sensor_f_pin_state)
            self.motor_single_step(velocity)
            self.update_position(direction)


class Gantry:
    def __init__(self):
        linear_axis_resolution = 8 * math.tan(math.radians(360 / 200))
        z_axis_resolution = 1
        self.x = Motor(name='x', step_pin=37, direction_pin=35,
                       edge_sensor_i_pin=8, edge_sensor_f_pin=10,
                       resolution=linear_axis_resolution)
        self.y = Motor(name='y', step_pin=36, direction_pin=32,
                       edge_sensor_i_pin=12, edge_sensor_f_pin=11,
                       resolution=linear_axis_resolution)
        self.z = Motor(name='z', step_pin=40, direction_pin=38,
                       edge_sensor_i_pin=15, edge_sensor_f_pin=13,
                       resolution=z_axis_resolution)
        self.x_position = self.x.position
        self.y_position = self.y.position
        self.z_position = self.z.position

    def homing_sequence(self):
        x_thread = threading.Thread(target=self.x.motor_while_loop, args=(400, 'backward',))
        y_thread = threading.Thread(target=self.y.motor_while_loop, args=(400, 'right',))
        z_thread = threading.Thread(target=self.z.motor_while_loop, args=(50, 'up',))

        x_thread.start()
        y_thread.start()
        z_thread.start()

        x_thread.join()
        y_thread.join()
        z_thread.join()

        self.x_position = 0
        self.y_position = 0
        self.z_position = 0

        print('Pen is above (0, 0).')

    def calculate_steps(self, axis_name, next_position):
        if axis_name == 'x':
            current_position = self.x_position
            resolution = self.x.resolution
            next_position = next_position['x']

        if axis_name == 'y':
            current_position = self.y_position
            resolution = self.y.resolution
            next_position = next_position['y']

        if axis_name == 'z':
            current_position = self.z_position
            resolution = self.z.resolution
            next_position = next_position['z']

        return int(abs(current_position - next_position) / resolution)

    def calculate_direction(self, current_position, next_position):
        if next_position > current_position:
            return 1

        else:
            return 0

    def gantry_for_loop(self, velocity, direction, steps, axis):
        velocity = 1 / velocity
        GPIO.output(axis.direction_pin, direction)
        for step in tqdm(range(steps)):
            axis.update_edge_sensors()
            # if (axis.direction_key[direction] and not axis.edge_sensor_i_pin_state) or (
            #         not axis.direction_key[direction] and not axis.edge_sensor_f_pin_state):
            axis.motor_single_step(velocity)
            axis.update_position(direction)
            if axis.name == 'x':
                self.x_position = axis.position
            if axis.name == 'y':
                self.y_position = axis.position
            if axis.name == 'z':
                self.z_position = axis.position
            # print(axis.edge_sensor_i_pin_state, axis.edge_sensor_f_pin_state)

        return f'{axis.name} axis done.'

    def move_to_position(self, next_position, velocity):
        x_steps_amount = self.calculate_steps('x', next_position)
        x_direction = self.calculate_direction(self.x_position, next_position['x'])

        y_steps_amount = self.calculate_steps('y', next_position)
        y_direction = self.calculate_direction(self.y_position, next_position['y'])

        z_steps_amount = self.calculate_steps('z', next_position)
        z_direction = self.calculate_direction(self.z_position, next_position['z'])

        x_thread = threading.Thread(target=self.gantry_for_loop,
                                    args=(velocity['x'], x_direction, x_steps_amount, self.x,))
        y_thread = threading.Thread(target=self.gantry_for_loop,
                                    args=(velocity['y'], y_direction, y_steps_amount, self.y,))
        z_thread = threading.Thread(target=self.gantry_for_loop,
                                    args=(velocity['z'], z_direction, z_steps_amount, self.z,))

        x_thread.start()
        y_thread.start()
        z_thread.start()

        x_thread.join()
        y_thread.join()
        z_thread.join()


if __name__ == "__main__":
    gantry = Gantry()
    time.sleep(5)
    gantry.homing_sequence()

    gantry.x.motor_while_loop(400, 'forward')
    print(f'X position: {gantry.x.position}')

    gantry.y.motor_while_loop(400, 'left')
    print(f'Y position: {gantry.y.position}')

    gantry.z.motor_while_loop(50, 'down')
    print(f'Z position: {gantry.z.position}')
