import RPi.GPIO as GPIO
import time
from tqdm.auto import tqdm
import serial
import threading
import distance_control

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
            if (not self.direction_key[direction] and self.edge_sensor_i_pin_state) or (
                    self.direction_key[direction] and self.edge_sensor_f_pin_state):
                self.motor_single_step(velocity)
                self.update_position(direction)
            # print('axis', self.name, 'i: ', self.edge_sensor_i_pin_state, '     f: ', self.edge_sensor_f_pin_state)

    def motor_while_loop(self, velocity, direction):
        velocity = 1 / velocity
        GPIO.output(self.direction_pin, self.set_direction(direction))
        real_stop = False
        while not real_stop:
            while (not self.direction_key[direction] and self.edge_sensor_i_pin_state) or (
                    self.direction_key[direction] and self.edge_sensor_f_pin_state):
                self.update_edge_sensors()
                # print('axis', self.name, 'i: ', self.edge_sensor_i_pin_state, '     f: ', self.edge_sensor_f_pin_state)
                self.motor_single_step(velocity)
                self.update_position(direction)

            pin_count = []

            for i in range(100):
                self.update_edge_sensors()
                if not self.direction_key[direction]:
                    pin_count.append(self.edge_sensor_i_pin_state)
                if self.direction_key[direction]:
                    pin_count.append(self.edge_sensor_f_pin_state)

            if sum(pin_count) > 20:
                real_stop = False
            else:
                real_stop = True

    def axis_to_position(self, velocity, direction):
        velocity = 1 / velocity
        GPIO.output(self.direction_pin, self.set_direction(direction))
        while (not self.direction_key[direction] and self.edge_sensor_i_pin_state) or (
                self.direction_key[direction] and self.edge_sensor_f_pin_state):
            self.update_edge_sensors()
            self.motor_single_step(velocity)
            self.update_position(direction)


class PenAxis(Motor):
    def __init__(self, name, step_pin, direction_pin, edge_sensor_i_pin, edge_sensor_f_pin, pen_axis_resolution):
        Motor.__init__(self, name=name, step_pin=step_pin, direction_pin=direction_pin,
                       edge_sensor_i_pin=edge_sensor_i_pin, edge_sensor_f_pin=edge_sensor_f_pin,
                       resolution=pen_axis_resolution)
        self.arduino = serial.Serial('/dev/ttyACM0', 57600)

    def lift_pen(self):
        self.motor_for_loop(velocity=100, distance=50, direction='up')

    def draw(self, intensity):
        self.motor_while_loop(velocity=100, direction='down')
        self.motor_for_loop(velocity=100, distance=intensity, direction='down')

    def distance_from_page(self):
        return distance_control.get_stable_distance(min_value=0, max_value=1000,
                                                    test_amount=500, arduino=self.arduino,
                                                    stdev_limit=1.0)


class Gantry:
    def __init__(self):
        linear_axis_resolution = 1  # float(4 * 8 * math.tan(math.radians(360 / 200)))
        z_axis_resolution = 1
        self.x = Motor(name='x', step_pin=18, direction_pin=16,
                       edge_sensor_i_pin=37, edge_sensor_f_pin=13,
                       resolution=linear_axis_resolution)
        self.y = Motor(name='y', step_pin=38, direction_pin=40,
                       edge_sensor_i_pin=31, edge_sensor_f_pin=29,
                       resolution=linear_axis_resolution)
        self.pen = PenAxis(name='Pen', step_pin=8, direction_pin=10,
                           edge_sensor_i_pin=23, edge_sensor_f_pin=11,
                           pen_axis_resolution=z_axis_resolution)
        self.x_position = self.x.position
        self.y_position = self.y.position
        self.pen_position = self.pen.position

    def homing_sequence(self, padding):
        x_thread = threading.Thread(target=self.x.motor_while_loop, args=(400, 'backward',))
        y_thread = threading.Thread(target=self.y.motor_while_loop, args=(400, 'left',))
        self.pen.motor_while_loop(100, 'up')

        x_thread.start()
        y_thread.start()

        x_thread.join()
        y_thread.join()

        self.x.position = 0
        self.y.position = 0
        self.pen.position = self.pen.distance_from_page()

        self.x.motor_for_loop(100, padding, 'forward')
        self.y.motor_for_loop(100, padding, 'right')

        self.update_gantry_position()

        print(f'Pen is above ({self.x.position}, {self.y.position}), at height: {self.pen.position:.2f} [mm].')

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
            current_position = self.pen_position
            resolution = self.pen.resolution
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
        for step in range(steps):
            axis.update_edge_sensors()
            if (direction == 0 and axis.edge_sensor_i_pin_state) or (
                    direction == 1 and axis.edge_sensor_f_pin_state):
                axis.motor_single_step(velocity)
                axis.update_position(direction)
                if axis.name == 'x':
                    self.x_position = axis.position
                if axis.name == 'y':
                    self.y_position = axis.position
                if axis.name == 'z':
                    self.pen_position = axis.position
                # print(axis.edge_sensor_i_pin_state, axis.edge_sensor_f_pin_state)

        return f'{axis.name} axis done.'

    def path_sequence(self, next_position, velocity):
        x_steps_amount = self.calculate_steps('x', next_position)
        x_direction = self.calculate_direction(self.x_position, next_position['x'])

        y_steps_amount = self.calculate_steps('y', next_position)
        y_direction = self.calculate_direction(self.y_position, next_position['y'])

        x_thread = threading.Thread(target=self.gantry_for_loop,
                                    args=(velocity['x'], x_direction, x_steps_amount, self.x,))
        y_thread = threading.Thread(target=self.gantry_for_loop,
                                    args=(velocity['y'], y_direction, y_steps_amount, self.y,))

        x_thread.start()
        y_thread.start()

        x_thread.join()
        y_thread.join()

    def update_gantry_position(self):
        self.x_position = self.x.position
        self.y_position = self.y.position
        self.pen_position = self.pen.position


if __name__ == "__main__":
    gantry = Gantry()
    time.sleep(5)
    pad = 150
    gantry.homing_sequence(pad)
    x_steps = 2500
    y_steps = 3000
    step_delta = 300
    # Y steps amount - 3539
    # X steps amount - 2772
    # gantry.pen.motor_for_loop(velocity=100, distance=850, direction='down')
    while x_steps > step_delta and y_steps > step_delta:
        gantry.y.motor_for_loop(velocity=1000, distance=y_steps, direction='forward')
        print(f'Pen is above ({gantry.x.position}, {gantry.y.position}), '
              f'at height: {gantry.pen.distance_from_page():.2f} [mm].')
        gantry.x.motor_for_loop(velocity=1000, distance=x_steps, direction='right')
        print(f'Pen is above ({gantry.x.position}, {gantry.y.position}), '
              f'at height: {gantry.pen.distance_from_page():.2f} [mm].')
        x_steps -= step_delta
        y_steps -= step_delta
        gantry.y.motor_for_loop(velocity=1000, distance=y_steps, direction='backward')
        print(f'Pen is above ({gantry.x.position}, {gantry.y.position}), '
              f'at height: {gantry.pen.distance_from_page():.2f} [mm].')
        gantry.x.motor_for_loop(velocity=1000, distance=x_steps, direction='left')
        print(f'Pen is above ({gantry.x.position}, {gantry.y.position}), '
              f'at height: {gantry.pen.distance_from_page():.2f} [mm].')
        x_steps -= step_delta
        y_steps -= step_delta

        gantry.update_gantry_position()

        # print(f'X position: {gantry.x.position}')
        # print(f'Y position: {gantry.y.position}')
        # print(f'Z position: {gantry.pen.position}')
