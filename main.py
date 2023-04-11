import RPi.GPIO as GPIO
import axis_control
import json
import time


def open_json_file(file_path):
    with open(file_path, 'r') as f:
        _data = json.load(f)
    return _data


if __name__ == "__main__":
    time.sleep(2)

    # X steps amount - 3539
    # Y steps amount - 2772

    gantry = axis_control.Gantry()
    painting_commands = open_json_file(r'/home/nk47/PycharmProjects/AutoPainter/Test 2 - paths.json')

    pad = 539
    frame_x = 3539 - pad
    frame_y = 2772 - pad

    for i, painting_command in enumerate(painting_commands):
        for j, step in enumerate(painting_command):
            command = step['command']
            data = step['data']

            if command == 'meta_data':
                meta_data = data
                x_scale_factor = 2  # frame_x / meta_data['x_length']
                y_scale_factor = 2  # frame_y / meta_data['y_length']

            if command == 'home':
                gantry.homing_sequence(pad)

            if command == 'move':
                data = {'x': pad + (x_scale_factor * data['x']),
                        'y': pad + (y_scale_factor * data['y'])}
                gantry.path_sequence(next_position=data, velocity={'x': 80, 'y': 80})

            if command == 'drop':
                gantry.pen.draw(intensity=1)

            if command == 'lift':
                gantry.pen.lift_pen()

            if command == 'done':
                print('DONE!')
                exit(0)

            print(f'Step #{i}: command #{j}  -  command: {command}, data: {data}')

    time.sleep(1)

    GPIO.cleanup()
