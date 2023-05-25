import RPi.GPIO as GPIO
import axis_control
import json
import time


def open_json_file(file_path):
    with open(file_path, 'r') as f:
        _data = json.load(f)
    return _data


def draw_a_file(file_path, x_offset, y_offset, scale, gantry):
    time.sleep(2)

    # X steps amount - 2772
    # Y steps amount - 3539

    painting_commands = open_json_file(file_path)

    pad = 100
    frame_x = 2772 - pad
    frame_y = 3539 - pad

    for i, painting_command in enumerate(painting_commands):
        for j, step in enumerate(painting_command):
            command = step['command']
            data = step['data']
            print(f'Step #{i}: command #{j}  -  command: {command}, data: {data}')
            if command == 'meta_data':
                meta_data = data
                scale_factor = scale  # max(frame_x / meta_data['y_length'], 0.15 * frame_y / meta_data['x_length'])
                print(f' Scale factor: {scale_factor:.2f}')
                x_scale_factor = scale_factor
                y_scale_factor = scale_factor

            if command == 'start':
                gantry.homing_sequence(0)
                gantry.homing_sequence(pad)
                gantry.pen.go_to_hover_position()

            if command == 'move':
                data = {'x': pad + (x_scale_factor * data['x'] + x_offset),
                        'y': pad + (y_scale_factor * data['y'] + y_offset)}
                gantry.path_sequence(next_position=data, velocity={'x': 400, 'y': 400})

            if command == 'drop':
                gantry.pen.draw(intensity=0)

            if command == 'lift':
                gantry.pen.motor_for_loop(1000, 1200, 'up')

            if command == 'done':
                print('DONE!')



if __name__ == "__main__":
    files_to_draw = [{'file_path': r'/home/nk47/PycharmProjects/AutoPainter/Paths/drawing - paths.json',
                      'x_offset': 150,
                      'y_offset': 400,
                      'scale': 1.1},
                     {'file_path': r'/home/nk47/PycharmProjects/AutoPainter/Paths/Hello world - paths.json',
                      'x_offset': 150,
                      'y_offset': 1000,
                      'scale': 13}]
    gantry = axis_control.Gantry()

    if input('Enter "y" if you would like to change the writing tool: ') == 'y':
        gantry.replace_tool()

    for file in files_to_draw:
        draw_a_file(file_path=file['file_path'],
                    x_offset=file['x_offset'],
                    y_offset=file['y_offset'],
                    scale=file['scale'],
                    gantry=gantry)

    gantry.replace_tool()
    GPIO.cleanup()
    time.sleep(1)
    exit(0)
