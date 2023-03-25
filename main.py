import RPi.GPIO as GPIO
import axis_control
import image_processing
import time


if __name__ == "__main__":
    time.sleep(2)
    gantry = axis_control.Gantry()

    img = image_processing.load_image(r'/home/nk47/PycharmProjects/AutoPainter/potato.jpeg')
    resized = image_processing.resize_image(img, width=1300, height=1900)
    black_and_white = image_processing.convert_to_bw(resized)
    painting_instructions = image_processing.image_to_instructions(black_and_white, 5)

    gantry.homing_sequence()
    time.sleep(1)

    GPIO.cleanup()
