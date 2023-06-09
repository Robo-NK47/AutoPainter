import cv2
import matplotlib.pyplot as plt
import numpy as np


def resize_image(image, width=None, height=None):
    h, w = image.shape[:2]
    if width and height:
        new_size = (width, height)
    elif width:
        new_size = (width, int(height * (float(h) / w)))
    elif height:
        new_size = (int(width * (float(w) / h)), height)
    else:
        return image
    resized_img = cv2.resize(image, new_size)
    return resized_img


def quantize_image(image, levels):
    image = np.float32(image)
    pixel_values = image.reshape((-1, 3))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    flags = cv2.KMEANS_RANDOM_CENTERS
    _, labels, centers = cv2.kmeans(pixel_values, levels, None, criteria, 10, flags)
    centers = np.uint8(centers)
    quantized_values = centers[labels.flatten()]
    quantized_img = quantized_values.reshape(image.shape)
    quantized_img = np.uint8(quantized_img)
    return quantized_img


def convert_to_bw(image):
    bw_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return bw_img


def show_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    plt.imshow(image)
    plt.show()


def image_to_instructions(image, layers):
    instructions = np.zeros(image.shape)
    image_shape = image.shape
    layers = 255 / layers

    for i in range(image_shape[0]):
        for j in range(image_shape[1]):
            image_value = image[i][j]
            instructions[i][j] = int(image_value / layers)

    return instructions


def load_image(path):
    return cv2.imread(path)


def generate_contoures(_path):
    image = cv2.imread(_path)
    _, thresh = cv2.threshold(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 0, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


if __name__ == "__main__":
    img = load_image(r'/home/nk47/PycharmProjects/AutoPainter/potato.jpeg')
    black_and_white = convert_to_bw(resize_image(img, width=600, height=600))
    painting_instructions = image_to_instructions(black_and_white, 5)

    contours = generate_contoures(r'/home/nk47/PycharmProjects/AutoPainter/potato.jpeg')
    for contour in contours:
        points = np.squeeze(contour)

        print(f'G0 X{points[0][0]} Y{points[0][1]}\n'.encode())
        for point in points[1:]:
            print(f'G1 X{point[0]} Y{point[1]}\n'.encode())

    print('a')
