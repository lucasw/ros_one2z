import cv2
import numpy as np


# TODO(lucasw) how to move this into location that both ros1 and ros2 can use?
def update_value(value: float, velocity: float, min_value: float, max_value: float, margin: float):
    """
    bounce when reaching limit
    """
    value += velocity

    min_value = min_value + margin
    if value <= min_value:
        value = min_value
        velocity *= -1

    max_value = max_value - margin
    if value >= max_value:
        value = max_value
        velocity *= -1

    return value, velocity


class BouncingBall:
    def __init__(self):
        self.width = 2048
        self.height = 1024
        self.radius = 64
        self.x = 100
        self.y = 120
        self.vel_x = 14
        self.vel_y = 30

    def update(self) -> np.array:
        image_np = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        cv2.circle(image_np, (self.x, self.y), radius=self.radius,
                   color=(200, 190, 180), thickness=-1)
        self.x, self.vel_x = update_value(self.x, self.vel_x, 0, self.width, self.radius)
        self.y, self.vel_y = update_value(self.y, self.vel_y, 0, self.height, self.radius)
        return image_np
