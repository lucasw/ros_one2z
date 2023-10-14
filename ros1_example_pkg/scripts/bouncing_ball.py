import cv2
import numpy as np
from geometry_msgs.msg import (
    Point32,
    PolygonStamped,
)
from std_msgs.msg import Header


def image_to_contour(header: Header, image_np: np.ndarray) -> PolygonStamped:
    if len(image_np.shape) > 2:
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

    contours, hierarchy = cv2.findContours(image_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    polygon = PolygonStamped()
    polygon.header = header

    for contour in contours:
        for i in range(contour.shape[0]):
            sc = 0.003
            x = contour[i, 0, 0] * sc
            y = contour[i, 0, 1] * sc
            pt = Point32(x=x, y=y)
            polygon.polygon.points.append(pt)
    return polygon


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
