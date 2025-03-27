import math

def wrap_angle(angle: float) -> float:
    """
    Wraps an angle to the range [-π, π].

    :param angle: The input angle in radians.
    :return: The wrapped angle in radians.
    """
    wrapped_angle = angle
    while wrapped_angle > math.pi:
        wrapped_angle -= 2 * math.pi
    while wrapped_angle < -math.pi:
        wrapped_angle += 2 * math.pi
    return wrapped_angle