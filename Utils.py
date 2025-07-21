def remap(x: float, min_x: float, max_x: float, min_y: float, max_y: float) -> float:
    """Remaps the value 'x' (ranging from 'min_x' to 'max_x') to a value ranging from 'min_y' to 'max_y'.
    Arguments:
        x {float}: the input value to be remapped.
        min_x {float} -- the minimum value 'x' can take on.
        max_x {float} -- the maximum value 'x' can take on.
        min_y {float} -- the minimum value the output can take on.
        max_y {float} -- the maximum value the output can take on.
    Returns:
        float: the remapped value ranging from 'min_y' to 'max_y'.
    Raises:
        ValueError: if max_x == min_x (division by zero)
    """
    if max_x == min_x:
        raise ValueError("max_x and min_x cannot be equal in remap()")
    return min_y + (max_y - min_y) * ((x - min_x) / (max_x - min_x))

def clamp(x: float, min_x: float, max_x: float) -> float:
    """Clamps 'x' to 'min_x' on the bottom and 'max_x' on the top.
    Arguments:
        x {float} -- the value to clamp.
        min_x {float} -- the floor of the clamp.
        max_x {float} -- the ceiling of the clamp.
    Returns:
        float -- The new clamped value.
    """
    return max(min_x, min(x, max_x))

def signum(x: float) -> float:
    """Returns the sign of 'x'.
    Arguments:
        x {float} -- The value to be tested.
    Returns:
        float -- Returns +1.0 if 'x' > 0, -1.0 if 'x' < 0, and 0.0 if 'x' == 0.
    """
    if x < 0:
        return -1.0
    elif x > 0:
        return 1.0
    return 0.0
