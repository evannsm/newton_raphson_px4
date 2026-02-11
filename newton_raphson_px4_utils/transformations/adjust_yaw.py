import numpy as np
def adjust_yaw(node, yaw: float) -> float:
    """Adjust yaw angle to account for full rotations and return the adjusted yaw.

    This function keeps track of the number of full rotations both clockwise and counterclockwise, and adjusts the yaw angle accordingly so that it reflects the absolute angle in radians. It ensures that the yaw angle is not wrapped around to the range of -pi to pi, but instead accumulates the full rotations.
    This is particularly useful for applications where the absolute orientation of the vehicle is important, such as in control algorithms or navigation systems.
    The function also initializes the first yaw value and keeps track of the previous yaw value to determine if a full rotation has occurred.

    Args:
        yaw (float): The yaw angle in radians from the motion capture system after being converted from quaternion to euler angles.

    Returns:
        psi (float): The adjusted yaw angle in radians, accounting for full rotations.
    """
    mocap_psi = yaw
    psi = None

    if not node.mocap_initialized:
        node.mocap_initialized = True
        node.prev_mocap_psi = mocap_psi
        psi = mocap_psi
        return psi

    # MoCap angles are from -pi to pi, whereas the angle state variable should be an absolute angle (i.e. no modulus wrt 2*pi)
    #   so we correct for this discrepancy here by keeping track of the number of full rotations.
    if node.prev_mocap_psi > np.pi*0.9 and mocap_psi < -np.pi*0.9:
        node.full_rotations += 1  # Crossed 180deg in the CCW direction from +ve to -ve rad value so we add 2pi to keep it the equivalent positive value
    elif node.prev_mocap_psi < -np.pi*0.9 and mocap_psi > np.pi*0.9:
        node.full_rotations -= 1 # Crossed 180deg in the CW direction from -ve to +ve rad value so we subtract 2pi to keep it the equivalent negative value

    psi = mocap_psi + 2*np.pi * node.full_rotations
    node.prev_mocap_psi = mocap_psi

    return psi
