import numpy as np

def pose_integration(vx, vy, wz, theta, dt):
    """
    Integrates computed chassis velocities over dynamic time
    steps (dt) to continuously update global pose coordinates (X, Y,
    theta).

    Parameters:
    vx : x component of the velocity
    vy : y component of the velocity
    wz : angular velocity around the z-axis
    theta : current angle of rotation in radians
    dt : time step for integration
    """
    X_dot = vx * np.cos(theta) - vy * np.sin(theta)
    Y_dot = vx * np.sin(theta) + vy * np.cos(theta)

    X += X_dot * dt
    Y += Y_dot * dt
    theta += wz * dt

    return X, Y, theta
