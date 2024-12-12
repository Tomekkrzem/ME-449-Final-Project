import numpy as np
import modern_robotics as mr

def conf():
    # Body Frame in Space Frame
    Tsb = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0.0963],
                    [0,0,0,1]])

    # Arm Base Frame in Body Frame
    Tb0 = np.array([[1,0,0,0.1662],
                    [0,1,0,0],
                    [0,0,1,0.0026],
                    [0,0,0,1]])

    # End Effector Home Configuration in Arm Base Frame
    M0e = np.array([[1,0,0,0.033],
                    [0,1,0,0],
                    [0,0,1,0.6546],
                    [0,0,0,1]])

    # Screw Axes for the Five Joints
    B_list = np.array([[0, 0, 0, 0, 0],
                       [0, -1, -1, -1, 0],
                       [1, 0, 0, 0, 1],
                       [0, -0.5076, -0.3526, -0.2176, 0],
                       [0.033, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0]])

    # Length, Width, and Wheel Radius
    l = 0.235
    w = 0.15
    r = 0.0475

    # F Matrix for Odometry
    F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1]])


    # End Effector Home Configuration in Space Frame
    Mse = np.matmul(Tsb, np.matmul(Tb0, M0e))

    # End Effector Configuration in Space Frame
    Tse = np.array([[0,0,1,0],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]])

    # Initial Robot Configuration
    R_config = np.array([0, -0.4, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0])

    # Time Step
    dt = 0.01

    return [Tse, M0e, Tb0, B_list, F, R_config, dt]
