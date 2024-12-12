import numpy as np

# State Configuration for Best
def conf_best():
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

    # End Effector Configuration in Space Frame
    Tse = np.array([[0,0,1,0],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]])

    # Initial Robot Configuration
    R_config = np.array([0.4, -0.4, -0.2, 0, 0, 1, -1.8, 0, 0, 0, 0, 0])

    # Time Step
    dt = 0.01

    # Proportional Gain Value
    Kp = 30
    # Integral Gain Value
    Ki = 0.2

    # Array to Store Integral of the Error
    err_arr = []

    # Velocity Limit
    v_lim = 15

    return [Tse, M0e, Tb0, B_list, F, R_config, dt, Kp, Ki, err_arr, v_lim]

# State Configuration for Overshoot
def conf_overshoot():
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

    # End Effector Configuration in Space Frame
    Tse = np.array([[0,0,1,0],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]])

    # Initial Robot Configuration
    R_config = np.array([0.6, 0.4, -0.5, 0, 0, 0.2, -2, 0, 0, 0, 0, 0])

    # Time Step
    dt = 0.01

    # Proportional Gain Value
    Kp = 3
    # Integral Gain Value
    Ki = 3.1

    # Array to Store Integral of the Error
    err_arr = []

    # Velocity Limit
    v_lim = 25

    return [Tse, M0e, Tb0, B_list, F, R_config, dt, Kp, Ki, err_arr, v_lim]

# State Configuration for New Task
def conf_newTask():
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

    # End Effector Configuration in Space Frame
    Tse = np.array([[0,-1,0,1],
                    [1,0,0,0.5],
                    [0,0,1,0.6],
                    [0,0,0,1]])

    # Initial Robot Configuration
    R_config = np.array([1.5, 0.7, -0.5, 0, 0, 0.3, -0.7, 0, 0, 0, 0, 0])

    # Time Step
    dt = 0.01

    # Proportional Gain Value
    Kp = 5
    # Integral Gain Value
    Ki = 0.1

    # Array to Store Integral of the Error
    err_arr = []

    # Velocity Limit
    v_lim = 25

    return [Tse, M0e, Tb0, B_list, F, R_config, dt, Kp, Ki, err_arr, v_lim]