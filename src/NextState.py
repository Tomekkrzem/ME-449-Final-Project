import modern_robotics as mr
import numpy as np
from numpy import sin, cos

def NextState(curr_config: np.ndarray, rate_vect: np.ndarray, dt: float, v_lim: int) -> np.ndarray:
    """
    :param curr_config: 12-Vector Representation of the Current Robot Configuration
        [0:3]: Chassis Configuration Variables (phi, x, y)
        [3:8]: Arm Configuration Variables (theta)
        [8:12]: Wheel Configuration Variables (wheel_ang)
    :param rate_vect: 9-Vector Representation of Current Wheel and Joint Speeds
        [0:4]: Wheel Speeds (u)
        [4:8] Joint Angle Speeds (theta_dot)
    :param dt: The Timestep of the Simulation
    :param v_lim: The Joint Velocity Limit

    :return: next_state: 12-Vector Representation of the Next State Robot Configuration

    Example Input:
        curr_config = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
        rate_vect = np.array([10,10,10,10,0,0,0,0,0])
        dt = 0.01
        v_lim = 10

    Output:
        np.array([0.0, 0.00475, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1])
    """
    # Chassis Configuration
    curr_chassis_conf = curr_config[0:3]

    # Joint Angles and Joint Speeds
    curr_arm_conf, curr_arm_speed = [curr_config[3:8], rate_vect[4:]]

    # Wheel Angles and Wheel Speeds
    curr_wheel_conf, curr_wheel_speed = [curr_config[8:], rate_vect[0:4]]

    # Velocity Control. The Lambda Function Ensures Wheel Speeds are Within Range
    v_control = lambda s_lst, v: np.array([v if s > v else (-v if s < -v else s) for s in s_lst])
    curr_wheel_speed = v_control(curr_wheel_speed,v_lim)
    curr_arm_speed = v_control(curr_arm_speed,v_lim)

    # Chassis Configuration Variables
    phi, x, y = curr_chassis_conf

    # Initialize F for Odometry Calculation
    l = 0.235
    w = 0.15
    r = 0.0475

    F = (r/4) *  np.array([[-1/(l+w), 1/(l+w), 1/(l+w),-1/(l+w)],
                           [1, 1, 1, 1],
                           [-1, 1, -1, 1]])


    # CALCULATING NEXT STATE OF ANGLES, SPEEDS, and ACCELERATIONS
    # Calculating Joint Angles of Next State
    next_arm_conf = curr_arm_conf + curr_arm_speed * dt

    # Calculating Wheel Angles of Next State
    next_wheel_conf = curr_wheel_conf + curr_wheel_speed * dt

    # Calculating Change in Wheel Angle
    delta_wheel_conf = curr_wheel_speed * dt

    # Calculating the Body Twist
    Vb = np.matmul(F, delta_wheel_conf)


    wbz, xb, yb = Vb

    if wbz == 0:
        dt_qb = np.array([0, xb, yb])
    else:
        dt_qb = np.array([wbz,
                          (xb*sin(wbz) + yb*(cos(wbz)-1))/wbz,
                          (yb*sin(wbz) + xb*(1-cos(wbz)))/wbz])

    chassis_rot = np.array([[1,          0,           0],
                            [0,   cos(phi),   -sin(phi)],
                            [0,   sin(phi),    cos(phi)]])

    delta_chassis_conf = np.matmul(chassis_rot,dt_qb)


    # Determining Next Chassis Configuration Variables
    next_chassis_conf = curr_chassis_conf + delta_chassis_conf


    # Creating 12-Vector Representing the Next Configuration
    next_state = np.array([*next_chassis_conf.tolist(),
                  *next_arm_conf.tolist(),
                  *next_wheel_conf.tolist()])

    return next_state

