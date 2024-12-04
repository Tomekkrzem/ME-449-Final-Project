import modern_robotics as mr
import numpy as np
from numpy import sin, cos

def NextState(curr_config, rate_vect, dt, v_lim):
    # INITIALIZATION OF NECESSARY COMPONENTS
    # Initializes Chassis Configuration
    curr_chassis_conf = curr_config[0:3]

    # Initializes Joint Angles and Joint Speeds
    curr_arm_conf, curr_arm_speed = [curr_config[3:8], rate_vect[4:]]
    # Initializes Wheel Angles and Wheel Speeds
    curr_wheel_conf, curr_wheel_speed = [curr_config[8:], rate_vect[0:4]]

    # Velocity Control. The Lambda Function Ensures Wheel Speeds are Within Range
    v_control = lambda s_lst, v: np.array([v if s > v else (-v if s < -v else s) for s in s_lst])
    curr_wheel_speed = v_control(curr_wheel_speed,v_lim)
    curr_arm_speed = v_control(curr_arm_speed,v_lim)


    # Chassis Configuration Variables
    phi, x, y = curr_chassis_conf

    # Configuration of Mobile Base in Space Frame
    Tsb = np.array([[cos(phi), -sin(phi), 0, x],
                    [sin(phi), cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]])

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
    delta_wheel_conf = next_wheel_conf - curr_wheel_conf

    # Calculating the Body Twist
    body_twist = np.matmul(F, delta_wheel_conf)

    # Calculating the Planar Twist
    planar_twist = np.array([0,0,*body_twist,0])

    # Calculating the Next Body Frame Relative to Initial Body Frame
    Tbb_prime = mr.MatrixExp6(mr.VecTose3(planar_twist))

    # Calculating the Next Body Frame Relative to the Space Frame
    Tsb_prime = np.matmul(Tsb,Tbb_prime)

    # Calculating the Spatial Twist
    Vsb = mr.se3ToVec(mr.MatrixLog6(Tsb_prime))

    # Determining Next Chassis Configuration Variables
    next_chassis_conf = Vsb[2:5]

    # Creating 12-Vector Representing the Next Configuration
    next_state = [*next_chassis_conf.tolist(),
                  *next_arm_conf.tolist(),
                  *next_wheel_conf.tolist()]


    return next_state


