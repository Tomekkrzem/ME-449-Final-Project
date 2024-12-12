import numpy as np
from .init_state import conf

def conf_t():
    # Initial Cube Configuration
    Tsc_i = np.array([[1, 0, 0, 1],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    # Final Cube Configuration
    Tsc_f = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    # End Effector Home Configuration in Space Frame
    Mse = conf()[0]

    # End Effector Angle
    ee_ang = np.sin(np.pi / 4)

    # Configuration of End Effector When Grasping
    T_grasp = np.array([[-ee_ang, 0, ee_ang, 0],
                        [0, 1, 0, 0],
                        [-ee_ang, 0, -ee_ang, 0],
                        [0, 0, 0, 1]])

    # Configuration of End Effector at Standoff
    T_standoff = np.array([[-ee_ang, 0, ee_ang, 0],
                           [0, 1, 0, 0],
                           [-ee_ang, 0, -ee_ang, 0.3],
                           [0, 0, 0, 1]])

    return [Mse, Tsc_i, Tsc_f, T_grasp, T_standoff,1]

