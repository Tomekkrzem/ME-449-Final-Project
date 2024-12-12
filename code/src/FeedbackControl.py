import modern_robotics as mr
from numpy import matmul as m_mul
import numpy as np
from code.src import NextState

def FeedbackControl(X: np.ndarray, Xd: np.ndarray, Xd_next: np.ndarray, kp: float, ki: float,
                    dt: float, X_err_sum: np.ndarray) -> [np.ndarray,np.ndarray]:
    """
    :param X: Current Actual End Effector Configuration (Tse: se(3))
    :param Xd: Current Desired End Effector Configuration (Tse,d: se(3))
    :param Xd_next: Next Desired End Effector Configuration (Tse,d_next: se(3))
    :param kp: Proportional Gain Number
    :param ki: Integral Gain Number
    :param dt: The Timestep of the Simulation
    :param X_err_sum: Integral of Total Error

    :return: The Commanded End Effector Twist (V: SE(3))
    :return: The Error of the Current Timestep

    Example Input:
        X = np.array([[ 0.170,   0,   0.985,   0.387],
                      [     0,   1,       0,       0],
                      [-0.985,   0,   0.170,   0.570],
                      [     0,   0,       0,       1]])

        Xd = np.array([[ 0,   0,   1,   0.5],
                       [ 0,   1,   0,     0],
                       [-1,   0,   0,   0.5],
                       [ 0,   0,   0,     1]])

        Xd_next = np.array([[ 0,   0,   1,   0.6],
                            [ 0,   1,   0,     0],
                            [-1,   0,   0,   0.3],
                            [ 0,   0,   0,     1]])

        kp = 0

        ki = 0

        dt = 0.01

        X_err_sum = np.array([0,0,0,0,0,0])

    Output:
        np.array([0, 0, 0, 21.4, 0, 6.46])
        np.array([0, 0.00171, 0.000795, 0, 0.00107])
    """

    # Proportional Gain Matrix
    Kp = np.diag([kp, kp, kp, kp, kp, kp])

    # Integral Gain Matrix
    Ki = np.diag([ki, ki, ki, ki, ki, ki])

    # Error Twist
    X_err = mr.se3ToVec(mr.MatrixLog6(m_mul(mr.TransInv(X),Xd)))

    # Adjoint Representation of Desired Configuration in Actual Configuration
    AdjX_Xd = mr.Adjoint(m_mul(mr.TransInv(X),Xd))

    # Reference Twist
    Vd = mr.se3ToVec((1/dt) * mr.MatrixLog6(m_mul(mr.TransInv(Xd),Xd_next)))

    # Commanded End Effector Twist
    V = m_mul(AdjX_Xd, Vd) + m_mul(Kp,X_err) + m_mul(Ki,X_err_sum)

    # Integrating the Error
    X_err_sum += X_err * dt

    return [V, X_err]

def Velocities(R_Config: np.ndarray, B_list: np.ndarray, v: np.ndarray, M0e: np.ndarray,
               Tb0: np.ndarray, F: np.ndarray)-> np.ndarray:
    """
    :param R_Config: 12-Vector Representing Current Robot Configuration
    :param B_list: List of 5 Screw Axes in Base Frame
    :param v: Commanded End Effector Twist (v: SE(3))
    :param M0e: Home Configuration of End Effector Frame in Arm Base Frame (M0e: se(3))
    :param Tb0: Distance between Base Frame and Arm Base Frame (Tb0: se(3))
    :param F: Pseudo Inverse of H-Matrix (3x4 Matrix)

    :return: 9-Vector Representing Current Control Velocities

    Example Input:
        R_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])

        B_list = B_list = np.array([[0,0,0,0,0],
                                    [0,-1,-1,-1,0],
                                    [1,0,0,0,1],
                                    [0,-0.5076,-0.3526,-0.2176,0],
                                    [0.033,0,0,0,0],
                                    [0,0,0,0,0]])

        v = np.array([0, 0, 0, 21.4, 0, 6.46)

        M0e = np.array([[1,0,0,0.033],
                        [0,1,0,0],
                        [0,0,1,0.6546],
                        [0,0,0,1]])

        Tb0 = np.array([[1,0,0,0.1662],
                        [0,1,0,0],
                        [0,0,1,0.0026],
                        [0,0,0,1]])

        F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1]])

    Output:
        np.array([157.2, 157.2, 157.2, 157.2, 0, -652.9, 1398.6, -745.7, 0)
    """
    # End Effector frame in the Arm Base Frame
    T0e = mr.FKinBody(M0e, B_list, R_Config[3:8])

    # Vector of 6 Zeros
    z_vec = np.zeros(len(np.transpose(F).tolist()))

    # F6 Matrix
    F6 = np.array([z_vec, z_vec, *F.tolist(), z_vec])

    # Arm Jacobian
    J_arm = mr.JacobianBody(B_list, R_Config[3:8])

    # Base Jacobian
    J_base = np.matmul(mr.Adjoint(np.matmul(mr.TransInv(T0e), mr.TransInv(Tb0))), F6)

    # End Effector Jacobian
    J_e = np.transpose(np.array([*np.transpose(J_base).tolist(), *np.transpose(J_arm).tolist()]))

    # Pseudo Jacobian
    J_e_pseudo = np.linalg.pinv(J_e)

    # Control Velocities
    Vel =  np.matmul(J_e_pseudo, v)

    return Vel


def testJointLimits(j_limits, R_config, dt, v_lim, j_rate):

    n_s = NextState(R_config,j_rate,dt,v_lim)[3:8]
    b_limit = [False,False,False,False,False]

    for i in range(len(n_s)):
        if j_limits[i] is None:
            b_limit[i] = False
            break
        if n_s[i] > j_limits[i][0] or n_s[i] < j_limits[i][0]:
            b_limit[i] = True
        elif n_s[i] < 0.001:
            b_limit[i] = True

    return b_limit