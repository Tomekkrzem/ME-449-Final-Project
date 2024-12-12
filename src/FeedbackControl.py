import modern_robotics as mr
from numpy import matmul as m_mul
import numpy as np
from src import testJointLimits

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
    :return: The Integral of Total Error

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
    X_err_sum = X_err_sum + X_err * dt

    return [V, X_err_sum]

def Velocities(R_Config, B_list, v, M0e, Tb0, F, lim, dt, vlim):

    T0e = mr.FKinBody(M0e, B_list, R_Config[3:])

    z_vec = np.zeros(len(np.transpose(F).tolist()))

    F6 = np.array([z_vec, z_vec, *F.tolist(), z_vec])

    J_arm = mr.JacobianBody(B_list, R_Config[3:8])

    J_base = np.matmul(mr.Adjoint(np.matmul(mr.TransInv(T0e), mr.TransInv(Tb0))), F6)

    J_e = np.transpose(np.array([*np.transpose(J_base).tolist(), *np.transpose(J_arm).tolist()]))

    J_e_pseudo = np.linalg.pinv(J_e)

    Vel =  np.matmul(J_e_pseudo, v)

    if lim is None:
        return Vel
    else:
        testJointLimits(lim,R_Config,dt,vlim,Vel)