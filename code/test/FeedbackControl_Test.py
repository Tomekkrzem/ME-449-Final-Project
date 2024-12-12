from code.src import FeedbackControl, Velocities
import numpy as np

# Feedback Control Test Case
def main():
    # Example Current Configuration
    X = np.array([[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]])

    # Example Desired Configuration
    Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])

    # Example Next Desired Configuration
    Xd_next = np.array([[0,0,1,0.6], [0,1,0,0], [-1,0,0,0.3], [0,0,0,1]])

    # Input Error Integral
    X_err_sum = np.array([0,0,0,0,0,0],dtype='float64')

    # Proportional Gain
    Kp = 0

    # Integral Gain
    Ki = 0

    # Time Step
    dt = 0.01

    # Commanded End Effector Twist
    V = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt,X_err_sum)[0]

    # Screw Axis List in Body Frame
    B_list = np.array([[0,0,0,0,0],
                       [0,-1,-1,-1,0],
                       [1,0,0,0,1],
                       [0,-0.5076,-0.3526,-0.2176,0],
                       [0.033,0,0,0,0],
                       [0,0,0,0,0]])

    # Current Robot Configuration
    robot_conf = np.array([0,0,0,0,0,0.2,-1.6,0])

    # Home Configuration of End Effector Frame in Arm Base Frame
    M0e = np.array([[1,0,0,0.033],
                    [0,1,0,0],
                    [0,0,1,0.6546],
                    [0,0,0,1]])

    # Distance between Base Frame and Arm Base Frame
    Tb0 = np.array([[1,0,0,0.1662],
                    [0,1,0,0],
                    [0,0,1,0.0026],
                    [0,0,0,1]])

    l = 0.235
    w = 0.15
    r = 0.0475

    # Pseudo Inverse of H-Matrix (3x4 Matrix)
    F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1]])

    # Control Velocities
    uO = Velocities(robot_conf,B_list,V,M0e,Tb0,F).tolist()

    print(list(map(lambda x: round(x, 1), uO)))

if __name__ == "__main__":
    main()