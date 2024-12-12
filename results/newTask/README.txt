The Controller used in this Simulation was a Feedforward-Plus-PI (Proportional Integral) Controller.

Kp = 3
Ki = 0.1

Timestep:
dt = 0.01

Velocity Limits:
v_lim = 25

Robot Configuration:
R_config = [1.5, 0.8, 0.2, 0, 0, 0.3, -0.7, 0, 0, 0, 0, 0]

Initial Cube Configuration:
Tsc_i = np.array([[1, 0, 0, 1],
                  [0, 1, 0, 2],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])

Final Cube Configuration:
Tsc_f = np.array([[ 0, 1, 0, -2],
                  [-1, 0, 0, -0.25],
                  [ 0, 0, 1,  0.025],
                  [ 0, 0, 0,  1]])