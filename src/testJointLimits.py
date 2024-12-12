from src import NextState

def testJointLimits(j_limits, R_config, dt, v_lim, j_rate):

    n_s = NextState(R_config,j_rate,dt,v_lim)[3:8]
    b_limit = [False,False,False,False,False]

    for i in range(len(n_s)):
        if n_s[i] > j_limits[i][0] or n_s[i] < j_limits[i][0]:
            b_limit[i] = True
        elif n_s[i] < 0.001:
            b_limit[i] = True

    return b_limit










