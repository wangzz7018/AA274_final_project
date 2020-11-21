import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):

    # alpha = 2.0
    times = [] 
    x = [] 
    y = []
    t = 0
    for i in range(len(path)):
        if i > 0:
            # whoever originally wrote the line below forgot "/ V_des"
            t += np.linalg.norm(np.array(path[i]) - np.array(path[i - 1])) / V_des
        times.append(t)
        x.append(path[i][0])
        y.append(path[i][1])
    
    tck_x = scipy.interpolate.splrep(times, x, None, None, None, k=3, task=0, s=alpha)
    tck_y = scipy.interpolate.splrep(times, y, None, None, None, k=3, task=0, s=alpha)

    t = np.arange(0.0, times[-1], dt)
    
    x_smooth = scipy.interpolate.splev(t, tck_x, der=0)
    xd_smooth = scipy.interpolate.splev(t, tck_x, der=1)
    xdd_smooth = scipy.interpolate.splev(t, tck_x, der=2)
    y_smooth = scipy.interpolate.splev(t, tck_y, der=0)  
    yd_smooth = scipy.interpolate.splev(t, tck_y, der=1) 
    ydd_smooth = scipy.interpolate.splev(t, tck_y, der=2) 
    
    th = np.arctan2(yd_smooth, xd_smooth)
   
    t_smoothed = t
    traj_smoothed = np.array([x_smooth, y_smooth, th, xd_smooth, yd_smooth, xdd_smooth, ydd_smooth]).T
    return traj_smoothed, t_smoothed

