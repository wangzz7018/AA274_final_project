import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
	"""
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
	"""
	########## Code starts here ##########
	x, y = list(zip(*path))
	x_displacement = np.array(x[1:]) - np.array(x[:-1])
	y_displacement = np.array(y[1:]) - np.array(y[:-1])

	times = np.zeros(len(x_displacement) + 1)
	for idx in range(1, len(x_displacement) + 1):
		times[idx] = times[idx-1] + np.sqrt(x_displacement[idx-1]**2 + y_displacement[idx-1]**2) / V_des

	x_tck = scipy.interpolate.splrep(times, x, s=alpha)
	y_tck = scipy.interpolate.splrep(times, y, s=alpha)
	t_smoothed = np.arange(0, times[-1], dt)

	traj_smoothed_x = scipy.interpolate.splev(t_smoothed, x_tck)
	traj_smoothed_y = scipy.interpolate.splev(t_smoothed, y_tck)
	traj_smoothed_xdot = scipy.interpolate.splev(t_smoothed, x_tck, der=1)
	traj_smoothed_ydot = scipy.interpolate.splev(t_smoothed, y_tck, der=1)
	traj_smoothed_xddot = scipy.interpolate.splev(t_smoothed, x_tck, der=2)
	traj_smoothed_yddot = scipy.interpolate.splev(t_smoothed, y_tck, der=2)
	traj_smoothed_theta = np.arctan2(traj_smoothed_ydot, traj_smoothed_xdot)
	traj_smoothed = np.vstack([traj_smoothed_x, 
		                       traj_smoothed_y,
		                      traj_smoothed_theta,
		                      traj_smoothed_xdot,
		                      traj_smoothed_ydot,
		                      traj_smoothed_xddot,
		                      traj_smoothed_yddot]).T
	########## Code ends here ##########

	return traj_smoothed, t_smoothed


