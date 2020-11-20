import numpy as np

EPSILON_OMEGA = 1e-3

def compute_dynamics(xvec, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to xvec.
        Gu: np.array[3,2] - Jacobian of g with respect to u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    # HINT: To compute the new state g, you will need to integrate the dynamics of x, y, theta
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.
    x, y, theta = xvec
    V, om = u
    if abs(om) >= EPSILON_OMEGA:
        x += V * (np.sin(theta + om * dt) - np.sin(theta)) / om
        y += V * (np.cos(theta) - np.cos(theta + om * dt)) / om
        Gx = np.array([[1, 0, V * (np.cos(theta + om * dt) - np.cos(theta)) / om],
                       [0, 1, V * (np.sin(theta + om * dt) - np.sin(theta)) / om],
                       [0, 0, 1]])
        Gu = np.array([[(np.sin(theta + om * dt) - np.sin(theta)) / om,
                        -V / om ** 2 * (np.sin(theta + om * dt) - np.sin(theta)) + V / om * dt * np.cos(
                            theta + om * dt)],
                       [(-np.cos(theta + om * dt) + np.cos(theta)) / om,
                        V / om ** 2 * (np.cos(theta + om * dt) - np.cos(theta)) + V / om * dt * np.sin(
                            theta + om * dt)],
                       [0, dt]])
    else:
        x += 0.5 * V * (np.cos(theta + om * dt) + np.cos(theta)) * dt
        y += 0.5 * V * (np.sin(theta + om * dt) + np.sin(theta)) * dt
        Gx = np.array([[1, 0, 0.5 * V * dt * (-np.sin(theta + om * dt) - np.sin(theta))],
                       [0, 1, 0.5 * V * dt * (np.cos(theta + om * dt) + np.cos(theta))],
                       [0, 0, 1]])
        Gu = np.array(
            [[0.5 * dt * (np.cos(theta + om * dt) + np.cos(theta)), 0.5 * V * dt ** 2 * (-np.sin(theta + om * dt))],
             [0.5 * dt * (np.sin(theta + om * dt) + np.sin(theta)), 0.5 * V * dt ** 2 * (np.cos(theta + om * dt))],
             [0, dt]])
    theta += om * dt
    g = np.array([x, y, theta])
    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    # HINT: Calculate the pose of the camera in the world frame (x_cam, y_cam, th_cam), a rotation matrix may be useful.
    # HINT: To compute line parameters in the camera frame h = (alpha_in_cam, r_in_cam), 
    #       draw a diagram with a line parameterized by (alpha,r) in the world frame and 
    #       a camera frame with origin at x_cam, y_cam rotated by th_cam wrt to the world frame
    # HINT: What is the projection of the camera location (x_cam, y_cam) on the line r? 
    # HINT: To find Hx, write h in terms of the pose of the base in world frame (x_base, y_base, th_base)
    x_w, y_w, theta_w = x
    x_cam, y_cam, theta_cam = tf_base_to_camera
    R = np.array([[np.cos(theta_w), -np.sin(theta_w)], [np.sin(theta_w), np.cos(theta_w)]])
    x_cam_w, y_cam_w = R.dot(tf_base_to_camera[0:2]) + x[0:2]  # 2*1 camera in the world frame
    theta_cam_w = theta_w + theta_cam
    h = np.array([alpha - theta_cam_w, r - x_cam_w * np.cos(alpha) - y_cam_w * np.sin(alpha)])
    Hx = np.array([[0, 0, -1], [-np.cos(alpha), -np.sin(alpha),
                                x_cam * np.sin(theta_w - alpha) + y_cam * np.cos(theta_w - alpha)]])
    ########## Code ends here ##########

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h

