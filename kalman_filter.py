

import numpy as np


# TODO Part 3: Comment the code explaining each part
# State vector (6x1): x = [x, y, theta, w, v, vdot]
# Measurement vector (4x1): z = [v, w, ax, ay]
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P, Q, R, x, dt):
        # P (6x6): state estimate covariance
        # Q (6x6): process noise covariance
        # R (4x4): measurement noise covariance
        # x (6x1): initial state vector [x, y, theta, w, v, vdot]
        # dt: time step for Euler integration
        self.P = np.array(P, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.x = np.array(x, dtype=float)
        self.dt = dt

    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):
        # A (6x6): process model Jacobian (df/dx) evaluated at current state
        self.A = self.jacobian_A()
        
        self.motion_model()
        
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):
        # C (4x6): measurement model Jacobian (dh/dx) evaluated at predicted state
        self.C = self.jacobian_H()
        
        # S (4x4): innovation covariance S = C*P*C' + R
        S = self.C @ self.P @ self.C.T + self.R

        # K (6x4): Kalman gain K = P*C'*inv(S)
        kalman_gain = self.P @ self.C.T @ np.linalg.inv(S)

        # Correct state estimate: x = x + K * (z - h(x))
        self.x = self.x + kalman_gain @ (z - self.measurement_model())
        self.x[2] = (self.x[2] + np.pi) % (2*np.pi) - np.pi
        
        # Correct covariance: P = (I - K*C)*P
        self.P = (np.eye(6) - kalman_gain @ self.C) @ self.P
        self.P = (self.P + self.P.T) / 2  # maintain symmetry

    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,     # v
            w,     # w
            vdot,  # ax
            v*w,   # ay: centripetal acceleration in robot frame
        ])

    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v + vdot*dt,
            vdot,
        ])

    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        return np.array([
            #x, y, th,                 w,  v,              vdot
            [1, 0, -v*np.sin(th)*dt,  0,  np.cos(th)*dt, 0],
            [0, 1,  v*np.cos(th)*dt,  0,  np.sin(th)*dt, 0],
            [0, 0,  1,                dt, 0,             0],
            [0, 0,  0,                1,  0,             0],
            [0, 0,  0,                0,  1,             dt],
            [0, 0,  0,                0,  0,             1],
        ])

    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            # x, y, th, w, v, vdot
            [0,  0, 0,  0, 1, 0], # v
            [0,  0, 0,  1, 0, 0], # w
            [0,  0, 0,  0, 0, 1], # ax
            [0,  0, 0,  v, w, 0], # ay: d(v*w)/dw = v, d(v*w)/dv = w
        ])

    # TODO Part 3: return the states here    
    def get_states(self):
        return self.x
