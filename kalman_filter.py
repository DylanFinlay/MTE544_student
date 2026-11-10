

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):

        self.P=np.array(P, dtype=float) # uncertainty in the state estimate
        self.Q=np.array(Q, dtype=float) # process noise
        self.R=np.array(R, dtype=float) # measurement noise
        self.x=np.array(x, dtype=float) #x, y,th, w, v,vdot
        self.dt=dt

    # TODO Part 3: Replace the matrices with Jacobians where needed
    def predict(self):

        # Linearize the motion at the previous estimate
        self.A=self.jacobian_A()
        self.motion_model()
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):

        # C maps state uncertainty into measurement uncertainty
        self.C=self.jacobian_H() # evaluated at the predicted state
        S=self.C @ self.P @ self.C.T + self.R

        # solve for gain 
        kalman_gain=np.linalg.solve(S, self.C @ self.P).T

        # correction 
        self.x=self.x + kalman_gain @ (z - self.measurement_model())
        self.x[2]=(self.x[2] + np.pi) % (2*np.pi) - np.pi
        self.P=(np.eye(6) - kalman_gain @ self.C) @ self.P
        self.P=(self.P + self.P.T)/2 # remove small roundoff asymmetry

    # TODO Part 3: Implement here the measurement model
    # expected sensor readings in the robot frame, ay is centripetal acceleration
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v, # v
            w, # w
            vdot, # ax
            v*w, # ay
        ])

    # TODO Part 3: Impelment the motion model (state-transition matrice)
    # euler integration, hold w and vdot constant over step
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

    # partial derivatives of the motion model wrt each state
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        return np.array([
            #x, y, th, w, v, vdot
            [1, 0, -v*np.sin(th)*dt,  0, np.cos(th)*dt,    0],
            [0, 1,  v*np.cos(th)*dt,  0, np.sin(th)*dt,    0],
            [0, 0,               1, dt,             0,    0],
            [0, 0,               0,  1,             0,    0],
            [0, 0,               0,  0,             1,   dt],
            [0, 0,               0,  0,             0,    1],
        ])

    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)
    # partial derivatives of h, d(v*w)/dw = v and d(v*w)/dv = w
    def jacobian_H(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            # x, y, th, w, v, vdot
            [0, 0,  0, 0, 1, 0], # v
            [0, 0,  0, 1, 0, 0], # w
            [0, 0,  0, 0, 0, 1], # ax
            [0, 0,  0, v, w, 0], # ay
        ])

    # TODO Part 3: return the states here
    # Return the current estimate for localization
    def get_states(self):
        return self.x
