import numpy as np
import math
class KalmanFilter:
    def __init__(self, init_state):
        self.x =  np.matrix([init_state]).T
        self.P = np.diag([10.0, 10.0, 1.0, 1.0, 1.0, 1.0])
    
    
    def step(self, measurements, dt):
        n = self.x.size
        I = np.eye(n)
        A = np.matrix([[1.0, 0.0, dt, 0.0, 1/2.0 * dt**2, 0.0],
                    [0.0, 1.0, 0.0, dt, 0.0, 1/2.0 * dt**2],
                    [0.0, 0.0, 1.0, 0.0, dt, 0.0],
                    [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        
        H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]])   
        sj = 0.1               
        Q = np.matrix([[(dt**6)/36, 0, (dt**5)/12, 0, (dt**4)/6, 0],
                    [0, (dt**6)/36, 0, (dt**5)/12, 0, (dt**4)/6],
                    [(dt**5)/12, 0, (dt**4)/4, 0, (dt**3)/2, 0],
                    [0, (dt**5)/12, 0, (dt**4)/4, 0, (dt**3)/2],
                    [(dt**4)/6, 0, (dt**3)/2, 0, (dt**2),0],
                    [0, (dt**4)/6, 0, (dt**3)/2, 0, (dt**2)]]) *sj**2 
                             

        rp = 10
        rv = 1
        R = np.matrix([[rp, 0.0, 0.0, 0.0],
                    [0.0, rp, 0.0, 0.0],
                    [0.0, 0.0, rv, 0.0],
                    [0.0, 0.0, 0.0, rv]])
                          
        # Time Update (Prediction)
        # ========================
        # Project the state ahead
        self.x = A * self.x
        
        # Project the error covariance ahead
        self.P = A * self.P * A.T + Q    
        
        
        # Measurement Update (Correction)
        # ===============================
        # Compute the Kalman Gain
        S = H * self.P * H.T + R
        K = (self.P * H.T) * np.linalg.pinv(S)

        
        # Update the estimate via z
        Z = measurements.reshape(H.shape[0],1)
        if not (math.isnan(Z[0]) or math.isnan(Z[1])):
            y = Z - (H * self.x)                            # Innovation or Residual
            self.x = self.x + (K * y) 
        
        # Update the error covariance
        self.P = (I - (K * H)) * self.P
        return self.x 