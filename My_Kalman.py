import numpy as np
from time import sleep
from datetime import datetime

class My_Kalman:
    
    def __init__(self, my_mpu):

        self.my_mpu = my_mpu # Note that the MPU return data in [m/s**2] for accelerometer and [deg/s] for gyro
        self.t_prev = datetime.now() # Time when object is initialised
        
        # We need to caliberate the MPU for errors

        calib = []

        print("Starting MPU caliberation..."); sleep(1)

        for i in range(100):

            xx = [self.my_mpu.get_accel_data()['y'], self.my_mpu.get_accel_data()['z'] - 9.8,
                  self.my_mpu.get_gyro_data()['x']]
            
            calib.append(xx)
            
        calib = np.array(calib) # Make is a 100*3 matrix
        
        self.error = np.mean(calib, axis = 0)
        
        print("MPU caliberated, the mean accY, accZ [m/s2] and gyro [deg/s] error is ", self.error[0], self.error[1], self.error[2])
        
        sleep(0.5)
        
        print('Calculating initial conditions for Kalman filter')
        
   
        R = [] # For initial values and sensor covariance matrix
        
        for i in range(100):
            
            f1 = np.arctan((self.my_mpu.get_accel_data()['y'] - self.error[0]) / (self.my_mpu.get_accel_data()['z'] - self.error[1]))
            f2 = np.deg2rad(np.self.my_mpu.get_gyro_data()['x'] - self.error[2]) # Gyro bias [rad] 
            
            R.append([f1,f2]) # 100 samples of angle and angular velocity (bias in gyro) in [rad] and [rad/s] respectively
            
        R = np.array(R); # Make it an array of 100*2
        
        init_conditions = np.transpose(np.mean(R, axis = 0)) # Get initial conditions in 2*1 form
        
        print('Calculated initial states for Kalman filter Acc angle, Gyro bias (constant inaccuracy) ', init_conditions)    
            
        self.B = np.array([[1], [0]]) # System B (input) matrix used for giving input (n_states * 1)        
        self.C = np.array([[1,0]]) # Matrix to map state values onto sensor values (n_sensors*n_states) 
        self.theta_prev = init_conditions[0] # Initial theta, used for getting angular velocity [rad]
        
        print('B and C matrices initialised, A matrix will be calulated while calculating angle')
        
        '''
        Now we need to initialise the following matrices for the Kalman filter to work
        1) Initial conditions X_0 (n_states*1)
        2) Error Covariance matrix P (n_states*n_states)
        3) Process noise covariance matrix Q which tells how noisy our process is (n_states*n_states)
        4) Sensor noise covariance matrix R which tells how noisy our sensors are (n_sensors*n_sensors)
        '''
        print('Initialising X_0, Q, P and R matrices')
        self.X_0 = init_conditions # These are the initial conditions (n_states * 1)
        self.P = np.random.rand(2,2)*np.eye(2) # Error covariance matrix initialised (n_states * n_states)
        self.Q = np.diag([0.001, 0.003]) # Process noise covariance matrix (n_states * n_states) contains variance (std**2 of both states)
        self.R = (np.std(R[:,0]))**2 # Sensor noise covariance matrix for accelerometer
        
        print('Initialised X_0, Q, P and R matrices, system ready')
        
        def get_angle(self, units = 'rad'):
            
            # Get the time elapsed first
            
            t_now = datetime.now()            
                        
            dt = (t_now - self.t_prev).total_seconds()# Total time difference in seconds
            
            # Step 1: Initialise A matrix and predict state. The system model is X_k = A*X_(k-1) + B*U(k)
            
            self.A = np.array([[1,-dt],[0,1]]) # System A matrix (n_states * n_states) for describing system dynamics
            
            U = np.deg2rad(np.self.my_mpu.get_gyro_data()['x'] - self.error[2]) # Input angular velocity in [rad/s]
            
            X_now = np.dot(self.A,self.X_0) + self.B*dt*U
            
            # Step 2: Project error covariance matrix, The process noise is added here and multiplied by dt 
            #as it has got added over time to the plant 
            
            self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q*dt
            
            # Step 3: Calculate Kalman gain and update measurements
            
            self.Kf = np.dot(np.dot(self.P, self.C.T),np.linalg.inv(np.dot(np.dot(self.C, self.P), self.C.T) + self.R)) 
            
            # Step 3a Take angle reading now
            
            Z = np.arctan((self.my_mpu.get_accel_data()['y'] - self.error[0]) / (self.my_mpu.get_accel_data()['z'] - self.error[1])) 
            
            # Step 3b Update the predicted values
            
            X_now = X_now + np.dot(self.Kf, Z - np.dot(self.C,X_now))
            
            # Step 3c Update Error covariance matrix
            
            self.P = np.dot((np.eye(2,2)-np.dot(self.Kf, self.C)), self.P)
            
            theta = X_now[0]; theta_dot = (theta - self.theta_prev)/dt
            
            # Save variables for next step
            
            self.X_0 = X_now # State prev = state now for next step calculation
            
            self.theta_prev = X_now[0] # make present angle equal to prev angle for next step calculation
            
            self.t_prev = t_now # Time now = time prev for next step
            
            if units == 'deg':
                dd = np.rad2deg([theta, theta_dot])
            else:
                dd = np.array([theta,theta_dot])
            
            return dd

        




        






