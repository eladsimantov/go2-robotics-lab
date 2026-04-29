import numpy as np

class MinJerk:
    def __init__(self, p0, pf, T, v0=None, a0=None, vf=None, af=None, dt=0.002, method='fast'):
        """
        Multivariate Minimum Jerk Trajectory Generator.
        Defaults: Zero velocity/acceleration at boundaries, dt = 0.002s, fast evaluation method.
        Parameters:
        - p0: Initial position (N-dimensional)
        - pf: Final position (N-dimensional)
        - T: Duration of the trajectory (seconds)
        - v0: Initial velocity (N-dimensional, default=0)
        - a0: Initial acceleration (N-dimensional, default=0)
        - vf: Final velocity (N-dimensional, default=0)
        - af: Final acceleration (N-dimensional, default=0)
        - dt: Time step for evaluation (default=0.002s)
        - method: 'fast' for Real Time (Horner's method), 'standard' for matrix multiplication. Default is 'fast'.

        Usage:
            traj = MinJerk(p0=[0, 0], pf=[10, 5], T=2.0)
            p, v, a = traj.eval(t=1.0)  # Evaluate at t=1 second
        
        """
        # Input normalization
        self.p0 = np.atleast_1d(p0)
        self.pf = np.atleast_1d(pf)
        self.T = T
        self.dt = dt
        self.method = method

        # Default boundary conditions to zero if not provided
        self.v0 = np.zeros_like(self.p0) if v0 is None else np.atleast_1d(v0)
        self.a0 = np.zeros_like(self.p0) if a0 is None else np.atleast_1d(a0)
        self.vf = np.zeros_like(self.p0) if vf is None else np.atleast_1d(vf)
        self.af = np.zeros_like(self.p0) if af is None else np.atleast_1d(af)

        # Validate dimensions before solving
        self._validate_dimensions()

        # Solve for coefficients: M * c = b
        b = np.array([self.p0, self.v0, self.a0, self.pf, self.vf, self.af])
        M = np.array([
            [0,      0,      0,     0,    0, 1],
            [0,      0,      0,     0,    1, 0],
            [0,      0,      0,     2,    0, 0],
            [T**5,   T**4,   T**3,  T**2, T, 1],
            [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [20*T**3, 12*T**2, 6*T, 2,    0, 0]
        ])
        # self.c shape is (6, N) where N is number of dimensions
        self.c = np.linalg.solve(M, b)

    def _validate_dimensions(self):
        """Validate that all inputs have consistent dimensions."""
        n_dims = np.size(self.p0)
        
        if np.size(self.pf) != n_dims:
            raise ValueError(f"End position dimension {np.size(self.pf)} doesn't match "
                           f"start position dimension {n_dims}.")
        
        if np.size(self.v0) != n_dims:
            raise ValueError(f"Initial velocity dimension {np.size(self.v0)} doesn't match "
                           f"position dimension {n_dims}.")
        
        if np.size(self.a0) != n_dims:
            raise ValueError(f"Initial acceleration dimension {np.size(self.a0)} doesn't match "
                           f"position dimension {n_dims}.")
        
        if np.size(self.vf) != n_dims:
            raise ValueError(f"Final velocity dimension {np.size(self.vf)} doesn't match "
                           f"position dimension {n_dims}.")
        
        if np.size(self.af) != n_dims:
            raise ValueError(f"Final acceleration dimension {np.size(self.af)} doesn't match "
                           f"position dimension {n_dims}.")
        

    def eval(self, t):
        """Evaluate trajectory at time t."""
        t = np.clip(t, 0, self.T)
        
        if self.method == 'fast':
            # Horner's Method: O(N) complexity, minimal operations
            p = ((((self.c[0]*t + self.c[1])*t + self.c[2])*t + self.c[3])*t + self.c[4])*t + self.c[5]
            v = (((5*self.c[0]*t + 4*self.c[1])*t + 3*self.c[2])*t + 2*self.c[3])*t + self.c[4]
            a = ((20*self.c[0]*t + 12*self.c[1])*t + 6*self.c[2])*t + 2*self.c[3]
        else:
            # Standard Matrix Method
            t_vec = np.array([t**5, t**4, t**3, t**2, t, 1])
            v_vec = np.array([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0])
            a_vec = np.array([20*t**3, 12*t**2, 6*t, 2, 0, 0])
            p, v, a = t_vec @ self.c, v_vec @ self.c, a_vec @ self.c
            
        return p, v, a

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # --- SIMULATION PARAMETERS ---
    start_pos = [0, 0, 0]    # 3D Trajectory (e.g., XYZ space)
    end_pos = [10, -5, 2]
    duration = 2.0        # seconds
    
    # Initialize class with defaults (v=0, a=0, dt=0.002)
    traj = MinJerk(start_pos, end_pos, duration, dt=0.002)  

    # --- SIMULATION LOOP ---
    times = np.arange(0, duration + traj.dt, traj.dt)
    results = {"p": [], "v": [], "a": []}

    for t in times:
        p, v, a = traj.eval(t)
        results["p"].append(p)
        results["v"].append(v)
        results["a"].append(a)

    # Convert to arrays for plotting
    pos_res = np.array(results["p"])
    vel_res = np.array(results["v"])
    acc_res = np.array(results["a"])

    # --- VISUALIZATION ---
    # Generic visualization: position, velocity, and acceleration profiles
    # Works with any N-dimensional trajectory
    
    n_dims = pos_res.shape[1]
    dim_labels = [f"Dim {i+1}" for i in range(n_dims)]
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 3*n_dims), sharex=True)
    
    # Position subplot
    for i in range(n_dims):
        axs[0].plot(times, pos_res[:, i], label=dim_labels[i])
    axs[0].set_ylabel("Position")
    axs[0].set_title("Minimum Jerk Trajectory (Multivariate)")
    axs[0].legend(loc='best')
    axs[0].grid(True, alpha=0.3)

    # Velocity subplot
    for i in range(n_dims):
        axs[1].plot(times, vel_res[:, i], label=dim_labels[i])
    axs[1].set_ylabel("Velocity")
    axs[1].legend(loc='best')
    axs[1].grid(True, alpha=0.3)

    # Acceleration subplot
    for i in range(n_dims):
        axs[2].plot(times, acc_res[:, i], label=dim_labels[i])
    axs[2].set_ylabel("Acceleration")
    axs[2].set_xlabel("Time (s)")
    axs[2].legend(loc='best')
    axs[2].grid(True, alpha=0.3)

    plt.tight_layout()
    print(f"Simulation finished. Dimensions: {n_dims}, dt: {traj.dt}s, Mode: {traj.method}")
    plt.show()