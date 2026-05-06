"""Python Kalman filter: 9-state constant-acceleration model in 3D.

State: [x, y, z, vx, vy, vz, ax, ay, az]
Extends the original 2D (6-state) version to three dimensions.
"""

import numpy as np


class KalmanFilterCA:
    """Constant-acceleration Kalman filter (9 states, 3D)."""

    def __init__(self, dt=0.5, eP=0.25, eQ_pos=0.01, eQ_vel=0.05, eQ_acc=0.05,
                 eR_pos=0.04, eR_vel=0.3, eR_acc=0.6, avg_frames=5):
        self.dt = dt
        self.avg_frames = avg_frames

        # State transition matrix A (9x9)
        # State: [x, y, z, vx, vy, vz, ax, ay, az]
        dt2 = 0.5 * dt * dt
        self.A = np.eye(9)
        # position += velocity*dt + 0.5*acceleration*dt^2
        for i in range(3):
            self.A[i, i + 3] = dt       # pos <- vel
            self.A[i, i + 6] = dt2      # pos <- acc
            self.A[i + 3, i + 6] = dt   # vel <- acc

        self.B = np.zeros((9, 9))
        self.H = np.eye(9)

        self.P_init = np.eye(9) * eP
        self.Q = np.diag([eQ_pos]*3 + [eQ_vel]*3 + [eQ_acc]*3)
        self.R = np.diag([eR_pos]*3 + [eR_vel]*3 + [eR_acc]*3)

        # Current state
        self.states = None
        self.P = None
        self.is_initialized = False
        self.history = []

    def init_state(self, x, y, z=0.0):
        """Initialize state with position, zero velocity/acceleration."""
        self.states = np.array([[x], [y], [z],
                                [0.0], [0.0], [0.0],
                                [0.0], [0.0], [0.0]])
        self.P = self.P_init.copy()
        self.is_initialized = True
        self.history = [{"x": x, "y": y, "z": z,
                         "vx": 0.0, "vy": 0.0, "vz": 0.0}]

    def predict(self):
        """Prediction step. Returns predicted state before update."""
        if not self.is_initialized:
            return None
        pred_states = self.A @ self.states
        self.P = self.A @ self.P @ self.A.T + self.Q
        self.states = pred_states
        return pred_states.flatten()

    def update(self, x_obs, y_obs, z_obs=0.0):
        """Construct observation and perform Kalman update.

        Observation construction (extended to 3D):
        - position: direct measurement
        - velocity: (current_pos - pos_k_frames_ago) / (dt * k)
        - acceleration: (current_vel - prev_vel) / (dt * k)
        """
        if not self.is_initialized:
            self.init_state(x_obs, y_obs, z_obs)
            return self.states.flatten()

        k = min(self.avg_frames, len(self.history))
        prev = self.history[-k]

        dtk = self.dt * k
        vx_obs = (x_obs - prev["x"]) / dtk
        vy_obs = (y_obs - prev["y"]) / dtk
        vz_obs = (z_obs - prev["z"]) / dtk
        ax_obs = (vx_obs - prev["vx"]) / dtk
        ay_obs = (vy_obs - prev["vy"]) / dtk
        az_obs = (vz_obs - prev["vz"]) / dtk

        Z = np.array([[x_obs], [y_obs], [z_obs],
                       [vx_obs], [vy_obs], [vz_obs],
                       [ax_obs], [ay_obs], [az_obs]])

        # Kalman update
        S = self.R + self.H @ self.P @ self.H.T
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.states = self.states + K @ (Z - self.H @ self.states)
        I = np.eye(9)
        self.P = (I - K @ self.H) @ self.P

        # Store in history
        self.history.append({
            "x": self.states[0, 0],
            "y": self.states[1, 0],
            "z": self.states[2, 0],
            "vx": self.states[3, 0],
            "vy": self.states[4, 0],
            "vz": self.states[5, 0],
        })

        return self.states.flatten()

    def predict_position(self):
        """Return predicted position (x, y, z) from current state after predict step."""
        pred = self.A @ self.states
        return pred[0, 0], pred[1, 0], pred[2, 0]

    def get_state(self):
        """Return current state as flat array [x, y, z, vx, vy, vz, ax, ay, az]."""
        if self.states is None:
            return np.zeros(9)
        return self.states.flatten()


def run_kf_on_trajectory(positions, dt=0.5, **kf_params):
    """Run Kalman filter on a trajectory and return per-step predictions.

    Args:
        positions: np.array (N, 3) of (x, y, z) positions
        dt: time step
        **kf_params: passed to KalmanFilterCA

    Returns:
        predictions: np.array (N-1, 3) -- one-step-ahead predicted positions
                     predictions[i] is the KF prediction for positions[i+1]
    """
    kf = KalmanFilterCA(dt=dt, **kf_params)
    predictions = []

    for i in range(len(positions)):
        pos = positions[i]
        x, y = pos[0], pos[1]
        z = pos[2] if len(pos) > 2 else 0.0

        if i == 0:
            kf.init_state(x, y, z)
            continue

        # Predict step (before seeing observation at time i)
        pred_state = kf.predict()
        predictions.append([pred_state[0], pred_state[1], pred_state[2]])

        # Update with actual observation
        kf.update(x, y, z)

    return np.array(predictions)
