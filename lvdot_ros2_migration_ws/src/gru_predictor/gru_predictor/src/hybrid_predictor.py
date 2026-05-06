"""Hybrid predictor combining Kalman filter and GRU (3D version).

Implements equations (32)-(35) from the proposal report:
    p_hybrid = gamma_t * p_KF + (1 - gamma_t) * p_GRU
    gamma_t  = sigmoid(-beta_s * (e_GRU - e_KF))
    gamma_t  = gamma_t * max(0, 1 - c_occ / C_max)

All three methods (KF, GRU, Hybrid) predict position[i+1] after observing
position[i], so they are fairly comparable.
"""

import numpy as np
import torch

from .kalman_baseline import KalmanFilterCA
from .model import GRUPredictor


class HybridPredictor:
    """Adaptive KF+GRU hybrid predictor with dynamic weight switching (3D)."""

    def __init__(self, model_path, config, device="cpu"):
        self.device = torch.device(device)
        self.seq_len = config["gru"]["seq_len"]
        self.input_dim = config["gru"]["input_dim"]
        self.output_dim = config["gru"]["output_dim"]

        # GRU model
        self.gru = GRUPredictor(
            input_dim=self.input_dim,
            hidden_dim=config["gru"]["hidden_dim"],
            output_dim=self.output_dim,
            num_layers=config["gru"]["num_layers"],
        ).to(self.device)
        self.gru.load_state_dict(torch.load(model_path, map_location=self.device))
        self.gru.eval()

        # Kalman filter
        kcfg = config["kalman"]
        self._kf_params = dict(
            dt=kcfg["dt"], eP=kcfg["eP"],
            eQ_pos=kcfg["eQ_pos"], eQ_vel=kcfg["eQ_vel"], eQ_acc=kcfg["eQ_acc"],
            eR_pos=kcfg["eR_pos"], eR_vel=kcfg["eR_vel"], eR_acc=kcfg["eR_acc"],
            avg_frames=kcfg["avg_frames"],
        )
        self.kf = KalmanFilterCA(**self._kf_params)

        # Hybrid parameters
        hcfg = config["hybrid"]
        self.beta_s = hcfg["beta_s"]
        self.error_window = hcfg["error_window"]
        self.c_max = hcfg["c_max"]

        # Runtime state
        self._reset_buffers()

    def _reset_buffers(self):
        self.kf_errors = []
        self.gru_errors = []
        self.input_buffer = []  # incremental features for GRU
        self.prev_pos = None
        self.prev_vel = None  # finite-difference velocity (matches training)
        self.gamma_history = []
        # Previous predictions -- used to compute retrospective errors
        self._prev_kf_pred = None
        self._prev_gru_pred = None

    def reset(self):
        """Reset state for a new trajectory."""
        self.kf = KalmanFilterCA(**self._kf_params)
        self._reset_buffers()

    def _compute_gamma(self, c_occ=0):
        """Compute adaptive weight gamma_t (eq. 34-35)."""
        if not self.kf_errors or not self.gru_errors:
            return 0.5

        W = self.error_window
        e_kf = np.mean(self.kf_errors[-W:])
        e_gru = np.mean(self.gru_errors[-W:])

        # eq. 34: gamma high when GRU error > KF error (prefer KF)
        gamma = 1.0 / (1.0 + np.exp(self.beta_s * (e_gru - e_kf)))

        # eq. 35: occlusion decay
        occ_factor = max(0.0, 1.0 - c_occ / self.c_max)
        gamma *= occ_factor

        return gamma

    def predict(self, x_obs, y_obs, z_obs=0.0, w=0.0, h=0.0, l=0.0, c_occ=0):
        """Process one observation and return prediction for the NEXT position.

        All predictions (kf_pred, gru_pred, hybrid_pred) target position[i+1]
        after observing position[i] = (x_obs, y_obs, z_obs).

        Args:
            x_obs, y_obs, z_obs: current observed 3D position
            w, h, l: bounding box size
            c_occ: consecutive occlusion frame count

        Returns:
            dict with keys: hybrid_pred, kf_pred, gru_pred, gamma
        """
        curr_pos = np.array([x_obs, y_obs, z_obs])

        # --- Retrospective error tracking ---
        if self._prev_kf_pred is not None:
            kf_err = np.linalg.norm(self._prev_kf_pred - curr_pos)
            gru_err = np.linalg.norm(self._prev_gru_pred - curr_pos)
            self.kf_errors.append(kf_err)
            self.gru_errors.append(gru_err)

        # --- KF: predict + update ---
        if not self.kf.is_initialized:
            self.kf.init_state(x_obs, y_obs, z_obs)
            self.prev_pos = curr_pos.copy()
            self.prev_vel = np.zeros(3)
            self._prev_kf_pred = curr_pos.copy()
            self._prev_gru_pred = curr_pos.copy()
            return {
                "hybrid_pred": curr_pos.copy(),
                "kf_pred": curr_pos.copy(),
                "gru_pred": curr_pos.copy(),
                "gamma": 0.5,
            }

        self.kf.predict()
        self.kf.update(x_obs, y_obs, z_obs)

        # KF prediction for next frame: apply transition to filtered state
        kf_next_state = self.kf.A @ self.kf.states
        kf_pred_next = np.array([kf_next_state[0, 0],
                                  kf_next_state[1, 0],
                                  kf_next_state[2, 0]])

        # --- Build GRU incremental feature (matching training data) ---
        curr_vel = curr_pos - self.prev_pos
        dpos = curr_pos - self.prev_pos
        dvel = curr_vel - self.prev_vel
        # 9-dim feature: [dx, dy, dz, dvx, dvy, dvz, w, h, l]
        feat = np.array([dpos[0], dpos[1], dpos[2],
                         dvel[0], dvel[1], dvel[2],
                         w, h, l], dtype=np.float32)
        self.input_buffer.append(feat)
        self.prev_pos = curr_pos.copy()
        self.prev_vel = curr_vel.copy()

        # --- GRU prediction for next frame ---
        if len(self.input_buffer) >= self.seq_len:
            seq = np.array(self.input_buffer[-self.seq_len:])
            x_tensor = torch.tensor(seq, dtype=torch.float32).unsqueeze(0).to(self.device)
            with torch.no_grad():
                delta_pred, _ = self.gru(x_tensor)
            delta = delta_pred.cpu().numpy().flatten()
            gru_pred_next = curr_pos + delta[:3]
        else:
            gru_pred_next = kf_pred_next.copy()

        # --- Adaptive fusion ---
        gamma = self._compute_gamma(c_occ)
        self.gamma_history.append(gamma)

        hybrid_pred_next = gamma * kf_pred_next + (1 - gamma) * gru_pred_next

        # Store predictions for retrospective error at next step
        self._prev_kf_pred = kf_pred_next.copy()
        self._prev_gru_pred = gru_pred_next.copy()

        return {
            "hybrid_pred": hybrid_pred_next,
            "kf_pred": kf_pred_next,
            "gru_pred": gru_pred_next,
            "gamma": gamma,
        }
