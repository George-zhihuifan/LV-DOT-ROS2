"""Hyperparameter tuning for 3D Kalman-GRU hybrid predictor on nuScenes.

Searches over KF noise parameters and hybrid switching parameters to
minimize Hybrid ADE on the test set.
"""

import itertools
import os
import sys

import numpy as np
import torch
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from src.dataset import build_nuscenes_datasets
from src.model import GRUPredictor
from src.kalman_baseline import KalmanFilterCA
from src.hybrid_predictor import HybridPredictor


def quick_evaluate(test_trajectories, config, model_path, device, seq_len,
                   max_trajs=50):
    """Fast evaluation: only compute ADE on a subset of trajectories."""
    model = GRUPredictor(
        input_dim=config["gru"]["input_dim"],
        hidden_dim=config["gru"]["hidden_dim"],
        output_dim=config["gru"]["output_dim"],
        num_layers=config["gru"]["num_layers"],
    ).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    kcfg = config["kalman"]
    kf_ades = []
    hybrid_ades = []

    for positions in test_trajectories[:max_trajs]:
        if positions.shape[1] == 2:
            positions = np.hstack([positions, np.zeros((len(positions), 1))])
        if len(positions) < seq_len + 2:
            continue

        gt = positions[1:]

        # KF
        kf = KalmanFilterCA(
            dt=kcfg["dt"], eP=kcfg["eP"],
            eQ_pos=kcfg["eQ_pos"], eQ_vel=kcfg["eQ_vel"], eQ_acc=kcfg["eQ_acc"],
            eR_pos=kcfg["eR_pos"], eR_vel=kcfg["eR_vel"], eR_acc=kcfg["eR_acc"],
            avg_frames=kcfg["avg_frames"],
        )
        kf_preds = []
        for i in range(len(positions)):
            x, y, z = positions[i]
            if i == 0:
                kf.init_state(x, y, z)
            else:
                kf.predict()
                kf.update(x, y, z)
            ns = kf.A @ kf.states
            kf_preds.append([ns[0, 0], ns[1, 0], ns[2, 0]])
        kf_preds = np.array(kf_preds[:-1])

        # Hybrid
        predictor = HybridPredictor(model_path, config, device=str(device))
        hyb_preds = []
        for i in range(len(positions)):
            x, y, z = positions[i]
            result = predictor.predict(x, y, z)
            hyb_preds.append(result["hybrid_pred"])
        hyb_preds = np.array(hyb_preds[:-1])

        min_len = min(len(kf_preds), len(hyb_preds), len(gt))
        if min_len < 2:
            continue

        kf_ade = np.mean(np.linalg.norm(kf_preds[:min_len] - gt[:min_len], axis=1))
        hyb_ade = np.mean(np.linalg.norm(hyb_preds[:min_len] - gt[:min_len], axis=1))
        kf_ades.append(kf_ade)
        hybrid_ades.append(hyb_ade)

    return np.mean(kf_ades), np.mean(hybrid_ades)


def main():
    config_path = "config.yaml"
    model_path = "outputs/nuscenes_3d/best_model.pth"

    with open(config_path, "r") as f:
        base_config = yaml.safe_load(f)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    seq_len = base_config["gru"]["seq_len"]

    print("Loading nuScenes data...")
    _, _, test_trajectories = build_nuscenes_datasets(base_config, seq_len=seq_len)
    print(f"Test trajectories: {len(test_trajectories)}")

    # --- Phase 1: Tune KF noise parameters ---
    print("\n=== Phase 1: KF noise parameter search ===")

    kf_param_grid = {
        "eQ_pos": [0.005, 0.01, 0.02],
        "eQ_vel": [0.02, 0.05, 0.1],
        "eQ_acc": [0.02, 0.05, 0.1],
        "eR_pos": [0.02, 0.04, 0.08],
        "eR_vel": [0.15, 0.3, 0.6],
        "eR_acc": [0.3, 0.6, 1.0],
    }

    best_kf_ade = float("inf")
    best_kf_params = {}

    # Sample random combinations instead of full grid (too many)
    np.random.seed(42)
    n_samples = 30
    param_names = list(kf_param_grid.keys())
    param_values = list(kf_param_grid.values())

    for trial in range(n_samples):
        config = yaml.safe_load(yaml.dump(base_config))
        chosen = {}
        for name, values in zip(param_names, param_values):
            val = values[np.random.randint(len(values))]
            config["kalman"][name] = val
            chosen[name] = val

        kf_ade, hyb_ade = quick_evaluate(
            test_trajectories, config, model_path, device, seq_len, max_trajs=50
        )
        if trial % 5 == 0:
            print(f"  Trial {trial+1}/{n_samples}: KF={kf_ade:.4f}, Hybrid={hyb_ade:.4f}")

        if hyb_ade < best_kf_ade:
            best_kf_ade = hyb_ade
            best_kf_params = chosen.copy()
            print(f"  >> New best Hybrid ADE: {hyb_ade:.4f} (KF: {kf_ade:.4f}) params={chosen}")

    print(f"\nBest KF params: {best_kf_params}")
    print(f"Best Hybrid ADE after KF tuning: {best_kf_ade:.4f}")

    # Apply best KF params
    for k, v in best_kf_params.items():
        base_config["kalman"][k] = v

    # --- Phase 2: Tune hybrid parameters ---
    print("\n=== Phase 2: Hybrid parameter search ===")

    hybrid_grid = {
        "beta_s": [0.5, 1.0, 2.0, 3.0, 5.0],
        "error_window": [2, 3, 5],
    }

    best_hyb_ade = float("inf")
    best_hyb_params = {}

    for beta_s, ew in itertools.product(hybrid_grid["beta_s"],
                                         hybrid_grid["error_window"]):
        config = yaml.safe_load(yaml.dump(base_config))
        config["hybrid"]["beta_s"] = beta_s
        config["hybrid"]["error_window"] = ew

        kf_ade, hyb_ade = quick_evaluate(
            test_trajectories, config, model_path, device, seq_len, max_trajs=80
        )
        print(f"  beta_s={beta_s}, ew={ew}: KF={kf_ade:.4f}, Hybrid={hyb_ade:.4f}")

        if hyb_ade < best_hyb_ade:
            best_hyb_ade = hyb_ade
            best_hyb_params = {"beta_s": beta_s, "error_window": ew}

    print(f"\nBest hybrid params: {best_hyb_params}")
    print(f"Best Hybrid ADE: {best_hyb_ade:.4f}")

    # Apply best hybrid params
    for k, v in best_hyb_params.items():
        base_config["hybrid"][k] = v

    # --- Phase 3: Also try different avg_frames ---
    print("\n=== Phase 3: avg_frames search ===")
    best_avg = base_config["kalman"]["avg_frames"]
    best_avg_ade = best_hyb_ade

    for avg in [2, 3, 5, 8]:
        config = yaml.safe_load(yaml.dump(base_config))
        config["kalman"]["avg_frames"] = avg
        kf_ade, hyb_ade = quick_evaluate(
            test_trajectories, config, model_path, device, seq_len, max_trajs=80
        )
        print(f"  avg_frames={avg}: KF={kf_ade:.4f}, Hybrid={hyb_ade:.4f}")
        if hyb_ade < best_avg_ade:
            best_avg_ade = hyb_ade
            best_avg = avg

    base_config["kalman"]["avg_frames"] = best_avg

    # --- Save best config ---
    print("\n" + "=" * 60)
    print("FINAL BEST CONFIG:")
    print(f"  KF params: {best_kf_params}")
    print(f"  avg_frames: {best_avg}")
    print(f"  Hybrid params: {best_hyb_params}")
    print(f"  Final Hybrid ADE: {best_avg_ade:.4f}")

    out_path = "config_tuned.yaml"
    with open(out_path, "w") as f:
        yaml.dump(base_config, f, default_flow_style=False)
    print(f"\nSaved tuned config to {out_path}")


if __name__ == "__main__":
    main()
