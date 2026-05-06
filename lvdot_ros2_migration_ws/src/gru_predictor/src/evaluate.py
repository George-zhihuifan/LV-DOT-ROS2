"""Offline evaluation: KF vs GRU vs Hybrid comparison (3D version).

All three methods predict position[i+1] after observing position[i].
Metrics: ADE (Average Displacement Error), FDE (Final Displacement Error).
Uses 3D Euclidean distance.
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from src.dataset import (build_eth_ucy_datasets, build_lvdot_dataset,
                          build_nuscenes_datasets, ETH_UCY_SCENES, DATA_DIR)
from src.model import GRUPredictor
from src.kalman_baseline import KalmanFilterCA
from src.hybrid_predictor import HybridPredictor


def compute_ade_fde(predictions, ground_truth):
    """Compute ADE and FDE. Both arrays shape (N, 3)."""
    errors = np.linalg.norm(predictions - ground_truth, axis=1)
    ade = np.mean(errors)
    fde = errors[-1] if len(errors) > 0 else 0.0
    return ade, fde


def classify_motion(positions, threshold=0.1):
    """Classify trajectory as linear or nonlinear by acceleration variance."""
    if len(positions) < 3:
        return "linear"
    vel = np.diff(positions, axis=0)
    acc = np.diff(vel, axis=0)
    acc_var = np.var(np.linalg.norm(acc, axis=1))
    return "nonlinear" if acc_var > threshold else "linear"


def run_kf_next_step(positions, kf_params):
    """Run KF: at each step i, predict position[i+1] after observing position[i].

    Returns: predictions (N-1, 3) where predictions[i] targets positions[i+1].
    """
    kf = KalmanFilterCA(**kf_params)
    predictions = []

    for i in range(len(positions)):
        pos = positions[i]
        x, y = pos[0], pos[1]
        z = pos[2] if len(pos) > 2 else 0.0

        if i == 0:
            kf.init_state(x, y, z)
        else:
            kf.predict()
            kf.update(x, y, z)

        # Predict next position from filtered state
        next_state = kf.A @ kf.states
        predictions.append([next_state[0, 0], next_state[1, 0], next_state[2, 0]])

    # predictions[i] targets positions[i+1], so drop the last one (no GT)
    return np.array(predictions[:-1])


def run_gru_next_step(positions, model, seq_len, device, input_dim=9):
    """Run GRU: at each step i, predict position[i+1] after observing position[i].

    Features use finite-difference velocity (matching training).
    Returns: predictions (N-1, 3) where predictions[i] targets positions[i+1].
    """
    N = len(positions)
    # Ensure positions are 3D
    if positions.shape[1] == 2:
        positions = np.hstack([positions, np.zeros((N, 1))])

    velocities = np.zeros_like(positions)
    velocities[1:] = positions[1:] - positions[:-1]

    predictions = []
    input_buffer = []

    for i in range(N):
        if i >= 1:
            dpos = positions[i] - positions[i - 1]
            dvel = velocities[i] - velocities[i - 1]
            # 9-dim: [dx, dy, dz, dvx, dvy, dvz, w, h, l]
            feat = np.array([dpos[0], dpos[1], dpos[2],
                             dvel[0], dvel[1], dvel[2],
                             0.0, 0.0, 0.0], dtype=np.float32)
            input_buffer.append(feat)

        if len(input_buffer) >= seq_len:
            seq = np.array(input_buffer[-seq_len:])
            x_tensor = torch.tensor(seq, dtype=torch.float32).unsqueeze(0).to(device)
            with torch.no_grad():
                delta_pred, _ = model(x_tensor)
            delta = delta_pred.cpu().numpy().flatten()
            pred = positions[i] + delta[:3]
        else:
            pred = positions[i].copy()

        predictions.append(pred)

    return np.array(predictions[:-1])  # drop last (no GT for it)


def evaluate_scene(test_trajectories, model, config, model_path, device, seq_len):
    """Evaluate all three methods on test trajectories."""
    kcfg = config["kalman"]
    kf_params = {
        "dt": kcfg["dt"], "eP": kcfg["eP"],
        "eQ_pos": kcfg["eQ_pos"], "eQ_vel": kcfg["eQ_vel"], "eQ_acc": kcfg["eQ_acc"],
        "eR_pos": kcfg["eR_pos"], "eR_vel": kcfg["eR_vel"], "eR_acc": kcfg["eR_acc"],
        "avg_frames": kcfg["avg_frames"],
    }

    results = {"kf": [], "gru": [], "hybrid": []}
    motion_types = []
    traj_data = []

    for traj_idx, positions in enumerate(test_trajectories):
        # Ensure 3D
        if positions.ndim == 1:
            continue
        if positions.shape[1] == 2:
            positions = np.hstack([positions, np.zeros((len(positions), 1))])
        if len(positions) < seq_len + 2:
            continue

        # Ground truth: positions[1:]
        gt = positions[1:]

        # KF predictions
        kf_preds = run_kf_next_step(positions, kf_params)

        # GRU predictions
        gru_preds = run_gru_next_step(positions, model, seq_len, device)

        # Hybrid predictions
        predictor = HybridPredictor(model_path, config, device=str(device))
        hybrid_preds_list = []
        for i in range(len(positions)):
            pos = positions[i]
            result = predictor.predict(pos[0], pos[1], pos[2])
            hybrid_preds_list.append(result["hybrid_pred"])
        hybrid_preds = np.array(hybrid_preds_list[:-1])

        # Align lengths
        min_len = min(len(kf_preds), len(gru_preds), len(hybrid_preds), len(gt))
        if min_len < 2:
            continue

        kf_preds = kf_preds[:min_len]
        gru_preds = gru_preds[:min_len]
        hybrid_preds = hybrid_preds[:min_len]
        gt_aligned = gt[:min_len]

        kf_ade, kf_fde = compute_ade_fde(kf_preds, gt_aligned)
        gru_ade, gru_fde = compute_ade_fde(gru_preds, gt_aligned)
        hyb_ade, hyb_fde = compute_ade_fde(hybrid_preds, gt_aligned)

        results["kf"].append((kf_ade, kf_fde))
        results["gru"].append((gru_ade, gru_fde))
        results["hybrid"].append((hyb_ade, hyb_fde))

        motion_types.append(classify_motion(positions))
        traj_data.append({
            "positions": positions,
            "gt": gt_aligned,
            "kf": kf_preds,
            "gru": gru_preds,
            "hybrid": hybrid_preds,
            "gammas": predictor.gamma_history,
        })

    return results, motion_types, traj_data


def print_summary(results, motion_types, output_dir):
    """Print and save summary statistics."""
    lines = []

    def add(s):
        print(s)
        lines.append(s)

    add("=" * 70)
    add("Evaluation Results Summary (3D)")
    add("=" * 70)

    for method in ["kf", "gru", "hybrid"]:
        ades = [r[0] for r in results[method]]
        fdes = [r[1] for r in results[method]]
        add(f"\n{method.upper():>8s}:  ADE = {np.mean(ades):.4f} +/- {np.std(ades):.4f}  |  FDE = {np.mean(fdes):.4f} +/- {np.std(fdes):.4f}")

    for mtype in ["linear", "nonlinear"]:
        indices = [i for i, mt in enumerate(motion_types) if mt == mtype]
        if not indices:
            continue
        add(f"\n--- {mtype.upper()} trajectories ({len(indices)}) ---")
        for method in ["kf", "gru", "hybrid"]:
            ades = [results[method][i][0] for i in indices]
            fdes = [results[method][i][1] for i in indices]
            add(f"  {method.upper():>8s}:  ADE = {np.mean(ades):.4f}  |  FDE = {np.mean(fdes):.4f}")

    add("\n" + "=" * 70)

    with open(os.path.join(output_dir, "evaluation_results.txt"), "w") as f:
        f.write("\n".join(lines))


def plot_trajectories(traj_data, output_dir, max_plots=10):
    """Plot sample trajectories with predictions from all three methods (3D: XY + XZ)."""
    plot_dir = os.path.join(output_dir, "trajectory_plots")
    os.makedirs(plot_dir, exist_ok=True)

    for idx, td in enumerate(traj_data[:max_plots]):
        fig, axes = plt.subplots(1, 3, figsize=(20, 5))

        pos = td["positions"]
        kf = td["kf"]
        gru = td["gru"]
        hyb = td["hybrid"]
        min_len = min(len(kf), len(gru), len(hyb))

        # XY view
        ax = axes[0]
        ax.plot(pos[:, 0], pos[:, 1], "k-o", markersize=2, label="Ground Truth", zorder=5)
        ax.plot(kf[:min_len, 0], kf[:min_len, 1], "b--", alpha=0.7, label="KF")
        ax.plot(gru[:min_len, 0], gru[:min_len, 1], "r--", alpha=0.7, label="GRU")
        ax.plot(hyb[:min_len, 0], hyb[:min_len, 1], "g--", alpha=0.7, label="Hybrid")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title(f"Trajectory {idx} (XY)")
        ax.legend(fontsize=8)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

        # XZ view
        ax = axes[1]
        ax.plot(pos[:, 0], pos[:, 2], "k-o", markersize=2, label="Ground Truth", zorder=5)
        ax.plot(kf[:min_len, 0], kf[:min_len, 2], "b--", alpha=0.7, label="KF")
        ax.plot(gru[:min_len, 0], gru[:min_len, 2], "r--", alpha=0.7, label="GRU")
        ax.plot(hyb[:min_len, 0], hyb[:min_len, 2], "g--", alpha=0.7, label="Hybrid")
        ax.set_xlabel("X")
        ax.set_ylabel("Z")
        ax.set_title(f"Trajectory {idx} (XZ)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Error plot
        ax2 = axes[2]
        gt = td["gt"][:min_len]
        kf_err = np.linalg.norm(kf[:min_len] - gt, axis=1)
        gru_err = np.linalg.norm(gru[:min_len] - gt, axis=1)
        hyb_err = np.linalg.norm(hyb[:min_len] - gt, axis=1)
        ax2.plot(kf_err, "b-", alpha=0.7, label="KF Error")
        ax2.plot(gru_err, "r-", alpha=0.7, label="GRU Error")
        ax2.plot(hyb_err, "g-", alpha=0.7, label="Hybrid Error")
        ax2.set_xlabel("Frame")
        ax2.set_ylabel("3D Displacement Error")
        ax2.set_title("Per-frame Error")
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3)

        if td["gammas"]:
            ax3 = ax2.twinx()
            gammas = td["gammas"][:min_len]
            ax3.plot(gammas, "m:", alpha=0.6, label="gamma_t")
            ax3.set_ylabel("gamma_t", color="m")
            ax3.set_ylim(-0.05, 1.05)
            ax3.legend(loc="upper left", fontsize=8)

        fig.tight_layout()
        fig.savefig(os.path.join(plot_dir, f"traj_{idx:03d}.png"), dpi=150)
        plt.close(fig)

    print(f"Trajectory plots saved to {plot_dir}/")


def plot_gamma_overview(traj_data, output_dir):
    """Plot gamma_t curves for all evaluated trajectories."""
    fig, ax = plt.subplots(figsize=(10, 4))
    for td in traj_data:
        if td["gammas"]:
            ax.plot(td["gammas"], alpha=0.3, linewidth=0.8)
    ax.axhline(y=0.5, color="k", linestyle="--", alpha=0.5, label="gamma=0.5")
    ax.set_xlabel("Frame")
    ax.set_ylabel("gamma_t")
    ax.set_title("Adaptive Weight gamma_t Across Trajectories")
    ax.set_ylim(-0.05, 1.05)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.savefig(os.path.join(output_dir, "gamma_overview.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Gamma overview saved to {output_dir}/gamma_overview.png")


def main():
    parser = argparse.ArgumentParser(description="Evaluate KF vs GRU vs Hybrid (3D)")
    parser.add_argument("--config", type=str, default="config.yaml")
    parser.add_argument("--model", type=str, default="outputs/best_model.pth")
    parser.add_argument("--data", type=str, default=None, help="LV-DOT CSV path (optional)")
    parser.add_argument("--scene", type=str, default=None, help="ETH/UCY test scene")
    parser.add_argument("--nuscenes", action="store_true", help="Use nuScenes data")
    parser.add_argument("--output-dir", type=str, default="outputs")
    parser.add_argument("--max-trajectories", type=int, default=200)
    args = parser.parse_args()

    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    seq_len = config["gru"]["seq_len"]

    model = GRUPredictor(
        input_dim=config["gru"]["input_dim"],
        hidden_dim=config["gru"]["hidden_dim"],
        output_dim=config["gru"]["output_dim"],
        num_layers=config["gru"]["num_layers"],
    ).to(device)
    model.load_state_dict(torch.load(args.model, map_location=device))
    model.eval()
    print(f"Loaded model from {args.model}")

    if args.nuscenes:
        _, _, test_trajectories = build_nuscenes_datasets(config, seq_len=seq_len)
    elif args.data:
        from src.dataset import load_lvdot_csv
        trajs = load_lvdot_csv(args.data)
        test_trajectories = []
        for tid, traj in trajs.items():
            arr = np.array(traj)
            positions = arr[:, 1:3]
            if len(positions) >= seq_len + 2:
                test_trajectories.append(positions)
    else:
        test_scene = args.scene or "zara2"
        datasets = build_eth_ucy_datasets(seq_len=seq_len)
        if test_scene not in datasets:
            print(f"Scene '{test_scene}' not available. Available: {list(datasets.keys())}")
            return
        _, _, test_trajectories = datasets[test_scene]
        print(f"Test scene: {test_scene}")

    if len(test_trajectories) > args.max_trajectories:
        np.random.seed(42)
        indices = np.random.choice(len(test_trajectories), args.max_trajectories, replace=False)
        test_trajectories = [test_trajectories[i] for i in indices]

    print(f"Evaluating {len(test_trajectories)} trajectories...")

    results, motion_types, traj_data = evaluate_scene(
        test_trajectories, model, config, args.model, device, seq_len
    )

    os.makedirs(args.output_dir, exist_ok=True)
    print_summary(results, motion_types, args.output_dir)
    plot_trajectories(traj_data, args.output_dir)
    plot_gamma_overview(traj_data, args.output_dir)
    print(f"\nAll results saved to {args.output_dir}/")


if __name__ == "__main__":
    main()
