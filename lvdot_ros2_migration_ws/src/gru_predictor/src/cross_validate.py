#!/usr/bin/env python3
"""Run leave-one-out cross-validation across all 5 ETH/UCY scenes.

For each scene: train on the other 4, evaluate KF vs GRU vs Hybrid.
Outputs a summary table across all scenes.
"""

import os
import sys
import subprocess

import numpy as np
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))


SCENES = ["eth", "hotel", "univ", "zara1", "zara2"]


def run_scene(scene, config_path, base_output_dir):
    """Train and evaluate on one scene."""
    output_dir = os.path.join(base_output_dir, scene)
    os.makedirs(output_dir, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"  Scene: {scene} (train on others, test on {scene})")
    print(f"{'='*60}")

    # Train
    print(f"\n--- Training (test={scene}) ---")
    from src.dataset import build_eth_ucy_datasets
    from src.model import GRUPredictor
    from src.train import train_one_epoch, evaluate as eval_model

    import torch
    import torch.nn as nn
    from torch.optim.lr_scheduler import CosineAnnealingLR
    from torch.utils.data import DataLoader

    with open(config_path) as f:
        config = yaml.safe_load(f)

    gcfg = config["gru"]
    tcfg = config["train"]
    seq_len = gcfg["seq_len"]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    datasets = build_eth_ucy_datasets(seq_len=seq_len)
    if scene not in datasets:
        print(f"  Scene '{scene}' not available, skipping.")
        return None

    train_ds, test_ds, test_trajectories = datasets[scene]
    print(f"  Train: {len(train_ds)} samples, Test: {len(test_ds)} samples")

    train_loader = DataLoader(train_ds, batch_size=tcfg["batch_size"], shuffle=True)
    test_loader = DataLoader(test_ds, batch_size=tcfg["batch_size"], shuffle=False)

    model = GRUPredictor(
        input_dim=gcfg["input_dim"],
        hidden_dim=gcfg["hidden_dim"],
        output_dim=gcfg["output_dim"],
        num_layers=gcfg["num_layers"],
    ).to(device)

    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=tcfg["learning_rate"])
    scheduler = CosineAnnealingLR(optimizer, T_max=tcfg["epochs"])

    best_val_loss = float("inf")
    patience_counter = 0
    model_path = os.path.join(output_dir, "best_model.pth")

    for epoch in range(1, tcfg["epochs"] + 1):
        train_loss = train_one_epoch(model, train_loader, criterion, optimizer, device)
        val_loss = eval_model(model, test_loader, criterion, device)
        scheduler.step()

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
            torch.save(model.state_dict(), model_path)
        else:
            patience_counter += 1
            if patience_counter >= tcfg["patience"]:
                print(f"  Early stop at epoch {epoch}")
                break

    print(f"  Best val loss: {best_val_loss:.6f}")

    # Evaluate
    print(f"\n--- Evaluating (test={scene}) ---")
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    from src.evaluate import evaluate_scene, print_summary, plot_trajectories, plot_gamma_overview

    if len(test_trajectories) > 200:
        np.random.seed(42)
        indices = np.random.choice(len(test_trajectories), 200, replace=False)
        test_trajectories = [test_trajectories[i] for i in indices]

    results, motion_types, traj_data = evaluate_scene(
        test_trajectories, model, config, model_path, device, seq_len
    )

    print_summary(results, motion_types, output_dir)
    plot_trajectories(traj_data, output_dir, max_plots=5)
    plot_gamma_overview(traj_data, output_dir)

    return results, motion_types


def main():
    config_path = "config.yaml"
    base_output_dir = "outputs/cross_val"
    os.makedirs(base_output_dir, exist_ok=True)

    all_results = {}
    for scene in SCENES:
        result = run_scene(scene, config_path, base_output_dir)
        if result is not None:
            all_results[scene] = result

    # Print overall summary
    print(f"\n\n{'='*70}")
    print("CROSS-VALIDATION SUMMARY (all scenes)")
    print(f"{'='*70}")
    print(f"\n{'Scene':<10} {'Method':<10} {'ADE':>8} {'FDE':>8}")
    print("-" * 40)

    overall = {"kf": [], "gru": [], "hybrid": []}
    lines = []

    for scene in SCENES:
        if scene not in all_results:
            continue
        results, _ = all_results[scene]
        for method in ["kf", "gru", "hybrid"]:
            ades = [r[0] for r in results[method]]
            fdes = [r[1] for r in results[method]]
            mean_ade = np.mean(ades)
            mean_fde = np.mean(fdes)
            overall[method].append((mean_ade, mean_fde))
            line = f"{scene:<10} {method.upper():<10} {mean_ade:8.4f} {mean_fde:8.4f}"
            print(line)
            lines.append(line)
        print()
        lines.append("")

    print("-" * 40)
    lines.append("-" * 40)
    for method in ["kf", "gru", "hybrid"]:
        mean_ade = np.mean([r[0] for r in overall[method]])
        mean_fde = np.mean([r[1] for r in overall[method]])
        line = f"{'AVERAGE':<10} {method.upper():<10} {mean_ade:8.4f} {mean_fde:8.4f}"
        print(line)
        lines.append(line)

    # Save summary
    with open(os.path.join(base_output_dir, "cross_val_summary.txt"), "w") as f:
        f.write("CROSS-VALIDATION SUMMARY\n")
        f.write("=" * 70 + "\n\n")
        f.write(f"{'Scene':<10} {'Method':<10} {'ADE':>8} {'FDE':>8}\n")
        f.write("-" * 40 + "\n")
        f.write("\n".join(lines))

    # Plot comparison bar chart
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    x = np.arange(len(SCENES))
    width = 0.25

    for metric_idx, metric_name in enumerate(["ADE", "FDE"]):
        ax = axes[metric_idx]
        for i, method in enumerate(["kf", "gru", "hybrid"]):
            vals = []
            for scene in SCENES:
                if scene in all_results:
                    results, _ = all_results[scene]
                    rs = [r[metric_idx] for r in results[method]]
                    vals.append(np.mean(rs))
                else:
                    vals.append(0)
            bars = ax.bar(x + i * width, vals, width, label=method.upper())
        ax.set_xlabel("Scene")
        ax.set_ylabel(metric_name)
        ax.set_title(f"{metric_name} Comparison")
        ax.set_xticks(x + width)
        ax.set_xticklabels(SCENES)
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()
    fig.savefig(os.path.join(base_output_dir, "cross_val_comparison.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\nComparison chart saved to {base_output_dir}/cross_val_comparison.png")


if __name__ == "__main__":
    main()
