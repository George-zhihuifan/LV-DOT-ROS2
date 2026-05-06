"""GRU training script with cosine annealing and early stopping."""

import argparse
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from src.dataset import build_eth_ucy_datasets, build_lvdot_dataset, build_nuscenes_datasets
from src.model import GRUPredictor


def train_one_epoch(model, loader, criterion, optimizer, device):
    model.train()
    total_loss = 0.0
    for x_batch, y_batch in loader:
        x_batch = x_batch.to(device)
        y_batch = y_batch.to(device)
        pred, _ = model(x_batch)
        loss = criterion(pred, y_batch)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * x_batch.size(0)
    return total_loss / len(loader.dataset)


def evaluate(model, loader, criterion, device):
    model.eval()
    total_loss = 0.0
    with torch.no_grad():
        for x_batch, y_batch in loader:
            x_batch = x_batch.to(device)
            y_batch = y_batch.to(device)
            pred, _ = model(x_batch)
            loss = criterion(pred, y_batch)
            total_loss += loss.item() * x_batch.size(0)
    return total_loss / len(loader.dataset)


def main():
    parser = argparse.ArgumentParser(description="Train GRU predictor")
    parser.add_argument("--config", type=str, default="config.yaml")
    parser.add_argument("--data", type=str, default=None, help="LV-DOT CSV path (optional)")
    parser.add_argument("--scene", type=str, default=None,
                        help="ETH/UCY test scene (default: train on all except zara2, test on zara2)")
    parser.add_argument("--nuscenes", action="store_true", help="Use nuScenes 3D data")
    parser.add_argument("--output-dir", type=str, default="outputs")
    args = parser.parse_args()

    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    gcfg = config["gru"]
    tcfg = config["train"]
    seq_len = gcfg["seq_len"]

    # Build datasets
    if args.nuscenes:
        print("Loading nuScenes 3D datasets...")
        train_ds, test_ds, _ = build_nuscenes_datasets(config, seq_len=seq_len)
    elif args.data:
        print(f"Loading LV-DOT data from {args.data}")
        train_ds, test_ds, _ = build_lvdot_dataset(args.data, seq_len=seq_len)
    else:
        print("Loading ETH/UCY datasets...")
        datasets = build_eth_ucy_datasets(seq_len=seq_len)
        test_scene = args.scene or "zara2"
        if test_scene not in datasets:
            print(f"Scene '{test_scene}' not available. Available: {list(datasets.keys())}")
            return
        train_ds, test_ds, _ = datasets[test_scene]
        print(f"Leave-one-out: test on '{test_scene}'")

    print(f"Train samples: {len(train_ds)}, Test samples: {len(test_ds)}")

    train_loader = DataLoader(train_ds, batch_size=tcfg["batch_size"], shuffle=True)
    test_loader = DataLoader(test_ds, batch_size=tcfg["batch_size"], shuffle=False)

    # Model
    model = GRUPredictor(
        input_dim=gcfg["input_dim"],
        hidden_dim=gcfg["hidden_dim"],
        output_dim=gcfg["output_dim"],
        num_layers=gcfg["num_layers"],
    ).to(device)

    param_count = sum(p.numel() for p in model.parameters())
    print(f"Model parameters: {param_count}")

    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=tcfg["learning_rate"])
    scheduler = CosineAnnealingLR(optimizer, T_max=tcfg["epochs"])

    os.makedirs(args.output_dir, exist_ok=True)

    # Training loop
    best_val_loss = float("inf")
    patience_counter = 0
    train_losses = []
    val_losses = []

    for epoch in range(1, tcfg["epochs"] + 1):
        train_loss = train_one_epoch(model, train_loader, criterion, optimizer, device)
        val_loss = evaluate(model, test_loader, criterion, device)
        scheduler.step()

        train_losses.append(train_loss)
        val_losses.append(val_loss)

        if epoch % 10 == 0 or epoch == 1:
            print(f"Epoch {epoch:3d} | Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f} | LR: {scheduler.get_last_lr()[0]:.6f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
            torch.save(model.state_dict(), os.path.join(args.output_dir, "best_model.pth"))
        else:
            patience_counter += 1
            if patience_counter >= tcfg["patience"]:
                print(f"Early stopping at epoch {epoch} (patience={tcfg['patience']})")
                break

    print(f"Best validation loss: {best_val_loss:.6f}")

    # Plot training curves
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(train_losses, label="Train Loss")
    ax.plot(val_losses, label="Val Loss")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("MSE Loss")
    ax.set_title("Training Curves")
    ax.legend()
    ax.grid(True)
    fig.savefig(os.path.join(args.output_dir, "train_loss.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Training curve saved to {args.output_dir}/train_loss.png")


if __name__ == "__main__":
    main()
