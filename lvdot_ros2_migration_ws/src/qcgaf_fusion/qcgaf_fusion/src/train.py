"""
QC-GAF training script.

Loss = w_box * L_box + w_conf * L_conf + w_match * L_match + w_alpha * L_alpha
  L_box:   Smooth L1 on [x,y,z,w,h,l] (masked to valid cam dets)
  L_conf:  BCE on confidence
  L_match: BCE on soft matching matrix vs GT binary match
  L_alpha: Smooth L1 on effective sensor blend weight vs geometry-derived target
"""

import os
import time

import numpy as np
import torch
import torch.nn as nn
import yaml
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader

from dataset import build_datasets
from model import QCGAF, count_parameters


def collate_fn(batch):
    """Stack dicts of tensors into batched tensors."""
    out = {}
    for k in batch[0]:
        out[k] = torch.stack([b[k] for b in batch])
    return out


class QCGAFLoss(nn.Module):
    def __init__(self, w_box=1.0, w_conf=0.5, w_match=0.3, w_alpha=0.0):
        super().__init__()
        self.w_box = w_box
        self.w_conf = w_conf
        self.w_match = w_match
        self.w_alpha = w_alpha
        self.smooth_l1 = nn.SmoothL1Loss(reduction="none")
        self.bce = nn.BCEWithLogitsLoss(reduction="none")

    def forward(self, pred_boxes, pred_match, gt_fused, gt_match,
                cam_mask, lidar_mask, cam_dets, lidar_dets, aux):
        """
        Args:
            pred_boxes: (B, M, 7)  last dim = [x,y,z,w,h,l,conf_logit]
            pred_match: (B, M, N)  already softmaxed
            gt_fused:   (B, M, 7)  [x,y,z,w,h,l,conf]
            gt_match:   (B, M, N)  binary
            cam_mask:   (B, M)     bool
            lidar_mask: (B, N)     bool
        """
        B = pred_boxes.shape[0]
        cm = cam_mask.float().unsqueeze(-1)  # (B, M, 1)

        # ── L_box: Smooth L1 on [x,y,z,w,h,l] ──
        l_box = self.smooth_l1(pred_boxes[:, :, :6], gt_fused[:, :, :6])  # (B, M, 6)
        l_box = (l_box * cm).sum() / (cm.sum() * 6 + 1e-8)

        # ── L_conf: BCE on confidence ──
        l_conf = self.bce(pred_boxes[:, :, 6], gt_fused[:, :, 6])  # (B, M)
        l_conf = (l_conf * cam_mask.float()).sum() / (cam_mask.float().sum() + 1e-8)

        # ── L_match: BCE on matching matrix ──
        # mask: only valid (cam_i, lidar_j) pairs
        match_mask = cam_mask.unsqueeze(2).float() * lidar_mask.unsqueeze(1).float()  # (B,M,N)
        # clamp pred_match to avoid log(0)
        pm = pred_match.clamp(1e-7, 1 - 1e-7)
        l_match_raw = -(gt_match * pm.log() + (1 - gt_match) * (1 - pm).log())
        l_match = (l_match_raw * match_mask).sum() / (match_mask.sum() + 1e-8)

        # ── L_alpha: supervise effective camera-vs-lidar blend weight ──
        alpha_mask = cam_mask.float()
        matched_rows = (gt_match.sum(dim=2) > 0) & cam_mask
        gt_row_norm = gt_match / gt_match.sum(dim=2, keepdim=True).clamp(min=1e-8)
        gt_lidar_pos = torch.bmm(gt_row_norm, lidar_dets[:, :, :6])  # (B, M, 6)

        cam_err = torch.abs(cam_dets[:, :, :6] - gt_fused[:, :, :6]).mean(dim=-1)
        lidar_err = torch.abs(gt_lidar_pos - gt_fused[:, :, :6]).mean(dim=-1)

        alpha_target = torch.ones_like(cam_err)
        alpha_target[matched_rows] = lidar_err[matched_rows] / (
            cam_err[matched_rows] + lidar_err[matched_rows] + 1e-6
        )

        eff_alpha = aux["eff_alpha"].squeeze(-1)  # (B, M)
        l_alpha_raw = self.smooth_l1(eff_alpha, alpha_target)
        l_alpha = (l_alpha_raw * alpha_mask).sum() / (alpha_mask.sum() + 1e-8)

        loss = (
            self.w_box * l_box
            + self.w_conf * l_conf
            + self.w_match * l_match
            + self.w_alpha * l_alpha
        )
        return loss, {
            "box": l_box.item(),
            "conf": l_conf.item(),
            "match": l_match.item(),
            "alpha": l_alpha.item(),
        }


def train_epoch(model, loader, criterion, optimizer, device):
    model.train()
    total_loss = 0.0
    comp = {"box": 0.0, "conf": 0.0, "match": 0.0, "alpha": 0.0}
    n = 0

    for batch in loader:
        cam = batch["cam_dets"].to(device)
        lidar = batch["lidar_dets"].to(device)
        q = batch["quality"].to(device)
        cm = batch["cam_mask"].to(device)
        lm = batch["lidar_mask"].to(device)
        gt_fused = batch["gt_fused"].to(device)
        gt_match = batch["gt_match"].to(device)

        pred_boxes, pred_match, aux = model(cam, lidar, q, cm, lm, return_aux=True)
        loss, details = criterion(pred_boxes, pred_match, gt_fused, gt_match, cm, lm, cam, lidar, aux)

        optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
        optimizer.step()

        bs = cam.size(0)
        total_loss += loss.item() * bs
        for k in comp:
            comp[k] += details[k] * bs
        n += bs

    return total_loss / n, {k: v / n for k, v in comp.items()}


@torch.no_grad()
def eval_epoch(model, loader, criterion, device):
    model.eval()
    total_loss = 0.0
    comp = {"box": 0.0, "conf": 0.0, "match": 0.0, "alpha": 0.0}
    n = 0

    for batch in loader:
        cam = batch["cam_dets"].to(device)
        lidar = batch["lidar_dets"].to(device)
        q = batch["quality"].to(device)
        cm = batch["cam_mask"].to(device)
        lm = batch["lidar_mask"].to(device)
        gt_fused = batch["gt_fused"].to(device)
        gt_match = batch["gt_match"].to(device)

        pred_boxes, pred_match, aux = model(cam, lidar, q, cm, lm, return_aux=True)
        loss, details = criterion(pred_boxes, pred_match, gt_fused, gt_match, cm, lm, cam, lidar, aux)

        bs = cam.size(0)
        total_loss += loss.item() * bs
        for k in comp:
            comp[k] += details[k] * bs
        n += bs

    return total_loss / n, {k: v / n for k, v in comp.items()}


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Train QC-GAF")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--output-dir", default=None,
                        help="Directory for checkpoints/history. Defaults to ../outputs")
    parser.add_argument("--init-checkpoint", default=None,
                        help="Optional checkpoint to initialize weights from.")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # data
    train_ds, test_ds = build_datasets(config)
    tcfg = config["train"]
    train_loader = DataLoader(train_ds, batch_size=tcfg["batch_size"],
                              shuffle=True, collate_fn=collate_fn,
                              num_workers=0, drop_last=False)
    test_loader = DataLoader(test_ds, batch_size=tcfg["batch_size"],
                             shuffle=False, collate_fn=collate_fn,
                             num_workers=0)

    # model
    mcfg = config["model"]
    model = QCGAF(
        cam_dim=mcfg["cam_dim"], lidar_dim=mcfg["lidar_dim"],
        quality_dim=mcfg["quality_dim"], hidden_dim=mcfg["hidden_dim"],
        feat_dim=mcfg["feat_dim"], output_dim=mcfg["output_dim"],
    ).to(device)
    n_params = count_parameters(model)
    print(f"Model parameters: {n_params:,}")

    if args.init_checkpoint:
        ckpt = torch.load(args.init_checkpoint, map_location=device)
        model.load_state_dict(ckpt["model_state_dict"], strict=False)
        print(f"Initialized from {args.init_checkpoint} (epoch={ckpt.get('epoch', 'n/a')})")

    # loss
    criterion = QCGAFLoss(
        w_box=tcfg["loss_w_box"],
        w_conf=tcfg["loss_w_conf"],
        w_match=tcfg["loss_w_match"],
        w_alpha=tcfg.get("loss_w_alpha", 0.0),
    )

    # optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=tcfg["learning_rate"])
    scheduler = CosineAnnealingLR(optimizer, T_max=tcfg["epochs"], eta_min=1e-6)

    # training loop
    out_dir = args.output_dir or os.path.join(os.path.dirname(__file__), "..", "outputs")
    os.makedirs(out_dir, exist_ok=True)
    best_path = os.path.join(out_dir, "best_model.pt")

    best_val_loss = float("inf")
    patience_counter = 0
    history = []

    print(f"\nTraining for up to {tcfg['epochs']} epochs (patience={tcfg['patience']})")
    print("-" * 80)

    for epoch in range(1, tcfg["epochs"] + 1):
        t0 = time.time()
        train_loss, train_comp = train_epoch(model, train_loader, criterion, optimizer, device)
        val_loss, val_comp = eval_epoch(model, test_loader, criterion, device)
        scheduler.step()

        dt = time.time() - t0
        lr = optimizer.param_groups[0]["lr"]

        history.append({
            "epoch": epoch,
            "train_loss": train_loss, "val_loss": val_loss,
            "train_comp": train_comp, "val_comp": val_comp,
            "lr": lr,
        })

        improved = val_loss < best_val_loss
        if improved:
            best_val_loss = val_loss
            patience_counter = 0
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "val_loss": val_loss,
                "config": config,
            }, best_path)
            tag = " *"
        else:
            patience_counter += 1
            tag = ""

        if epoch <= 10 or epoch % 10 == 0 or improved or patience_counter >= tcfg["patience"]:
            print(
                f"Epoch {epoch:3d} | "
                f"train {train_loss:.4f} (box={train_comp['box']:.4f} conf={train_comp['conf']:.4f} match={train_comp['match']:.4f} alpha={train_comp['alpha']:.4f}) | "
                f"val {val_loss:.4f} (box={val_comp['box']:.4f} conf={val_comp['conf']:.4f} match={val_comp['match']:.4f} alpha={val_comp['alpha']:.4f}) | "
                f"lr={lr:.1e} | {dt:.1f}s{tag}"
            )

        if patience_counter >= tcfg["patience"]:
            print(f"\nEarly stopping at epoch {epoch}")
            break

    print(f"\nBest val loss: {best_val_loss:.4f}")
    print(f"Model saved to {best_path}")

    # save history
    history_path = os.path.join(out_dir, "train_history.npy")
    np.save(history_path, history, allow_pickle=True)
    print(f"History saved to {history_path}")


if __name__ == "__main__":
    main()
