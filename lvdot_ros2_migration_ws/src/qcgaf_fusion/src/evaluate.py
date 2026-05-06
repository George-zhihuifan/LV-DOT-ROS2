"""
QC-GAF offline evaluation.

Metrics:
  - Matching accuracy (soft match argmax vs GT)
  - Fused box RMSE on [x,y,z] and [w,h,l]
  - Per-scenario breakdown
"""

import os

import numpy as np
import torch
import yaml
from torch.utils.data import DataLoader

from dataset import QCGAFDataset, build_datasets, SCENARIOS
from model import QCGAF
from train import collate_fn


@torch.no_grad()
def evaluate(model, loader, device):
    """Run evaluation and collect per-sample metrics."""
    model.eval()
    results = []

    for batch in loader:
        cam = batch["cam_dets"].to(device)
        lidar = batch["lidar_dets"].to(device)
        q = batch["quality"].to(device)
        cm = batch["cam_mask"].to(device)
        lm = batch["lidar_mask"].to(device)
        gt_fused = batch["gt_fused"]
        gt_match = batch["gt_match"]
        scenario_ids = batch["scenario_id"]

        pred_boxes, pred_match = model(cam, lidar, q, cm, lm)
        pred_boxes = pred_boxes.cpu()
        pred_match = pred_match.cpu()

        B = cam.size(0)
        for i in range(B):
            m = batch["num_cam"][i].item()
            n = batch["num_lidar"][i].item()
            if m == 0:
                continue

            # matching accuracy: for each cam det, does argmax match GT?
            pm = pred_match[i, :m, :n]          # (m, n)
            gm = gt_match[i, :m, :n]            # (m, n)
            pred_assign = pm.argmax(dim=1)       # (m,)
            gt_assign = gm.argmax(dim=1)         # (m,)
            # only count cam dets that have a GT match
            has_match = gm.sum(dim=1) > 0        # (m,)
            if has_match.sum() > 0:
                match_correct = (pred_assign[has_match] == gt_assign[has_match]).float()
                match_acc = match_correct.mean().item()
            else:
                match_acc = float("nan")

            # box RMSE
            pb = pred_boxes[i, :m]               # (m, 7)
            gb = gt_fused[i, :m]                 # (m, 7)
            pos_err = (pb[:, :3] - gb[:, :3]).pow(2).mean(dim=0).sqrt()  # (3,) per-axis
            size_err = (pb[:, 3:6] - gb[:, 3:6]).pow(2).mean(dim=0).sqrt()
            pos_rmse = pos_err.mean().item()
            size_rmse = size_err.mean().item()

            # confidence MAE
            conf_pred = torch.sigmoid(pb[:, 6])
            conf_gt = gb[:, 6]
            conf_mae = (conf_pred - conf_gt).abs().mean().item()

            results.append({
                "scenario": scenario_ids[i].item(),
                "match_acc": match_acc,
                "pos_rmse": pos_rmse,
                "size_rmse": size_rmse,
                "conf_mae": conf_mae,
                "num_cam": m,
                "num_lidar": n,
            })

    return results


def print_results(results):
    """Print evaluation summary."""
    # overall
    valid = [r for r in results if not np.isnan(r["match_acc"])]
    print(f"\n{'='*70}")
    print(f"Overall ({len(valid)} samples with matches, {len(results)} total)")
    print(f"{'='*70}")
    avg_match = np.mean([r["match_acc"] for r in valid])
    avg_pos = np.mean([r["pos_rmse"] for r in results])
    avg_size = np.mean([r["size_rmse"] for r in results])
    avg_conf = np.mean([r["conf_mae"] for r in results])
    print(f"  Matching accuracy:   {avg_match:.4f} ({avg_match*100:.1f}%)")
    print(f"  Position RMSE:       {avg_pos:.4f} m")
    print(f"  Size RMSE:           {avg_size:.4f} m")
    print(f"  Confidence MAE:      {avg_conf:.4f}")

    # per-scenario
    print(f"\n{'─'*70}")
    print(f"Per-scenario breakdown:")
    print(f"{'─'*70}")
    print(f"  {'Scenario':<12} {'Count':>6} {'Match%':>8} {'PosRMSE':>10} {'SizeRMSE':>10} {'ConfMAE':>10}")
    for sid, name in enumerate(SCENARIOS):
        sc = [r for r in results if r["scenario"] == sid]
        if not sc:
            continue
        sc_valid = [r for r in sc if not np.isnan(r["match_acc"])]
        ma = np.mean([r["match_acc"] for r in sc_valid]) if sc_valid else 0
        pr = np.mean([r["pos_rmse"] for r in sc])
        sr = np.mean([r["size_rmse"] for r in sc])
        cm = np.mean([r["conf_mae"] for r in sc])
        print(f"  {name:<12} {len(sc):>6} {ma*100:>7.1f}% {pr:>10.4f} {sr:>10.4f} {cm:>10.4f}")

    # verification checks
    print(f"\n{'─'*70}")
    print("Verification:")
    ok_match = avg_match > 0.9
    ok_rmse = avg_pos < 0.15
    print(f"  Matching accuracy > 90%:  {'PASS' if ok_match else 'FAIL'} ({avg_match*100:.1f}%)")
    print(f"  Position RMSE < 0.15m:    {'PASS' if ok_rmse else 'FAIL'} ({avg_pos:.4f}m)")
    print(f"{'='*70}")

    return {"match_acc": avg_match, "pos_rmse": avg_pos, "size_rmse": avg_size}


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Evaluate QC-GAF")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--checkpoint", default=None,
                        help="Model checkpoint (default: outputs/best_model.pt)")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # load data
    _, test_ds = build_datasets(config)
    test_loader = DataLoader(test_ds, batch_size=64, shuffle=False,
                             collate_fn=collate_fn, num_workers=0)

    # load model
    ckpt_path = args.checkpoint or os.path.join(
        os.path.dirname(__file__), "..", "outputs", "best_model.pt"
    )
    print(f"Loading checkpoint: {ckpt_path}")
    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)

    mcfg = config["model"]
    model = QCGAF(
        cam_dim=mcfg["cam_dim"], lidar_dim=mcfg["lidar_dim"],
        quality_dim=mcfg["quality_dim"], hidden_dim=mcfg["hidden_dim"],
        feat_dim=mcfg["feat_dim"], output_dim=mcfg["output_dim"],
    ).to(device)
    model.load_state_dict(ckpt["model_state_dict"])
    print(f"Loaded model from epoch {ckpt['epoch']} (val_loss={ckpt['val_loss']:.4f})")

    # evaluate
    results = evaluate(model, test_loader, device)
    metrics = print_results(results)

    # save results
    out_dir = os.path.join(os.path.dirname(__file__), "..", "outputs")
    np.save(os.path.join(out_dir, "eval_results.npy"), results, allow_pickle=True)
    print(f"\nResults saved to {out_dir}/eval_results.npy")


if __name__ == "__main__":
    main()
