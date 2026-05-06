"""
QC-GAF synthetic dataset generator.

Loads nuScenes mini GT annotations and creates fusion training samples
by adding structured noise under 4 scenario profiles:
  normal / glare / vibration / rain_fog

Each sample: (cam_dets, lidar_dets, quality, gt_boxes, gt_match, cam_mask, lidar_mask, scenario_id)
Padded to M_max=N_max=max_dets.
"""

import glob
import json
import os
import random

import numpy as np
import torch
import yaml
from torch.utils.data import Dataset


# ── nuScenes GT loading ─────────────────────────────────────────────────────

def load_nuscenes_frames(nuscenes_root: str, version: str = "v1.0-mini"):
    """Load per-sample GT boxes from nuScenes mini.

    Returns:
        frames: list of dicts, each with:
            - boxes: np.array (K, 6) [x,y,z,w,h,l]
            - scene: str scene name
    """
    ann_dir = os.path.join(nuscenes_root, version)

    with open(os.path.join(ann_dir, "scene.json")) as f:
        scenes = {s["token"]: s["name"] for s in json.load(f)}

    with open(os.path.join(ann_dir, "sample.json")) as f:
        samples_raw = json.load(f)
    samples_by_token = {s["token"]: s for s in samples_raw}

    with open(os.path.join(ann_dir, "sample_annotation.json")) as f:
        annotations = json.load(f)

    # group annotations by sample_token
    ann_by_sample = {}
    for ann in annotations:
        st = ann["sample_token"]
        if st not in ann_by_sample:
            ann_by_sample[st] = []
        ann_by_sample[st].append(ann)

    frames = []
    for s_token, sample in samples_by_token.items():
        scene_name = scenes.get(sample["scene_token"], "unknown")
        anns = ann_by_sample.get(s_token, [])
        if not anns:
            continue
        boxes = []
        for ann in anns:
            t = ann["translation"]  # [x, y, z]
            s = ann["size"]         # [w, l, h] in nuScenes
            boxes.append([t[0], t[1], t[2], s[0], s[2], s[1]])  # [x,y,z,w,h,l]
        frames.append({
            "boxes": np.array(boxes, dtype=np.float32),
            "scene": scene_name,
        })

    return frames


# ── Noise / quality simulation ──────────────────────────────────────────────

SCENARIOS = ["normal", "glare", "vibration", "rain_fog"]
SCENARIO_ID = {s: i for i, s in enumerate(SCENARIOS)}


def _simulate_quality(scenario: str, rng: np.random.Generator) -> np.ndarray:
    """Generate a 7-dim quality vector for the given scenario.

    Indices: [brightness, edge, depth_consistency, yolo_conf,
              point_density, imu_vib, depth_temporal]
    """
    q = rng.uniform(0.7, 1.0, size=7).astype(np.float32)

    if scenario == "glare":
        q[0] = rng.uniform(0.05, 0.3)   # brightness
        q[1] = rng.uniform(0.1, 0.35)   # edge
        q[2] = rng.uniform(0.1, 0.35)   # depth_consistency
        q[3] = rng.uniform(0.2, 0.5)    # yolo_conf lower
    elif scenario == "vibration":
        q[5] = rng.uniform(0.05, 0.3)   # imu_vib
        q[6] = rng.uniform(0.15, 0.4)   # depth_temporal
    elif scenario == "rain_fog":
        q[4] = rng.uniform(0.05, 0.3)   # point_density
        q[2] = rng.uniform(0.2, 0.45)   # depth_consistency

    return q


def _add_noise(boxes: np.ndarray, pos_std: float, size_std: float,
               drop_rate: float, vib_std: float,
               rng: np.random.Generator, max_dets: int):
    """Add Gaussian noise and random drops to GT boxes.

    Returns:
        noisy_boxes: (K', 6) after drops
        kept_indices: original GT indices that survived
    """
    K = len(boxes)
    # random drop
    keep_mask = rng.random(K) > drop_rate
    if keep_mask.sum() == 0:
        keep_mask[rng.integers(K)] = True  # keep at least one

    kept_idx = np.where(keep_mask)[0]
    kept = boxes[kept_idx].copy()

    # position noise
    kept[:, :3] += rng.normal(0, pos_std, size=kept[:, :3].shape)
    # vibration offset (common shift for all boxes)
    if vib_std > 0:
        vib = rng.normal(0, vib_std, size=3).astype(np.float32)
        kept[:, :3] += vib
    # size noise
    kept[:, 3:6] += rng.normal(0, size_std, size=kept[:, 3:6].shape)
    kept[:, 3:6] = np.maximum(kept[:, 3:6], 0.1)  # clamp sizes

    # truncate to max_dets
    if len(kept) > max_dets:
        sel = rng.choice(len(kept), max_dets, replace=False)
        sel.sort()
        kept = kept[sel]
        kept_idx = kept_idx[sel]

    return kept, kept_idx


def _make_cam_features(boxes: np.ndarray, quality: np.ndarray,
                       rng: np.random.Generator) -> np.ndarray:
    """Append 3 camera-specific features to make (K, 9).

    Features: [depth_pixel_count, depth_variance, valid_pixel_ratio]
    """
    K = len(boxes)
    # simulate depth pixel count (higher when quality is good), normalized to [0,1]
    dpc = rng.normal(0.5, 0.15, K) * (0.3 + 0.7 * quality[2])
    dpc = np.clip(dpc, 0.01, 1.0)
    # depth variance (small is good), normalized
    dv = rng.exponential(0.15, K) * (2.0 - quality[2])
    dv = np.clip(dv, 0.0, 1.0)
    # valid pixel ratio
    vpr = rng.beta(5, 2, K) * (0.5 + 0.5 * quality[0])

    extra = np.stack([dpc, dv, vpr], axis=-1).astype(np.float32)
    return np.concatenate([boxes, extra], axis=-1)


def _make_lidar_features(boxes: np.ndarray, quality: np.ndarray,
                         rng: np.random.Generator) -> np.ndarray:
    """Append 4 LiDAR-specific features to make (K, 10).

    Features: [cluster_point_count, std_x, std_y, std_z]
    """
    K = len(boxes)
    # cluster point count, normalized to [0,1]
    cpc = rng.normal(0.4, 0.15, K) * (0.3 + 0.7 * quality[4])
    cpc = np.clip(cpc, 0.01, 1.0)
    # spatial std, normalized
    std_x = rng.exponential(0.1, K) * (2.0 - quality[4])
    std_x = np.clip(std_x, 0.0, 1.0)
    std_y = rng.exponential(0.1, K) * (2.0 - quality[4])
    std_y = np.clip(std_y, 0.0, 1.0)
    std_z = rng.exponential(0.08, K) * (2.0 - quality[4])
    std_z = np.clip(std_z, 0.0, 1.0)

    extra = np.stack([cpc, std_x, std_y, std_z], axis=-1).astype(np.float32)
    return np.concatenate([boxes, extra], axis=-1)


# ── Sample generation ────────────────────────────────────────────────────────

def generate_sample(gt_boxes: np.ndarray, scenario: str,
                    noise_cfg: dict, max_dets: int,
                    rng: np.random.Generator):
    """Generate one fusion training sample from GT boxes.

    Key design choices:
    1. Select a common GT pool (≤ max_dets) first so cam/lidar share objects
    2. Normalize xyz by subtracting frame center (zero-mean positions)
    3. This ensures high match density and learnable coordinate ranges

    Returns dict with all tensors needed for training.
    """
    cfg = noise_cfg[scenario]
    cam_pos_std = cfg["cam_pos_std"]
    lidar_pos_std = cfg["lidar_pos_std"]
    cam_size_std = cfg["cam_size_std"]
    lidar_size_std = cfg["lidar_size_std"]
    cam_drop = cfg["cam_drop_rate"]
    lidar_drop = cfg["lidar_drop_rate"]
    vib_std = cfg.get("vib_offset_std", 0.0)

    quality = _simulate_quality(scenario, rng)

    # Step 1: Select common GT pool (≤ max_dets)
    K = len(gt_boxes)
    if K > max_dets:
        pool_idx = rng.choice(K, max_dets, replace=False)
        pool_idx.sort()
        gt_pool = gt_boxes[pool_idx]
    else:
        gt_pool = gt_boxes.copy()

    # Step 2: Normalize positions — subtract frame center
    center = gt_pool[:, :3].mean(axis=0)
    gt_pool_centered = gt_pool.copy()
    gt_pool_centered[:, :3] -= center

    # Step 3: Generate noisy detections from centered pool
    cam_raw, cam_kept = _add_noise(
        gt_pool_centered, cam_pos_std, cam_size_std, cam_drop, vib_std, rng, max_dets
    )
    lidar_raw, lidar_kept = _add_noise(
        gt_pool_centered, lidar_pos_std, lidar_size_std, lidar_drop, vib_std, rng, max_dets
    )

    M = len(cam_raw)
    N = len(lidar_raw)

    # camera features (M, 9)
    cam_dets = _make_cam_features(cam_raw, quality, rng)
    # lidar features (N, 10)
    lidar_dets = _make_lidar_features(lidar_raw, quality, rng)

    # GT match matrix (M, N): 1 if cam_i and lidar_j come from same GT box
    gt_match = np.zeros((max_dets, max_dets), dtype=np.float32)
    for i, ci in enumerate(cam_kept):
        for j, lj in enumerate(lidar_kept):
            if ci == lj:
                gt_match[i, j] = 1.0

    # GT fused boxes: ideal centered output for each camera detection
    gt_fused = np.zeros((max_dets, 7), dtype=np.float32)
    for i, ci in enumerate(cam_kept):
        gt_fused[i, :6] = gt_pool_centered[ci]
        # confidence: 1 if matched to any lidar, else lower
        gt_fused[i, 6] = 1.0 if ci in lidar_kept else 0.5

    # pad to max_dets
    cam_padded = np.zeros((max_dets, 9), dtype=np.float32)
    lidar_padded = np.zeros((max_dets, 10), dtype=np.float32)
    cam_padded[:M] = cam_dets
    lidar_padded[:N] = lidar_dets

    cam_mask = np.zeros(max_dets, dtype=bool)
    lidar_mask = np.zeros(max_dets, dtype=bool)
    cam_mask[:M] = True
    lidar_mask[:N] = True

    return {
        "cam_dets": cam_padded,
        "lidar_dets": lidar_padded,
        "quality": quality,
        "gt_fused": gt_fused,
        "gt_match": gt_match,
        "cam_mask": cam_mask,
        "lidar_mask": lidar_mask,
        "scenario_id": SCENARIO_ID[scenario],
        "num_cam": M,
        "num_lidar": N,
    }


# ── Dataset class ────────────────────────────────────────────────────────────

class QCGAFDataset(Dataset):
    """Precomputed fusion training dataset."""

    def __init__(self, samples: list):
        self.samples = samples

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        s = self.samples[idx]
        return {k: torch.tensor(v) for k, v in s.items()}


def _normalize_loaded_sample(sample):
    if hasattr(sample, "item"):
        sample = sample.item()
    out = {}
    for k, v in sample.items():
        if isinstance(v, np.ndarray):
            out[k] = v
        elif isinstance(v, np.generic):
            out[k] = np.array(v)
        else:
            out[k] = np.asarray(v)
    return out


def load_exported_gazebo_samples(sample_paths):
    samples = []
    for path in sample_paths:
        arr = np.load(path, allow_pickle=True)
        for sample in arr:
            samples.append(_normalize_loaded_sample(sample))
    return samples


def build_datasets(config: dict):
    """Build train/test datasets from nuScenes GT and/or exported Gazebo samples.

    Returns:
        train_ds, test_ds: QCGAFDataset
    """
    data_cfg = config["data"]
    train_samples = []
    test_samples = []

    if data_cfg.get("use_nuscenes", True):
        noise_cfg = config["noise"]
        model_cfg = config["model"]
        max_dets = model_cfg["max_cam_dets"]

        nuscenes_root = data_cfg["nuscenes_root"]
        version = data_cfg["nuscenes_version"]
        repeats = data_cfg["repeats_per_frame"]
        n_train_scenes = data_cfg["train_scenes"]

        print(f"Loading nuScenes {version} from {nuscenes_root} ...")
        frames = load_nuscenes_frames(nuscenes_root, version)
        print(f"Loaded {len(frames)} frames")

        scene_names = sorted(set(f["scene"] for f in frames))
        train_scene_set = set(scene_names[:n_train_scenes])
        test_scene_set = set(scene_names[n_train_scenes:])
        print(f"Train scenes ({len(train_scene_set)}): {sorted(train_scene_set)}")
        print(f"Test scenes ({len(test_scene_set)}): {sorted(test_scene_set)}")

        sw = data_cfg["scenario_weights"]
        scenarios = list(sw.keys())
        weights = [sw[s] for s in scenarios]

        rng = np.random.default_rng(42)

        for frame in frames:
            is_train = frame["scene"] in train_scene_set
            gt = frame["boxes"]
            if len(gt) == 0:
                continue

            for _ in range(repeats):
                scenario = rng.choice(scenarios, p=weights)
                sample = generate_sample(gt, scenario, noise_cfg, max_dets, rng)
                if is_train:
                    train_samples.append(sample)
                else:
                    test_samples.append(sample)

    gazebo_sample_paths = []
    gazebo_sample_paths.extend(data_cfg.get("gazebo_sample_files", []))
    if data_cfg.get("gazebo_sample_glob"):
        gazebo_sample_paths.extend(glob.glob(data_cfg["gazebo_sample_glob"]))
    gazebo_sample_paths = sorted(set(gazebo_sample_paths))

    if gazebo_sample_paths:
        print(f"Loading Gazebo exported samples from {len(gazebo_sample_paths)} files")
        gazebo_samples = load_exported_gazebo_samples(gazebo_sample_paths)
        rng = random.Random(data_cfg.get("gazebo_split_seed", 42))
        rng.shuffle(gazebo_samples)
        val_ratio = float(data_cfg.get("gazebo_val_ratio", 0.2))
        split_idx = int(len(gazebo_samples) * (1.0 - val_ratio))
        split_idx = max(1, min(split_idx, len(gazebo_samples)))
        gazebo_train = gazebo_samples[:split_idx]
        gazebo_val = gazebo_samples[split_idx:]
        if not gazebo_val and gazebo_train:
            gazebo_val = gazebo_train[-1:]
            gazebo_train = gazebo_train[:-1] or gazebo_train

        mode = data_cfg.get("gazebo_mode", "append")
        if mode == "only":
            train_samples = list(gazebo_train)
            test_samples = list(gazebo_val)
        else:
            train_samples.extend(gazebo_train)
            test_samples.extend(gazebo_val)
        print(f"Loaded {len(gazebo_train)} Gazebo train and {len(gazebo_val)} Gazebo val samples")

    print(f"Generated {len(train_samples)} train, {len(test_samples)} test samples")
    return QCGAFDataset(train_samples), QCGAFDataset(test_samples)


# ── CLI ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="QC-GAF dataset generator")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--stats", action="store_true", help="Print dataset statistics")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    train_ds, test_ds = build_datasets(config)
    print(f"\nTrain: {len(train_ds)}, Test: {len(test_ds)}")

    if args.stats:
        # print per-scenario counts
        from collections import Counter
        train_scenarios = Counter(s["scenario_id"] for s in train_ds.samples)
        test_scenarios = Counter(s["scenario_id"] for s in test_ds.samples)
        for sid, name in enumerate(SCENARIOS):
            print(f"  {name}: train={train_scenarios.get(sid, 0)}, test={test_scenarios.get(sid, 0)}")

    # verify a sample
    s = train_ds[0]
    print(f"\nSample keys: {list(s.keys())}")
    for k, v in s.items():
        print(f"  {k}: shape={v.shape}, dtype={v.dtype}")
