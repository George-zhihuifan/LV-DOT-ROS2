"""Trajectory dataset with sliding window sampling.

Supports three data sources:
- ETH/UCY public pedestrian datasets (5 scenes, 2D)
- LV-DOT collected CSV trajectories (2D)
- nuScenes mini 3D tracking data (3D)
"""

import argparse
import os
import urllib.request

import numpy as np
import torch
from torch.utils.data import Dataset


_RAW_BASE = "https://raw.githubusercontent.com/abduallahmohamed/Social-STGCNN/master/datasets/raw/all_data"
ETH_UCY_SCENES = {
    "eth": {"out": "eth_eth.txt", "url": f"{_RAW_BASE}/biwi_eth.txt"},
    "hotel": {"out": "eth_hotel.txt", "url": f"{_RAW_BASE}/biwi_hotel.txt"},
    "univ": {"out": "ucy_univ.txt", "url": f"{_RAW_BASE}/students001.txt"},
    "zara1": {"out": "ucy_zara1.txt", "url": f"{_RAW_BASE}/crowds_zara01.txt"},
    "zara2": {"out": "ucy_zara2.txt", "url": f"{_RAW_BASE}/crowds_zara02.txt"},
}

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data", "eth_ucy")


# ---------------------------------------------------------------------------
# ETH/UCY helpers (kept for backward compatibility, but not used in 3D mode)
# ---------------------------------------------------------------------------

def download_eth_ucy(data_dir=None):
    """Download ETH/UCY raw trajectory files from Social-STGCNN repo."""
    if data_dir is None:
        data_dir = DATA_DIR
    os.makedirs(data_dir, exist_ok=True)
    for scene, info in ETH_UCY_SCENES.items():
        out_path = os.path.join(data_dir, info["out"])
        if os.path.exists(out_path):
            print(f"  {scene}: already exists, skipping")
            continue
        url = info["url"]
        print(f"  Downloading {scene} from {url} ...")
        try:
            urllib.request.urlretrieve(url, out_path)
            lines = sum(1 for _ in open(out_path))
            print(f"  {scene}: saved {lines} lines to {out_path}")
        except Exception as e:
            print(f"  {scene}: download failed ({e}), please download manually")


def load_eth_ucy_scene(filepath):
    """Load a single ETH/UCY scene file.

    Format: frame_id pedestrian_id x y (tab or space separated)
    Returns dict {ped_id: [(frame, x, y), ...]}, sorted by frame.
    """
    trajectories = {}
    with open(filepath, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            frame_id = int(float(parts[0]))
            ped_id = int(float(parts[1]))
            x = float(parts[2])
            y = float(parts[3])
            if ped_id not in trajectories:
                trajectories[ped_id] = []
            trajectories[ped_id].append((frame_id, x, y))
    for pid in trajectories:
        trajectories[pid].sort(key=lambda t: t[0])
    return trajectories


def load_lvdot_csv(filepath):
    """Load LV-DOT collected CSV trajectory file.

    Format: timestamp,track_id,x,y,vx,vy,w,h
    Returns dict {track_id: [(ts, x, y, vx, vy, w, h), ...]}.
    """
    import csv

    trajectories = {}
    with open(filepath, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tid = int(row["track_id"])
            entry = (
                float(row["timestamp"]),
                float(row["x"]),
                float(row["y"]),
                float(row["vx"]),
                float(row["vy"]),
                float(row["w"]),
                float(row["h"]),
            )
            if tid not in trajectories:
                trajectories[tid] = []
            trajectories[tid].append(entry)
    for tid in trajectories:
        trajectories[tid].sort(key=lambda t: t[0])
    return trajectories


def compute_increments_eth(positions, seq_len):
    """Compute incremental features from ETH/UCY positions (2D, kept for compat)."""
    N = len(positions)
    if N < seq_len + 2:
        return []

    velocities = np.zeros((N, 2))
    velocities[1:] = positions[1:] - positions[:-1]
    dx = positions[1:] - positions[:-1]
    dvx = velocities[1:] - velocities[:-1]

    samples = []
    for i in range(N - seq_len - 1):
        inp = np.zeros((seq_len, 6))
        for t in range(seq_len):
            idx = i + t + 1
            inp[t, 0] = dx[idx - 1, 0] if idx - 1 >= 0 else 0.0
            inp[t, 1] = dx[idx - 1, 1] if idx - 1 >= 0 else 0.0
            inp[t, 2] = dvx[idx, 0] if idx < len(dvx) else 0.0
            inp[t, 3] = dvx[idx, 1] if idx < len(dvx) else 0.0
        label_idx = i + seq_len
        label = dx[label_idx]
        samples.append((inp.astype(np.float32), label.astype(np.float32)))
    return samples


def compute_increments_lvdot(traj_data, seq_len):
    """Compute incremental features from LV-DOT trajectory data (2D, kept for compat)."""
    arr = np.array(traj_data)
    N = len(arr)
    if N < seq_len + 2:
        return []

    x = arr[:, 1]
    y = arr[:, 2]
    vx = arr[:, 3]
    vy = arr[:, 4]
    w = arr[:, 5]
    h = arr[:, 6]

    dx = np.diff(x)
    dy = np.diff(y)
    dvx = np.diff(vx)
    dvy = np.diff(vy)

    samples = []
    for i in range(N - seq_len - 1):
        inp = np.zeros((seq_len, 6), dtype=np.float32)
        for t in range(seq_len):
            idx = i + t
            inp[t, 0] = dx[idx]
            inp[t, 1] = dy[idx]
            inp[t, 2] = dvx[idx]
            inp[t, 3] = dvy[idx]
            inp[t, 4] = w[idx + 1]
            inp[t, 5] = h[idx + 1]
        label_idx = i + seq_len
        label = np.array([dx[label_idx], dy[label_idx]], dtype=np.float32)
        samples.append((inp, label))
    return samples


# ---------------------------------------------------------------------------
# nuScenes 3D trajectory loading
# ---------------------------------------------------------------------------

def load_nuscenes_trajectories(nuscenes_root, version="v1.0-mini",
                               min_track_length=12):
    """Extract 3D tracking trajectories from nuScenes.

    Loads only scene.json, sample.json, sample_annotation.json (skips the
    heavy sample_data.json / ego_pose.json that the devkit would load).

    Returns:
        tracks: dict  {scene_name: {instance_token: {
            'positions': np.array (N,3),  # x, y, z in global frame
            'velocities': np.array (N,3),
            'sizes': np.array (N,3),      # w, h, l
            'timestamps': np.array (N,),
            'category': str
        }}}
    """
    import json as _json

    ann_dir = os.path.join(nuscenes_root, version)

    print(f"  Loading scene.json ...")
    with open(os.path.join(ann_dir, "scene.json")) as f:
        scenes = _json.load(f)

    print(f"  Loading sample.json ...")
    with open(os.path.join(ann_dir, "sample.json")) as f:
        samples_raw = _json.load(f)
    sample_by_token = {s["token"]: s for s in samples_raw}

    print(f"  Loading sample_annotation.json ...")
    with open(os.path.join(ann_dir, "sample_annotation.json")) as f:
        annotations = _json.load(f)

    # Build instance_token → category_name lookup
    # (v1.0-mini has category_name directly, trainval needs instance→category)
    inst_to_cat = {}
    if annotations and "category_name" not in annotations[0]:
        print(f"  Loading instance.json + category.json for category names ...")
        with open(os.path.join(ann_dir, "instance.json")) as f:
            instances = _json.load(f)
        with open(os.path.join(ann_dir, "category.json")) as f:
            categories = _json.load(f)
        cat_by_token = {c["token"]: c["name"] for c in categories}
        inst_to_cat = {i["token"]: cat_by_token.get(i["category_token"], "unknown")
                       for i in instances}
        del instances, categories

    # Index annotations by token and by sample_token
    ann_by_token = {a["token"]: a for a in annotations}
    ann_by_sample = {}
    for ann in annotations:
        st = ann["sample_token"]
        if st not in ann_by_sample:
            ann_by_sample[st] = []
        ann_by_sample[st].append(ann)
    del annotations  # free memory

    print(f"  Loaded {len(scenes)} scenes, {len(sample_by_token)} samples, "
          f"{len(ann_by_token)} annotations")

    def _box_velocity(ann):
        """Compute velocity from consecutive annotations."""
        prev_token = ann.get("prev", "")
        if not prev_token:
            return np.array([0.0, 0.0, 0.0])
        prev_ann = ann_by_token.get(prev_token)
        if prev_ann is None:
            return np.array([0.0, 0.0, 0.0])
        cur_sample = sample_by_token.get(ann["sample_token"])
        prev_sample = sample_by_token.get(prev_ann["sample_token"])
        if cur_sample is None or prev_sample is None:
            return np.array([0.0, 0.0, 0.0])
        dt = 1e-6 * (cur_sample["timestamp"] - prev_sample["timestamp"])
        if abs(dt) < 1e-6:
            return np.array([0.0, 0.0, 0.0])
        pos_diff = np.array(ann["translation"]) - np.array(prev_ann["translation"])
        return pos_diff / dt

    tracks = {}

    for scene in scenes:
        scene_name = scene["name"]
        scene_tracks = {}

        # Collect all sample tokens in this scene
        sample_token = scene["first_sample_token"]
        sample_tokens = []
        while sample_token:
            sample_tokens.append(sample_token)
            sample = sample_by_token[sample_token]
            sample_token = sample["next"] if sample["next"] else None

        # For each sample, collect annotations grouped by instance
        instance_data = {}
        for s_idx, s_token in enumerate(sample_tokens):
            sample = sample_by_token[s_token]
            for ann in ann_by_sample.get(s_token, []):
                inst_token = ann["instance_token"]

                if inst_token not in instance_data:
                    instance_data[inst_token] = []

                vel = _box_velocity(ann)

                cat_name = ann.get("category_name",
                                    inst_to_cat.get(ann["instance_token"], "unknown"))

                instance_data[inst_token].append({
                    "sample_idx": s_idx,
                    "translation": np.array(ann["translation"]),
                    "size": np.array(ann["size"]),
                    "velocity": vel,
                    "category": cat_name,
                    "timestamp": sample["timestamp"],
                })

        # Convert to trajectory arrays, filter by length
        for inst_token, data_list in instance_data.items():
            if len(data_list) < min_track_length:
                continue

            data_list.sort(key=lambda d: d["sample_idx"])

            positions = np.array([d["translation"] for d in data_list])
            velocities = np.array([d["velocity"] for d in data_list])
            sizes = np.array([[d["size"][0], d["size"][2], d["size"][1]]
                              for d in data_list])
            timestamps = np.array([d["timestamp"] for d in data_list])
            category = data_list[0]["category"]

            scene_tracks[inst_token] = {
                "positions": positions,
                "velocities": velocities,
                "sizes": sizes,
                "timestamps": timestamps,
                "category": category,
            }

        if scene_tracks:
            tracks[scene_name] = scene_tracks

    return tracks


def compute_increments_3d(positions, velocities, sizes, seq_len):
    """Compute 3D incremental features for GRU input.

    Input features per frame: [dx, dy, dz, dvx, dvy, dvz, w, h, l]  (9-dim)
    Label: [dx, dy, dz] for the next frame                           (3-dim)

    Args:
        positions: (N, 3) array of [x, y, z]
        velocities: (N, 3) array of [vx, vy, vz]
        sizes: (N, 3) array of [w, h, l]
        seq_len: sliding window length

    Returns:
        list of (input_seq, label) tuples
    """
    N = len(positions)
    if N < seq_len + 2:
        return []

    # Position deltas
    dpos = positions[1:] - positions[:-1]  # (N-1, 3)

    # Velocity deltas
    dvel = velocities[1:] - velocities[:-1]  # (N-1, 3)

    samples = []
    for i in range(N - seq_len - 1):
        inp = np.zeros((seq_len, 9), dtype=np.float32)
        for t in range(seq_len):
            idx = i + t  # index into dpos/dvel (starts from 0)
            inp[t, 0:3] = dpos[idx]    # dx, dy, dz
            inp[t, 3:6] = dvel[idx]    # dvx, dvy, dvz
            inp[t, 6:9] = sizes[idx + 1]  # w, h, l at the current frame

        label_idx = i + seq_len
        label = dpos[label_idx].astype(np.float32)  # [dx, dy, dz]
        samples.append((inp, label))

    return samples


def build_nuscenes_datasets(config, seq_len=10):
    """Build train/test datasets from nuScenes mini.

    Uses scene-based split: last 2 scenes for test, rest for train.

    Returns:
        (train_dataset, test_dataset, test_trajectories)
        where test_trajectories is a list of (positions_3d,) arrays for evaluation.
    """
    data_cfg = config["data"]
    nuscenes_root = data_cfg["nuscenes_root"]
    version = data_cfg["nuscenes_version"]
    min_track_length = data_cfg.get("min_track_length", 12)

    print(f"Loading nuScenes {version} from {nuscenes_root} ...")
    all_tracks = load_nuscenes_trajectories(
        nuscenes_root, version=version, min_track_length=min_track_length
    )

    scene_names = sorted(all_tracks.keys())
    n_scenes = len(scene_names)
    print(f"Found {n_scenes} scenes with qualifying tracks")

    # Split: last 2 scenes for test
    n_test = max(1, n_scenes // 5)
    test_scenes = set(scene_names[-n_test:])
    train_scenes = set(scene_names[:-n_test])
    print(f"Train scenes ({len(train_scenes)}): {sorted(train_scenes)}")
    print(f"Test scenes ({len(test_scenes)}): {sorted(test_scenes)}")

    train_samples = []
    test_samples = []
    test_trajectories = []

    total_tracks = 0
    for scene_name, scene_tracks in all_tracks.items():
        is_test = scene_name in test_scenes
        for inst_token, track in scene_tracks.items():
            total_tracks += 1
            positions = track["positions"]
            velocities = track["velocities"]
            sizes = track["sizes"]

            samples = compute_increments_3d(positions, velocities, sizes, seq_len)

            if is_test:
                test_samples.extend(samples)
                if len(positions) >= seq_len + 2:
                    test_trajectories.append(positions)
            else:
                train_samples.extend(samples)

    print(f"Total tracks: {total_tracks}")
    print(f"Train samples: {len(train_samples)}, Test samples: {len(test_samples)}")
    print(f"Test trajectories: {len(test_trajectories)}")

    return (
        TrajectoryDataset(train_samples),
        TrajectoryDataset(test_samples),
        test_trajectories,
    )


# ---------------------------------------------------------------------------
# Common
# ---------------------------------------------------------------------------

class TrajectoryDataset(Dataset):
    """Trajectory dataset with sliding window sampling."""

    def __init__(self, samples):
        """
        Args:
            samples: list of (input_seq, label) tuples
        """
        self.samples = samples

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        inp, label = self.samples[idx]
        return torch.tensor(inp, dtype=torch.float32), torch.tensor(
            label, dtype=torch.float32
        )


def build_eth_ucy_datasets(seq_len=10, data_dir=None):
    """Build leave-one-out datasets for ETH/UCY.

    Returns dict: {test_scene: (train_dataset, test_dataset, test_trajectories)}
    where test_trajectories is a list of raw position arrays for evaluation.
    """
    if data_dir is None:
        data_dir = DATA_DIR

    all_scene_data = {}
    for scene, info in ETH_UCY_SCENES.items():
        fpath = os.path.join(data_dir, info["out"])
        if not os.path.exists(fpath):
            print(f"Warning: {fpath} not found. Run with --download first.")
            continue
        trajs = load_eth_ucy_scene(fpath)
        all_scene_data[scene] = trajs

    datasets = {}
    for test_scene in all_scene_data:
        train_samples = []
        test_samples = []
        test_trajectories = []

        for scene, trajs in all_scene_data.items():
            for pid, traj in trajs.items():
                positions = np.array([(t[1], t[2]) for t in traj])
                samples = compute_increments_eth(positions, seq_len)
                if scene == test_scene:
                    test_samples.extend(samples)
                    if len(positions) >= seq_len + 2:
                        test_trajectories.append(positions)
                else:
                    train_samples.extend(samples)

        if train_samples and test_samples:
            datasets[test_scene] = (
                TrajectoryDataset(train_samples),
                TrajectoryDataset(test_samples),
                test_trajectories,
            )
    return datasets


def build_lvdot_dataset(csv_path, seq_len=10, train_ratio=0.8):
    """Build train/test datasets from LV-DOT CSV file."""
    trajs = load_lvdot_csv(csv_path)
    all_samples = []
    test_trajectories = []

    for tid, traj in trajs.items():
        samples = compute_increments_lvdot(traj, seq_len)
        all_samples.extend(samples)
        arr = np.array(traj)
        positions = arr[:, 1:3]
        if len(positions) >= seq_len + 2:
            test_trajectories.append(positions)

    np.random.seed(42)
    indices = np.random.permutation(len(all_samples))
    split = int(len(indices) * train_ratio)
    train_samples = [all_samples[i] for i in indices[:split]]
    test_samples = [all_samples[i] for i in indices[split:]]

    return (
        TrajectoryDataset(train_samples),
        TrajectoryDataset(test_samples),
        test_trajectories,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dataset utilities")
    parser.add_argument("--download", action="store_true", help="Download ETH/UCY data")
    parser.add_argument("--data-dir", type=str, default=None)
    parser.add_argument("--nuscenes", action="store_true", help="Extract nuScenes trajectories")
    parser.add_argument("--config", type=str, default="config.yaml")
    args = parser.parse_args()

    if args.download:
        print("Downloading ETH/UCY datasets...")
        download_eth_ucy(args.data_dir)
        print("Done.")
    elif args.nuscenes:
        import yaml
        with open(args.config, "r") as f:
            config = yaml.safe_load(f)
        train_ds, test_ds, test_trajs = build_nuscenes_datasets(
            config, seq_len=config["gru"]["seq_len"]
        )
        print(f"\nDataset ready: {len(train_ds)} train, {len(test_ds)} test samples")
    else:
        print("Use --download to fetch ETH/UCY data files.")
        print("Use --nuscenes to extract nuScenes 3D trajectories.")
