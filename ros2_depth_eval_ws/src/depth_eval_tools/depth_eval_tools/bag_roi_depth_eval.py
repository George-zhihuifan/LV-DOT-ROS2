import argparse
import csv
from pathlib import Path
from typing import Iterable

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml
from depth_eval_tools.roi_eval_common import depth_image_to_array, evaluate_roi, load_targets, summarize


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Evaluate ROI depth quality directly from a ros2 bag depth topic.'
    )
    parser.add_argument('bag_dir', type=Path, help='Path to a ros2 bag directory.')
    parser.add_argument('gt_csv', type=Path, help='GT ROI CSV from experiment_gt_export.')
    parser.add_argument(
        '--topic',
        default='/rgbd_camera/depth_image',
        help='Depth image topic inside the bag.',
    )
    parser.add_argument(
        '--max-frames',
        type=int,
        default=1,
        help='Maximum number of depth frames to evaluate.',
    )
    parser.add_argument(
        '--far-clip-m',
        type=float,
        default=12.0,
        help='Far clip threshold used for saturation statistics.',
    )
    parser.add_argument(
        '--foreground-tolerance-m',
        type=float,
        default=0.35,
        help='Depth tolerance around GT treated as foreground support.',
    )
    parser.add_argument(
        '--output-csv',
        type=Path,
        default=None,
        help='Optional per-target output CSV path.',
    )
    return parser.parse_args()


def infer_storage_id(bag_dir: Path) -> str:
    metadata = bag_dir / 'metadata.yaml'
    if metadata.exists():
        with metadata.open('r', encoding='utf-8') as handle:
            parsed = yaml.safe_load(handle) or {}
        info = parsed.get('rosbag2_bagfile_information', {})
        storage_id = info.get('storage_identifier')
        if storage_id:
            return str(storage_id)
    if list(bag_dir.glob('*.db3')):
        return 'sqlite3'
    if list(bag_dir.glob('*.mcap')):
        return 'mcap'
    return ''


def iter_depth_frames(bag_dir: Path, topic: str, max_frames: int) -> Iterable[np.ndarray]:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=infer_storage_id(bag_dir))
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise SystemExit(f'Topic not found in bag: {topic}')
    msg_type = get_message(topic_types[topic])

    frames = 0
    while reader.has_next() and frames < max_frames:
        current_topic, data, _ = reader.read_next()
        if current_topic != topic:
            continue
        yield depth_image_to_array(deserialize_message(data, msg_type))
        frames += 1

    if frames == 0:
        raise SystemExit(f'No messages found on topic: {topic}')


def main() -> None:
    args = parse_args()
    targets = load_targets(args.gt_csv)

    rows: list[dict] = []
    for depth_m in iter_depth_frames(args.bag_dir, args.topic, args.max_frames):
        for target in targets:
            metrics = evaluate_roi(
                depth_m=depth_m,
                gt_depth_m=target.gt_depth_m,
                xmin=target.roi_xmin_px,
                ymin=target.roi_ymin_px,
                xmax=target.roi_xmax_px,
                ymax=target.roi_ymax_px,
                far_clip_m=args.far_clip_m,
                tolerance_m=args.foreground_tolerance_m,
            )
            rows.append({
                'target_name': target.name,
                'range_group': target.expected_range_group,
                'gt_depth_m': target.gt_depth_m,
                **metrics,
            })

    summary = summarize(rows)

    for row in rows + [summary]:
        print(
            ', '.join(
                [
                    f"target={row['target_name']}",
                    f"group={row['range_group']}",
                    f"roi_valid_ratio={row['roi_valid_ratio']:.4f}",
                    f"center_valid_ratio={row['center_valid_ratio']:.4f}",
                    f"foreground_support_ratio={row['foreground_support_ratio']:.4f}",
                    f"foreground_purity_ratio={row['foreground_purity_ratio']:.4f}",
                    f"abs_error_m={row['abs_error_m']:.4f}",
                    f"saturation_ratio={row['saturation_ratio']:.4f}",
                ]
            )
        )

    if args.output_csv is not None:
        args.output_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.output_csv.open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows + [summary])


if __name__ == '__main__':
    main()
