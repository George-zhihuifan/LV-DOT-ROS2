import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np


METRIC_KEYS = [
    'roi_valid_ratio',
    'center_valid_ratio',
    'foreground_support_ratio',
    'foreground_purity_ratio',
    'abs_error_m',
    'saturation_ratio',
    'min_depth_m',
    'p10_depth_m',
    'p25_depth_m',
    'p50_depth_m',
    'p75_depth_m',
    'median_depth_m',
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Summarize live ROI depth evaluation CSV by range group.')
    parser.add_argument('input_csv', type=Path, help='Input live ROI evaluation CSV')
    parser.add_argument('output_csv', type=Path, help='Output summary CSV')
    parser.add_argument('--output-md', type=Path, default=None, help='Optional markdown summary output')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    groups = defaultdict(list)
    with args.input_csv.open('r', encoding='utf-8', newline='') as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            groups[row['range_group']].append(row)

    ordered_groups = ['near', 'mid', 'far']
    summaries = []
    for group in ordered_groups:
        rows = groups.get(group, [])
        if not rows:
            continue
        summary = {
            'range_group': group,
            'samples': len(rows),
        }
        for key in METRIC_KEYS:
            values = np.array([float(row.get(key, 'nan')) for row in rows], dtype=float)
            summary[key] = float(np.nanmean(values))
        summaries.append(summary)

    if not summaries:
        raise SystemExit('No summary rows were produced.')

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0].keys()))
        writer.writeheader()
        writer.writerows(summaries)

    if args.output_md is not None:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        lines = [
            '# Live ROI Depth Eval Summary',
            '',
            '| range_group | samples | roi_valid_ratio | center_valid_ratio | foreground_support_ratio | foreground_purity_ratio | abs_error_m | saturation_ratio | min_depth_m | p10_depth_m | p25_depth_m | p50_depth_m | p75_depth_m | median_depth_m |',
            '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |',
        ]
        for row in summaries:
            lines.append(
                '| {range_group} | {samples} | {roi_valid_ratio:.4f} | {center_valid_ratio:.4f} | {foreground_support_ratio:.4f} | {foreground_purity_ratio:.4f} | {abs_error_m:.4f} | {saturation_ratio:.4f} | {min_depth_m:.4f} | {p10_depth_m:.4f} | {p25_depth_m:.4f} | {p50_depth_m:.4f} | {p75_depth_m:.4f} | {median_depth_m:.4f} |'.format(
                    **row
                )
            )
        args.output_md.write_text('\n'.join(lines) + '\n', encoding='utf-8')

    print(f'Wrote {len(summaries)} summary rows to {args.output_csv}')


if __name__ == '__main__':
    main()
