# GT Eval Suite Summary

- rounds: 5
- duration_sec: 60
- gt_source: `agents`
- runs_collected: 5
- gt_objects_avg: 8.00
- skew_warn_avg: 13.40

## Metrics (mean ± std)

- `uv` recall: 0.185 ± 0.043, center_err(m): 1.129 ± 0.013
- `db` recall: 0.048 ± 0.015, center_err(m): 1.131 ± 0.092
- `lidar` recall: 0.097 ± 0.037, center_err(m): 1.295 ± 0.012
- `fused` recall: 0.083 ± 0.028, center_err(m): 1.290 ± 0.019
- `tracked` recall: 0.101 ± 0.028, center_err(m): 1.278 ± 0.033
- `yolo_2d` recall: 0.322 ± 0.100

## Artifacts

- summary.txt: `/home/skbt2/lvdot_ros2_migration_ws/reports/gt_eval_suite_20260505_233903/summary.txt`
- eval logs: `/home/skbt2/lvdot_ros2_migration_ws/reports/gt_eval_suite_20260505_233903/eval_run*.txt`
- launch logs: `/home/skbt2/lvdot_ros2_migration_ws/reports/gt_eval_suite_20260505_233903/launch_run*.log`
