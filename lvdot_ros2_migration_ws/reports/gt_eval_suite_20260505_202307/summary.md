# GT Eval Suite Summary

- rounds: 5
- duration_sec: 60
- gt_source: `agents`
- runs_collected: 5
- gt_objects_avg: 8.00
- skew_warn_avg: 14.20

## Metrics (mean ± std)

- `uv` recall: 0.167 ± 0.025, center_err(m): 1.131 ± 0.009
- `db` recall: 0.044 ± 0.014, center_err(m): 1.122 ± 0.093
- `lidar` recall: 0.090 ± 0.030, center_err(m): 1.298 ± 0.013
- `fused` recall: 0.083 ± 0.029, center_err(m): 1.294 ± 0.022
- `tracked` recall: 0.100 ± 0.032, center_err(m): 1.276 ± 0.025
- `yolo_2d` recall: 0.269 ± 0.114

## Artifacts

- summary.txt: `/home/skbt2/lvdot_ros2_migration_ws/reports/gt_eval_suite_20260505_202307/summary.txt`
- eval logs: `/home/skbt2/lvdot_ros2_migration_ws/reports/gt_eval_suite_20260505_202307/eval_run*.txt`
- launch logs: `/home/skbt2/lvdot_ros2_migration_ws/reports/gt_eval_suite_20260505_202307/launch_run*.log`
