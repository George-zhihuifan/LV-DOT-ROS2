
  数据快速诊断

  核心好消息：A3 → A4 提升 48%（0.20 → 0.30），QC-GAF 价值清晰可见。

  4 个问题点：
  1. A2 (Visual-only) 全 0 — 不是性能差，是 fusion_mode 配置不对或没产出
  dynamic_bboxes
  2. A5 (0.227) < A4 (0.298) — GRU 只影响预测不影响检测，应当 ≈ A4，这是
  run-to-run variance
  3. A6 (0.113) < A3 (0.203) — 这是混淆：A6 用
  detector_param.yaml（你调过的），A1-A5 用
  detector_param_baseline.yaml，不只是 noise adapt 的差异
  4. sparse_wide 缺 4 组

  ---
  执行方案（按优先级排序）

  步骤 1：诊断 A2 失败（5 分钟）

  # 查看 A2 启动日志
  grep -E "fusion_mode|dynamic_bboxes|ERROR|WARN" \
    /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/logs/ablation_20260517_0920
  36/dense_open_A2_visual_only/launch.log | head -30

  # 查看 frames.csv 是否有任何 det
  head -5 /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/logs/ablation_2026051
  7_092036/dense_open_A2_visual_only/frames.csv
  tail -5 /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/logs/ablation_2026051
  7_092036/dense_open_A2_visual_only/frames.csv

  可能原因：fusion_mode:=depth_driven
  这个值在检测器里不存在/不支持。我需要看完日志才能确认修复方案。

  步骤 2：修复 A6 混淆（编辑 1 行）

  把 src/lvdot_bringup/scripts/run_ablation.sh 里 A6 的 detector_config 改成
  baseline：

  # 修改前（第 33 行附近）：
  "A6_noise_adapt:dual:true:true:true:${WS_ROOT}/install/lvdot_bringup/share/
  lvdot_bringup/config/detector_param.yaml:..."

  # 改成：
  "A6_noise_adapt:dual:true:true:true:${WS_ROOT}/install/lvdot_bringup/share/
  lvdot_bringup/config/detector_param_baseline.yaml:..."

  然后还要在 launch 命令里加 qcgaf_noise_adaptation_enabled:=true（要查
  launch 文件支不支持这个 override，目前 baseline yaml 里这个值是 false）。

  如果 launch 不支持单独覆盖这个参数，那 A6 就需要单独建一个
  detector_param_baseline_noise.yaml（在 baseline 基础上把
  qcgaf_noise_adaptation_enabled 设 true）。

  步骤 3：补跑缺失的 sparse_wide 4 组（~10 分钟）

  把 run_ablation.sh 临时改成只跑这 4 组：

  # 修改 SCENARIOS 数组，只留 sparse_wide
  SCENARIOS=(
    "sparse_wide:${DEPTH_WS}/install/depth_eval_bringup/share/depth_eval_brin
  gup/config/pedestrian_prototype.yaml"
  )
  # 修改 GROUPS 数组，只留 A3-A6
  GROUPS=(
    "A3_lvdot_baseline:..."
    "A4_qcgaf:..."
    "A5_qcgaf_gru:..."
    "A6_noise_adapt:..."
  )

  # 改输出目录避免覆盖
  OUT_ROOT="${WS_ROOT}/logs/ablation_sparsewide_$(date +%Y%m%d_%H%M%S)" bash
  src/lvdot_bringup/scripts/run_ablation.sh

  步骤 4：每组重复 3 次降噪（可选，~70 分钟）

  A4/A5 接近，需要确认是真实差异还是 variance。把 run_ablation.sh 外层套个
  for run in 1 2 3 循环，输出到 run_1/、run_2/、run_3/。

  ---
  我建议的最小可发表路径

  如果时间紧，只做步骤 1+2+3，跳过步骤 4，最终汇报：
  - 主表：dense_open 6 组 + sparse_wide 6 组的 F1@1m
  - 标注：A6 用 baseline+noise_adapt（修复后）
  - 结论：A3→A4 显著提升（QC-GAF 有效），A5 ≈ A4（QC-GAF 价值不变，GRU
  提供轨迹预测能力，看 ADE/FDE 表）

  你先执行步骤 1，把 A2 的日志贴出来，我看完再给步骤 2 的具体修法。

✻ Churned for 52s

❯ shijianbujin,woxianzaiyouchongfendeshijian,nigaowoyigewanzhengdefangan     

● 好，给你一个完整方案，分 5
  个阶段。每步都给具体命令和验收标准，按顺序执行即可。

  ---
  完整消融实验方案

  阶段 0：诊断现有结果（30 分钟）

  目标：搞清楚 A2 为什么 F1=0，A6 为什么倒退，确保后续实验配置正确。

  0.1 诊断 A2 (Visual-only) 失败

  # 1. 看启动日志，找 fusion_mode 是否被检测器识别
  grep -E "fusion_mode|Fusion mode|Unknown
  fusion|VisualOnly|visual_only|depth_driven|lidar_driven" \
    /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/logs/ablation_20260517_0920
  36/dense_open_A2_visual_only/launch.log

  # 2. 看检测器是否产出 dynamic_bboxes
  grep -E "Pipeline stats|Input health|dynamic_count|fused" \
    /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/logs/ablation_20260517_0920
  36/dense_open_A2_visual_only/launch.log | tail -30

  # 3. 看 frames.csv 总检测数
  awk -F, 'NR>1{sum+=$3; n++} END{print "frames=", n, "avg_det=", sum/n}' \
    /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/logs/ablation_20260517_0920
  36/dense_open_A2_visual_only/frames.csv

  把这三个命令的输出贴给我，我判断是：
  - (a) fusion_mode 参数值不被识别（需要改成 visual_only 或别的）
  - (b) 视觉-only 时没 LiDAR 兜底，YOLO+UV 检测确实出不了 dynamic
  - (c) 别的链路问题

  0.2 确认 fusion_mode 支持的值

  # 查源码里所有支持的 fusion_mode 字符串
  grep -rn "fusion_mode\|FusionMode\|fusionMode" \
    /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/src/lvdot_core/ \
    /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/src/lvdot_ros2/ | \
    grep -iE "lidar|visual|depth|dual|only" | head -20

  0.3 验证 A6 noise_adaptation 是否生效

  # 看 A6 启动日志里有没有打印 qcgaf_noise_adaptation_enabled=true
  grep -E "noise_adaptation|alpha_q|alpha_r|quality_vector" \
    /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws/logs/ablation_20260517_0920
  36/dense_open_A6_noise_adapt/launch.log | head -20

  ---
  阶段 1：修复配置（30-60 分钟）

  1.1 修复 A2 fusion_mode（待阶段 0.1 结果确定）

  可能的修复（三选一）：
  - 把 depth_driven 改成 visual_only
  - 把 depth_driven 改成 visual 或源码里实际接受的字符串
  - 如果纯视觉模式不存在，把 A2 重定义为「YOLO + depth UV，无
  lidar」并在配置里关 lidar 输入

  1.2 修复 A6 配置混淆

  创建 src/lvdot_bringup/config/detector_param_baseline_noise.yaml：
  - 复制 detector_param_baseline.yaml 全部内容
  - 只改一行：qcgaf_noise_adaptation_enabled: true

  修改 run_ablation.sh 第 33 行 A6 的 detector_config 指向新 yaml：
  "A6_noise_adapt:dual:true:true:true:${WS_ROOT}/install/lvdot_bringup/share/
  lvdot_bringup/config/detector_param_baseline_noise.yaml:/qcgaf/fused_bboxes
  :1:qcgaf_fused"

  这样 A6 和 A5 之间只差 noise_adaptation 这一个变量，对比才有意义。

  1.3 改进 run_ablation.sh 支持多 seed/多 run

  在脚本顶部加：
  NUM_RUNS="${NUM_RUNS:-3}"  # 每组重复次数

  把内层 for 循环改成：
  for run_idx in $(seq 1 "${NUM_RUNS}"); do
    for scenario in "${SCENARIOS[@]}"; do
      for group in "${GROUPS[@]}"; do
        run_dir="${OUT_ROOT}/run${run_idx}_${scenario_name}_${group_name}"
        # ... 原有逻辑
      done
    done
  done

  注意：场景默认 seed 是固定的（Gazebo 行人模型 social force
  内部有随机性，但行人初始位置由 YAML 决定）。所以「3
  次重复」其实是同样初始条件 + 不同行人 social force 演化 + 不同 YOLO
  推理时序。如果想要真随机种子，需要在 pedestrian_state_publisher
  启动参数里加 seed 支持（这是单独的改造，可选）。

  1.4 验收

  修复后跑一次单组测试：
  # 只跑 dense_open_A2 一组，verify visual_only 出 detections
  NUM_RUNS=1 OUT_ROOT=/tmp/test_A2 \
    SCENARIOS=("dense_open:...") \
    GROUPS=("A2_visual_only:...") \
    bash src/lvdot_bringup/scripts/run_ablation.sh
  # 看 summary.json 里 center_distance.1.0m.tp 是否 > 0

  ---
  阶段 2：跑完整消融（~70-90 分钟）

  2.1 配置

  - 2 场景：dense_open + sparse_wide
  - 6 实验组：A1, A2, A3, A4, A5, A6
  - 每组 3 次重复
  - 总计：2 × 6 × 3 = 36 次运行
  - 每次：15s warmup + 60s eval + ~30s 启动/清理 ≈ 105s
  - 总耗时：36 × 105s ≈ 63 分钟（不算切换开销，实测可能 75-90 分钟）

  2.2 执行

  cd /home/mcb/LV-DOT-ROS2/lvdot_ros2_migration_ws
  source /opt/ros/humble/setup.bash
  source /home/mcb/LV-DOT-ROS2/ros2_depth_eval_ws/install/setup.bash
  source install/setup.bash

  # 后台跑，输出到 log 文件方便检查进度
  NUM_RUNS=3 nohup bash src/lvdot_bringup/scripts/run_ablation.sh \
    > logs/ablation_full_$(date +%Y%m%d_%H%M%S).log 2>&1 &

  echo "PID=$!"

  2.3 进度监控

  每隔几分钟看：
  # 当前已完成几组
  ls -d logs/ablation_*/run*/  2>/dev/null | wc -l

  # 看最新一组的状态
  ls -t logs/ablation_*/ | head -3

  2.4 中断恢复

  如果中途断了，看哪些 summary.json 已经写出来了，把 run_ablation.sh
  改成跳过已完成的组：

  # 在 group 循环里加 skip 判断
  if [[ -f "${run_dir}/summary.json" ]]; then
    echo "skip ${run_dir} (already done)"
    continue
  fi

  ---
  阶段 3：聚合多 run 结果（30 分钟）

  3.1 改进 summarize_results.py 支持多 run 聚合

  新增逻辑：把 run{N}_{scenario}_{group} 按 {scenario}_{group} 分组，每组算
  mean ± std。

  输出三张表：
  1. 主表（mean）：列同现有 center-distance 表，单元格写 mean
  2. 方差表（std）：同结构，单元格写 std
  3. 合并表（mean±std）：单元格 0.298 ± 0.012 格式

  3.2 生成论文格式表

  新建 scripts/paper_tables.py：

  输出 4 张论文级表格（Markdown + LaTeX）：

  Table I — Detection Comparison（center-distance）

  ┌──────────────────┬─────────┬──────┬───────┬──────────┬──────┬───────┐
  │      Method      │ Dense   │      │       │ Sparse   │      │       │
  │                  │  Open   │      │       │   Wide   │      │       │
  ├──────────────────┼─────────┼──────┼───────┼──────────┼──────┼───────┤
  │                  │ P@1m    │ R@1m │ F1@1m │ P@1m     │ R@1m │ F1@1m │
  ├──────────────────┼─────────┼──────┼───────┼──────────┼──────┼───────┤
  │ LiDAR-only (A1)  │ ...     │ ...  │ ...   │ ...      │ ...  │ ...   │
  ├──────────────────┼─────────┼──────┼───────┼──────────┼──────┼───────┤
  │ Visual-only (A2) │ ...     │ ...  │ ...   │ ...      │ ...  │ ...   │
  ├──────────────────┼─────────┼──────┼───────┼──────────┼──────┼───────┤
  │ LV-DOT baseline  │ ...     │ ...  │ ...   │ ...      │ ...  │ ...   │
  │ (A3)             │         │      │       │          │      │       │
  ├──────────────────┼─────────┼──────┼───────┼──────────┼──────┼───────┤
  │ + QC-GAF (A4)    │ best    │ ...  │ ...   │ ...      │ ...  │ ...   │
  ├──────────────────┼─────────┼──────┼───────┼──────────┼──────┼───────┤
  │ + GRU (A5)       │ ...     │ ...  │ ...   │ ...      │ ...  │ ...   │
  ├──────────────────┼─────────┼──────┼───────┼──────────┼──────┼───────┤