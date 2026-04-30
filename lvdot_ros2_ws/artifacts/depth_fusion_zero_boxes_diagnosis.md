# 深度融合零输出诊断 — 2026-04-28

## 问题描述
`pipeline_stats` 显示 `depth_samples=2000+, u_map=3-4, depth_boxes=0`。
U-map 检测到线段组，但最终没有生成 3D 框。

## 根本原因分析

### 1. Launch 参数覆盖问题（已修复）
**症状**：检测器订阅 `/camera/depth/image_rect_raw` 而不是 `/rgbd_camera/depth_image`

**原因**：`run_detector_with_adapter.launch.py` 的 `DeclareLaunchArgument` 默认值还是旧的 relay 话题：
```python
DeclareLaunchArgument("depth_image_topic", default_value="/camera/depth/image_rect_raw"),
DeclareLaunchArgument("color_image_topic", default_value="/camera/color/image_raw"),
DeclareLaunchArgument("lidar_pointcloud_topic", default_value="/pointcloud"),
```

**修复**：改为直接订阅 gz 话题（38-40行）：
```python
DeclareLaunchArgument("depth_image_topic", default_value="/rgbd_camera/depth_image"),
DeclareLaunchArgument("color_image_topic", default_value="/rgbd_camera/image"),
DeclareLaunchArgument("lidar_pointcloud_topic", default_value="/rgbd_camera/points"),
```

修复后 `depth_samples` 从 0 变成 2000+。

### 2. U-map 面积阈值过严（已修复）
**症状**：`u_map=4` 但 `depth_boxes=0`

**原因**：`uv_detector.cpp:327` 的面积阈值 `>= 25` 对于降采样后的 U-map 太大。
- U-map 分辨率：`(480/4) × (640*0.5) = 120 × 320`
- 25 像素 ≈ 5×5，对应原图 ~10×20 像素
- 仿真中远处行人的 U-map 投影可能只有 3×4 = 12 像素

**修复**：降低阈值到 9（`uv_detector.cpp:327`）：
```cpp
if(UVboxes[b].bb.area() >= 9)  // was 25
```

添加调试输出（310-337行）：
```cpp
int total_uvboxes = static_cast<int>(UVboxes.size());
int passed_area_check = 0;
// ... 循环 ...
if (total_uvboxes > 0) {
    printf("[UV] %d line groups → %d merged boxes (area≥9)\n", total_uvboxes, passed_area_check);
}
```

### 3. 深度图类型兼容性（用户建议，已实现）
**问题**：原始 ROS1 代码假设深度图是 RealSense 的 `16UC1`（毫米），但 Gazebo 输出 `32FC1`（米）。

**现有方案**：`lvdot_runtime_bridge.cpp:depth_image_to_uint16_mm` 已经做了统一转换：
- `32FC1` (float 米) → `16UC1` (uint16 毫米)
- `16UC1` (传感器原始单位) → `16UC1` (毫米，通过 `depth_scale_factor` 转换)

**语义混乱**：
- `depthImageMm` 已经是毫米
- 但 `uv_detector.cpp:216` 还保留了 `depthScale_` 的除法/乘法：
  ```cpp
  depth_rescale_val = int((float(depth_rescale.at<unsigned short>(row, col))/this->depthScale_)*1000.0);
  ```
- 当 `depthScale_=1000` 时，这个转换变成恒等（`(mm/1000)*1000 = mm`），所以**目前能工作**

**潜在风险**：
- 如果有人改 `depthScale_` 参数，会破坏深度值
- 代码语义不清晰（`depthScale_` 在移植版中已经不是"传感器单位→米"的比例）

**建议改进**（未实施）：
1. 在 `detection_filter.cpp:318` 强制设置 `uv.depthScale_ = 1.0f`
2. 修改 `uv_detector.cpp:216` 为 `depth_rescale_val = int(depth_rescale.at<unsigned short>(row, col));`
3. 或者，在 `depth_image_to_uint16_mm` 后面添加注释，说明输出已经是毫米，`depthScale_` 应保持 1000.0

## 当前状态
- ✅ Launch 参数修复：检测器正确订阅 `/rgbd_camera/*`
- ✅ 面积阈值降低：25 → 9
- ⚠️ `depthScale_` 语义混乱但能工作（`depth_scale_factor: 1000.0` 时）

## 下一步
1. **验证修复**：重启全栈，检查 `depth_boxes` 是否 > 0
2. **调试输出**：查看 `[UV]` 日志，确认有多少线段组通过了面积检查
3. **如果仍然为 0**：
   - 检查 `extract_3Dbox()` 是否生成了有效的 3D 框
   - 检查 `transformUVBBoxes()` 坐标转换是否正确
   - 检查 DBSCAN 聚类参数（`dbscan_min_points_cluster: 5`）

## 文件修改记录
- `lvdot_ros2_ws/src/lvdot_bringup/launch/run_detector_with_adapter.launch.py`
  - 38-40行：话题默认值改为 `/rgbd_camera/*`
- `lvdot_ros2_ws/src/lvdot_core/src/uv_detector.cpp`
  - 327行：面积阈值 25 → 9
  - 310-337行：添加调试输出 `[UV] X line groups → Y merged boxes`

## 用户反馈
用户建议在接收深度图时做类型判断和统一转换，避免算法和传感器类型不匹配。
这个建议是对的，`depth_image_to_uint16_mm` 已经实现了这个功能，但后续代码的 `depthScale_` 语义需要澄清。
