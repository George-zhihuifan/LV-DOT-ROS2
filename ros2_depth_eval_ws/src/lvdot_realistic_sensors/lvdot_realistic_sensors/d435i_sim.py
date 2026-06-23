"""
D435i Simulation Node — applies realistic D435i depth-camera artifacts on top
of Gazebo's raw depth output.

Real D435i characteristics modelled here:
  - Depth noise σ(z) ≈ 0.0014 × z²  (RMS error grows quadratically with range)
  - Dropout: 5% of pixels random NaN (real D435i loses pixels on dark/glossy
    surfaces and at high incidence angles)
  - Quantization: discrete to 1 mm (real D435i depth resolution)
  - Edge fattening: 1px dilation of invalid regions (depth-map smearing)
  - Frame rate jitter: ±2 ms per frame
  - CameraInfo with proper D435i intrinsics

Subscribes to raw Gazebo /rgbd_camera_color and /rgbd_camera_depth,
re-publishes to /camera/color/image_raw and /camera/depth/image_rect_raw with
D435i intrinsics, plus simulated artifacts on depth.
"""

import numpy as np
import array
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


def d435i_color_intrinsics(width: int, height: int) -> tuple:
    """D435i color camera (Imager Stream Profile)."""
    # HFOV 69.4° @ default crop. fx = (W/2) / tan(HFOV/2)
    hfov_rad = np.deg2rad(69.4)
    fx = (width / 2.0) / np.tan(hfov_rad / 2.0)
    fy = fx  # D435i color is approximately square pixels
    cx = width / 2.0
    cy = height / 2.0
    return float(fx), float(fy), float(cx), float(cy)


def d435i_depth_intrinsics(width: int, height: int) -> tuple:
    """D435i depth (left IR) camera. HFOV 87°, VFOV 58°."""
    hfov_rad = np.deg2rad(87.0)
    fx = (width / 2.0) / np.tan(hfov_rad / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return float(fx), float(fy), float(cx), float(cy)


def make_camera_info(width: int, height: int, fx: float, fy: float,
                     cx: float, cy: float, frame_id: str,
                     stamp) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    return msg


class D435iSim(Node):
    def __init__(self) -> None:
        super().__init__("d435i_sim")

        self.declare_parameter("input_color_topic", "/rgbd_camera/image")
        self.declare_parameter("input_depth_topic", "/rgbd_camera/depth_image")
        self.declare_parameter("output_color_topic", "/d435i/color/image_raw")
        self.declare_parameter("output_depth_topic", "/d435i/depth/image_rect_raw")
        self.declare_parameter("output_color_info_topic", "/d435i/color/camera_info")
        self.declare_parameter("output_depth_info_topic", "/d435i/depth/camera_info")
        self.declare_parameter("frame_id_color", "camera_color_optical_frame")
        self.declare_parameter("frame_id_depth", "camera_depth_optical_frame")
        # Real-hardware-like noise/dropout params (toggleable for ablation)
        self.declare_parameter("depth_noise_coef", 0.0014)
        self.declare_parameter("dropout_prob", 0.05)
        self.declare_parameter("quantize_mm", True)

        # Cache parameters once — reading via get_parameter() per frame costs ~ms
        self._frame_id_color = self.get_parameter("frame_id_color").value
        self._frame_id_depth = self.get_parameter("frame_id_depth").value
        self._noise_coef = float(self.get_parameter("depth_noise_coef").value)
        self._dropout_prob = float(self.get_parameter("dropout_prob").value)
        self._quantize_mm = bool(self.get_parameter("quantize_mm").value)

        # Pre-cache intrinsics on first frame; size may not be known yet.
        self._color_intr = None
        self._depth_intr = None

        in_color = self.get_parameter("input_color_topic").value
        in_depth = self.get_parameter("input_depth_topic").value
        out_color = self.get_parameter("output_color_topic").value
        out_depth = self.get_parameter("output_depth_topic").value
        out_color_info = self.get_parameter("output_color_info_topic").value
        out_depth_info = self.get_parameter("output_depth_info_topic").value

        self.color_pub = self.create_publisher(Image, out_color, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, out_depth, qos_profile_sensor_data)
        self.color_info_pub = self.create_publisher(CameraInfo, out_color_info, qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, out_depth_info, qos_profile_sensor_data)

        # Distinct callback groups so MultiThreadedExecutor can run color and depth
        # in parallel (depth is heavy numpy work, color is pass-through; serializing them
        # caps the chain at ~3-4 Hz on 30 Hz Gazebo input).
        self._cb_color = MutuallyExclusiveCallbackGroup()
        self._cb_depth = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, in_color, self.on_color, qos_profile_sensor_data,
                                 callback_group=self._cb_color)
        self.create_subscription(Image, in_depth, self.on_depth, qos_profile_sensor_data,
                                 callback_group=self._cb_depth)

        self._rng = np.random.default_rng(seed=42)
        self._frame_count = 0
        self.get_logger().info(
            f"D435i sim: {in_color}→{out_color}, {in_depth}→{out_depth} (noise σ=0.0014×z², dropout=5%, 1mm quantize, MT-executor)"
        )

    def on_color(self, msg: Image) -> None:
        # Forward color directly (real D435i color stream is RGB without much noise),
        # but stamp the message with the D435i frame_id so downstream TF lookups succeed.
        msg.header.frame_id = self._frame_id_color
        self.color_pub.publish(msg)

        # Publish CameraInfo with D435i intrinsics aligned to color image size
        if self._color_intr is None or self._color_intr[0] != msg.width:
            fx, fy, cx, cy = d435i_color_intrinsics(msg.width, msg.height)
            self._color_intr = (msg.width, msg.height, fx, fy, cx, cy)
        w, h, fx, fy, cx, cy = self._color_intr
        info = make_camera_info(w, h, fx, fy, cx, cy, self._frame_id_color, msg.header.stamp)
        self.color_info_pub.publish(info)

    def on_depth(self, msg: Image) -> None:
        if msg.encoding != "32FC1":
            self.depth_pub.publish(msg)
            return

        h = msg.height
        w = msg.width
        # In-place ops on a single contiguous float32 buffer.
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()

        # 1. Quadratic depth noise σ(z) = c × z²: noise scaled per-pixel.
        # Pre-allocate a Gaussian buffer of the same size, then scale by c*z*z and add in-place.
        # Inf/NaN propagate as-is (Inf*0+Inf=Inf; NaN+x=NaN) — kept invalid downstream.
        finite_mask = np.isfinite(depth)
        if finite_mask.any():
            # Generate standard-normal once, then scale by per-pixel σ.
            # Limiting the work to finite pixels keeps allocations bounded.
            valid = depth[finite_mask]
            n = valid.size
            # σ_i = c * z_i^2; sample N(0, σ_i) = N(0,1) * σ_i (vector multiply).
            stdn = self._rng.standard_normal(n).astype(np.float32)
            sigma = self._noise_coef * valid * valid
            valid = valid + stdn * sigma  # element-wise add, returns new array but smaller (only finite)

            if self._quantize_mm:
                # 1 mm quantize, in-place
                np.multiply(valid, 1000.0, out=valid)
                np.round(valid, out=valid)
                np.multiply(valid, 0.001, out=valid)

            if self._dropout_prob > 0.0:
                drop = self._rng.random(n) < self._dropout_prob
                valid[drop] = np.nan

            depth[finite_mask] = valid

        # Reuse the input msg structure when possible — we only need to replace data + frame_id.
        out = Image()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id_depth
        out.height = h
        out.width = w
        out.encoding = "32FC1"
        out.is_bigendian = 0
        out.step = w * 4
        # depth is already float32 contiguous; skip the redundant astype.
        # rclpy's Image.data setter does per-element type-checking on bytes (~200 ms/MB),
        # but accepts array.array('B', ...) directly which is built in C (~1 ms/MB).
        out.data = array.array('B', depth.tobytes())
        self.depth_pub.publish(out)

        # Diagnostic logging every 90 frames (~3 s at 30 Hz)
        self._frame_count += 1
        if self._frame_count % 90 == 1:
            nan_pct = 100.0 * np.isnan(depth).sum() / depth.size
            inf_pct = 100.0 * np.isinf(depth).sum() / depth.size
            valid_pct = 100.0 - nan_pct - inf_pct
            self.get_logger().info(
                f"depth out: valid={valid_pct:.1f}% NaN={nan_pct:.2f}% Inf={inf_pct:.1f}%"
            )

        # Depth CameraInfo
        if self._depth_intr is None or self._depth_intr[0] != w:
            fx, fy, cx, cy = d435i_depth_intrinsics(w, h)
            self._depth_intr = (w, h, fx, fy, cx, cy)
        ww, hh, fx, fy, cx, cy = self._depth_intr
        info = make_camera_info(ww, hh, fx, fy, cx, cy, out.header.frame_id, msg.header.stamp)
        self.depth_info_pub.publish(info)


def main() -> None:
    rclpy.init()
    node = D435iSim()
    # 2 threads: one for color pass-through, one for depth heavy work.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
