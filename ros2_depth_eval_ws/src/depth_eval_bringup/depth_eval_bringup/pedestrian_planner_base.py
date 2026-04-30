from dataclasses import dataclass, field
import math
import random

from depth_eval_bringup.pedestrian_planner_profiles import BehaviorProfile, profile_for_behavior
from depth_eval_bringup.pedestrian_scene import AgentConfig, Scene, Waypoint


@dataclass
class BaseAreaPlanner:
    agent: AgentConfig
    scene: Scene
    rng: random.Random = field(init=False, repr=False)
    profile: BehaviorProfile = field(init=False)
    current_area_index: int = 0
    current_target: tuple[float, float] | None = None
    current_mode: str = 'transit'
    arrival_time: float | None = None
    dwell_duration: float = 0.0
    recent_area_indices: list[int] = field(default_factory=list)
    area_visit_counts: dict[int, int] = field(default_factory=dict)
    last_heading: float | None = None

    def __post_init__(self) -> None:
        if not self.agent.waypoint_ids:
            raise ValueError(f'Agent {self.agent.name} has no waypoint ids.')
        self.rng = random.Random(sum(ord(ch) for ch in self.agent.name))
        self.profile = profile_for_behavior(self.agent.behavior)
        self.recent_area_indices = [0]
        start = self.start_waypoint()
        initial_area = self._select_next_area(start.x, start.y, exclude_current=False)
        self._set_new_area(initial_area, start.x, start.y)

    def start_waypoint(self) -> Waypoint:
        return self.scene.waypoints[self.agent.waypoint_ids[0]]

    def current_waypoint(self) -> Waypoint:
        base = self._base_waypoint(self.current_area_index)
        if self.current_target is None:
            self.current_target = self._transit_point(self.current_area_index)
        return Waypoint(
            waypoint_id=base.waypoint_id,
            x=self.current_target[0],
            y=self.current_target[1],
            radius=max(base.radius, self.profile.area_radius),
            kind=base.kind,
            strength=base.strength,
        )

    def has_completed_destination(self, x: float, y: float, sim_time: float) -> bool:
        waypoint = self.current_waypoint()
        distance = math.hypot(waypoint.x - x, waypoint.y - y)
        if distance > waypoint.radius:
            self.arrival_time = None
            self.dwell_duration = 0.0
            return False
        if self.arrival_time is None:
            self.arrival_time = sim_time
            self.dwell_duration = self._sample_dwell_duration()
            return self.dwell_duration <= 1e-6
        return sim_time >= self.arrival_time + self.dwell_duration

    def advance(self, x: float, y: float, sim_time: float) -> None:
        raise NotImplementedError

    def _base_waypoint(self, area_index: int) -> Waypoint:
        return self.scene.waypoints[self.agent.waypoint_ids[area_index]]

    def _sample_dwell_duration(self) -> float:
        if self.current_mode == 'local':
            return self.rng.uniform(self.profile.local_dwell_min, self.profile.local_dwell_max)
        return self.rng.uniform(self.profile.transit_dwell_min, self.profile.transit_dwell_max)

    def _random_point_in_area(self, area_index: int, scale: float) -> tuple[float, float]:
        base = self._base_waypoint(area_index)
        effective_radius = max(base.radius, self.profile.area_radius)
        jitter_radius = max(0.25, min(effective_radius, effective_radius * scale))
        angle = self.rng.uniform(-math.pi, math.pi)
        radius = jitter_radius * math.sqrt(self.rng.random())
        return (
            base.x + math.cos(angle) * radius,
            base.y + math.sin(angle) * radius,
        )

    def _clamp_point_to_area(self, area_index: int, point: tuple[float, float]) -> tuple[float, float]:
        base = self._base_waypoint(area_index)
        effective_radius = max(base.radius, self.profile.area_radius)
        dx = point[0] - base.x
        dy = point[1] - base.y
        distance = math.hypot(dx, dy)
        if distance <= effective_radius or distance <= 1e-6:
            return point
        scale = effective_radius / distance
        return (base.x + dx * scale, base.y + dy * scale)

    def _helper_offset(self) -> tuple[float, float]:
        sigma = max(0.45, self.profile.helper_offset_sigma)
        radius = abs(self.rng.gauss(0.0, sigma))
        angles = (
            0.0,
            math.pi / 4.0,
            math.pi / 2.0,
            3.0 * math.pi / 4.0,
            math.pi,
            -3.0 * math.pi / 4.0,
            -math.pi / 2.0,
            -math.pi / 4.0,
        )
        angle = self.rng.choice(angles) + self.rng.uniform(-0.22, 0.22)
        if radius < 0.35:
            radius = 0.35 + self.rng.random() * 0.35
        return (math.cos(angle) * radius, math.sin(angle) * radius)

    def _next_local_target(self, area_index: int, x: float, y: float) -> tuple[float, float]:
        if self.current_mode == 'local' and self.current_target is not None:
            anchor = self.current_target
        else:
            anchor = self._random_point_in_area(area_index, self.profile.transit_jitter_scale)
        offset_x, offset_y = self._helper_offset()
        candidate = (anchor[0] + offset_x, anchor[1] + offset_y)
        candidate = self._clamp_point_to_area(area_index, candidate)
        if math.hypot(candidate[0] - x, candidate[1] - y) < 0.6:
            candidate = self._random_point_in_area(area_index, self.profile.local_jitter_scale)
        return candidate

    def _transit_point(self, area_index: int) -> tuple[float, float]:
        return self._random_point_in_area(area_index, self.profile.transit_jitter_scale)

    def _heading_weight(self, x: float, y: float, area_index: int) -> float:
        if self.last_heading is None:
            return 1.0
        base = self._base_waypoint(area_index)
        heading = math.atan2(base.y - y, base.x - x)
        delta = (heading - self.last_heading + math.pi) % (2.0 * math.pi) - math.pi
        straightness = max(0.0, math.cos(delta))
        bias = self.profile.heading_bias
        return (1.0 - bias) + bias * straightness

    def _select_next_area(self, x: float, y: float, exclude_current: bool) -> int:
        count = len(self.agent.waypoint_ids)
        candidates: list[int] = []
        weights: list[float] = []
        for index, waypoint_id in enumerate(self.agent.waypoint_ids):
            if exclude_current and index == self.current_area_index:
                continue
            base = self.scene.waypoints[waypoint_id]
            dist = math.hypot(base.x - x, base.y - y)
            if exclude_current and dist < 3.0:
                continue
            sigma = max(1.0, self.profile.hop_sigma)
            weight = math.exp(-((dist - self.profile.hop_distance) ** 2) / (2.0 * sigma * sigma)) + 0.06
            if index in self.recent_area_indices[-self.profile.recency_window:]:
                weight *= 0.01
            visit_count = self.area_visit_counts.get(index, 0)
            weight /= (1.0 + 0.45 * visit_count)
            weight *= self._heading_weight(x, y, index)
            if weight <= 1e-6:
                continue
            candidates.append(index)
            weights.append(weight)
        if not candidates:
            if exclude_current:
                return (self.current_area_index + 1) % count
            return self.current_area_index
        return self.rng.choices(candidates, weights=weights, k=1)[0]

    def _set_new_area(self, area_index: int, x: float, y: float) -> None:
        self.current_area_index = area_index
        self.current_mode = 'transit'
        self.current_target = self._transit_point(area_index)
        self.recent_area_indices.append(area_index)
        self.recent_area_indices = self.recent_area_indices[-10:]
        self.area_visit_counts[area_index] = self.area_visit_counts.get(area_index, 0) + 1
        self.last_heading = math.atan2(self.current_target[1] - y, self.current_target[0] - x)
