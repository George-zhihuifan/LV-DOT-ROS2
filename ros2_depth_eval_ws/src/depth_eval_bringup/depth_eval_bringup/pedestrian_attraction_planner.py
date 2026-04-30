from dataclasses import dataclass

from depth_eval_bringup.pedestrian_planner_base import BaseAreaPlanner


@dataclass
class AttractionPlanner(BaseAreaPlanner):
    helper_targets_remaining: int = 0
    attraction_waypoint_id: str | None = None

    def _base_waypoint(self, area_index: int):
        if self.attraction_waypoint_id is not None:
            return self.scene.waypoints[self.attraction_waypoint_id]
        return super()._base_waypoint(area_index)

    def _sample_helper_budget(self) -> int:
        return self.rng.randint(self.profile.local_stop_min, self.profile.local_stop_max)

    def set_attraction_waypoint(self, waypoint_id: str, x: float, y: float) -> None:
        self.attraction_waypoint_id = waypoint_id
        self.current_area_index = -1
        self.current_mode = 'transit'
        self.current_target = self._transit_point(self.current_area_index)
        self.arrival_time = None
        self.dwell_duration = 0.0
        self.helper_targets_remaining = self._sample_helper_budget()
        self.last_heading = None

    def _set_new_area(self, area_index: int, x: float, y: float) -> None:
        self.attraction_waypoint_id = None
        super()._set_new_area(area_index, x, y)
        self.helper_targets_remaining = self._sample_helper_budget()

    def _transit_point(self, area_index: int) -> tuple[float, float]:
        return self._random_point_in_area(area_index, self.profile.transit_jitter_scale)

    def _next_helper_target(self, area_index: int, x: float, y: float) -> tuple[float, float]:
        anchor = self.current_target
        if anchor is None or self.current_mode != 'local':
            anchor = self._random_point_in_area(area_index, self.profile.transit_jitter_scale)
        offset_x, offset_y = self._helper_offset()
        candidate = self._clamp_point_to_area(area_index, (anchor[0] + offset_x, anchor[1] + offset_y))
        if abs(candidate[0] - x) + abs(candidate[1] - y) < 0.8:
            candidate = self._random_point_in_area(area_index, self.profile.local_jitter_scale)
        return candidate

    def advance(self, x: float, y: float, sim_time: float) -> None:
        self.arrival_time = None
        self.dwell_duration = 0.0
        if self.helper_targets_remaining <= 0:
            self.helper_targets_remaining = self._sample_helper_budget()
        self.helper_targets_remaining -= 1
        self.current_mode = 'local'
        self.current_target = self._next_helper_target(self.current_area_index, x, y)
        self.last_heading = None
