from dataclasses import dataclass, field
import math
import random

from depth_eval_bringup.pedestrian_area_route_planner import AreaRoutePlanner
from depth_eval_bringup.pedestrian_attraction_planner import AttractionPlanner
from depth_eval_bringup.pedestrian_planner_profiles import profile_for_behavior
from depth_eval_bringup.pedestrian_scene import AgentConfig, Scene, Waypoint


@dataclass
class PedestrianAgentStateMachine:
    agent: AgentConfig
    scene: Scene
    rng: random.Random = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.route_planner = AreaRoutePlanner(agent=self.agent, scene=self.scene)
        self.attraction_planner = AttractionPlanner(agent=self.agent, scene=self.scene)
        self.profile = profile_for_behavior(self.agent.behavior)
        self.rng = random.Random(1000 + sum(ord(ch) for ch in self.agent.name))
        self.state = 'walking'
        self.current_mode = 'transit'
        self.current_area_index = self.route_planner.current_area_index
        self.shopping_started_at: float | None = None
        self.shopping_cooldown_until = 0.0
        self.last_attraction_waypoint_id: str | None = None

    @property
    def active_planner(self):
        if self.state == 'shopping':
            return self.attraction_planner
        return self.route_planner

    def _closest_attraction_area(self, x: float, y: float) -> tuple[str, float, float] | None:
        best: tuple[str, float, float] | None = None
        for waypoint_id, waypoint in self.scene.waypoints.items():
            if waypoint.kind != 'attraction':
                continue
            distance = math.hypot(waypoint.x - x, waypoint.y - y)
            if distance > self.profile.attraction_max_distance:
                continue
            if best is None or distance < best[1]:
                best = (waypoint_id, distance, waypoint.strength)
        return best

    def _activate_shopping(self, waypoint_id: str, x: float, y: float, sim_time: float) -> None:
        self.attraction_planner.set_attraction_waypoint(waypoint_id, x, y)
        self.state = 'shopping'
        self.current_area_index = self.attraction_planner.current_area_index
        self.current_mode = self.attraction_planner.current_mode
        self.shopping_started_at = sim_time
        self.last_attraction_waypoint_id = waypoint_id

    def _leave_shopping(self, x: float, y: float, sim_time: float) -> None:
        next_area = self.route_planner._select_next_area(x, y, exclude_current=True)
        self.route_planner._set_new_area(next_area, x, y)
        self.state = 'walking'
        self.current_area_index = self.route_planner.current_area_index
        self.current_mode = self.route_planner.current_mode
        self.shopping_started_at = None
        self.shopping_cooldown_until = sim_time + self.rng.uniform(
            self.profile.shopping_cooldown_min,
            self.profile.shopping_cooldown_max,
        )

    def update_state(self, x: float, y: float, sim_time: float, dt: float) -> None:
        if self.state == 'shopping':
            shopping_started = self.shopping_started_at if self.shopping_started_at is not None else sim_time
            enough_time = (sim_time - shopping_started) >= self.profile.shopping_min_duration
            release_probability = self.profile.shopping_release_probability * max(dt, 1e-3)
            if enough_time and self.rng.random() < release_probability:
                self._leave_shopping(x, y, sim_time)
            return

        if self.profile.attraction_probability <= 0.0:
            return
        if sim_time < self.shopping_cooldown_until:
            return
        attraction = self._closest_attraction_area(x, y)
        if attraction is None:
            return
        waypoint_id, distance, strength = attraction
        attraction_weight = max(0.0, 1.0 - distance / max(self.profile.attraction_max_distance, 1e-3))
        probability = self.profile.attraction_probability * strength * attraction_weight * max(dt, 1e-3)
        if self.last_attraction_waypoint_id == waypoint_id:
            probability *= 0.45
        if self.rng.random() < probability:
            self._activate_shopping(waypoint_id, x, y, sim_time)

    def start_waypoint(self) -> Waypoint:
        return self.route_planner.start_waypoint()

    def current_waypoint(self) -> Waypoint:
        planner = self.active_planner
        self.current_area_index = planner.current_area_index
        self.current_mode = planner.current_mode
        return planner.current_waypoint()

    def has_completed_destination(self, x: float, y: float, sim_time: float) -> bool:
        return self.active_planner.has_completed_destination(x, y, sim_time)

    def advance(self, x: float, y: float, sim_time: float) -> None:
        if self.state != 'shopping':
            self.route_planner.advance(x, y, sim_time)
            self.current_area_index = self.route_planner.current_area_index
            self.current_mode = self.route_planner.current_mode
            return

        self.attraction_planner.advance(x, y, sim_time)
        self.current_area_index = self.attraction_planner.current_area_index
        self.current_mode = self.attraction_planner.current_mode
