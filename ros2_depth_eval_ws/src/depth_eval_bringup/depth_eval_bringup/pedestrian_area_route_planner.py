from dataclasses import dataclass

from depth_eval_bringup.pedestrian_planner_base import BaseAreaPlanner


@dataclass
class AreaRoutePlanner(BaseAreaPlanner):
    def advance(self, x: float, y: float, sim_time: float) -> None:
        self.arrival_time = None
        self.dwell_duration = 0.0
        next_area = self._select_next_area(x, y, exclude_current=True)
        self._set_new_area(next_area, x, y)
