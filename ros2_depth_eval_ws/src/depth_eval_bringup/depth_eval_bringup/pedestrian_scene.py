from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Obstacle:
    name: str
    shape: str
    x: float
    y: float
    x1: float
    y1: float
    x2: float
    y2: float
    radius: float
    width: float
    depth: float
    length: float
    color: tuple[float, float, float]

    @property
    def effective_radius(self) -> float:
        if self.shape == 'segment':
            return math.hypot(self.x2 - self.x1, self.y2 - self.y1) * 0.5
        if self.shape == 'box':
            return ((self.width * 0.5) ** 2 + (self.depth * 0.5) ** 2) ** 0.5
        return self.radius

    def boundary_segments(self) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
        if self.shape == 'segment':
            return (((self.x1, self.y1), (self.x2, self.y2)),)
        if self.shape == 'box':
            hx = self.width * 0.5
            hy = self.depth * 0.5
            corners = (
                (self.x - hx, self.y - hy),
                (self.x + hx, self.y - hy),
                (self.x + hx, self.y + hy),
                (self.x - hx, self.y + hy),
            )
            return (
                (corners[0], corners[1]),
                (corners[1], corners[2]),
                (corners[2], corners[3]),
                (corners[3], corners[0]),
            )
        segment_count = 12
        points = []
        for index in range(segment_count):
            theta = 2.0 * math.pi * index / segment_count
            points.append((self.x + self.radius * math.cos(theta), self.y + self.radius * math.sin(theta)))
        segments = []
        for index in range(segment_count):
            segments.append((points[index], points[(index + 1) % segment_count]))
        return tuple(segments)


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    x: float
    y: float
    radius: float
    kind: str = 'route'
    strength: float = 1.0


@dataclass(frozen=True)
class AgentConfig:
    name: str
    agent_type: int
    group_id: int
    profile: str
    delay_start: float
    speed_mps: float
    loop: bool
    waypoint_ids: tuple[str, ...]
    desired_force_scale: float
    obstacle_force_scale: float
    social_force_scale: float
    preferred_distance: float
    behavior: str


@dataclass(frozen=True)
class WorldConfig:
    name: str
    ground_size: tuple[float, float]
    actor_model_uri: str
    actor_z: float
    actor_scale: float
    actor_speed_mps: float
    actor_radius: float
    trajectory_duration: float
    sim_dt: float
    waypoint_tolerance: float
    desired_force_gain: float
    obstacle_force_gain: float
    obstacle_influence_radius: float
    social_force_gain: float
    social_force_radius: float
    along_wall_force_gain: float
    along_wall_distance_threshold: float
    along_wall_speed_threshold: float
    random_force_gain: float
    random_force_time: float
    velocity_damping: float
    max_turn_rate_rad_s: float


@dataclass(frozen=True)
class Scene:
    world: WorldConfig
    obstacles: tuple[Obstacle, ...]
    waypoints: dict[str, Waypoint]
    agents: tuple[AgentConfig, ...]


def make_agent_config(item: dict, world: WorldConfig) -> AgentConfig:
    return AgentConfig(
        name=item['name'],
        agent_type=item.get('type', 1),
        group_id=item.get('group_id', -1),
        profile=item.get('profile', 'default'),
        delay_start=item.get('delay_start', 0.0),
        speed_mps=item.get('speed_mps', world.actor_speed_mps),
        loop=item.get('loop', True),
        waypoint_ids=tuple(item.get('waypoint_ids', ())),
        desired_force_scale=item.get('desired_force_scale', 1.0),
        obstacle_force_scale=item.get('obstacle_force_scale', 1.0),
        social_force_scale=item.get('social_force_scale', 1.0),
        preferred_distance=item.get('preferred_distance', world.social_force_radius),
        behavior=item.get('behavior', 'walk'),
    )


def expand_agent_groups(config: dict, world: WorldConfig) -> tuple[AgentConfig, ...]:
    expanded: list[AgentConfig] = []
    for item in config.get('agents', []):
        expanded.append(make_agent_config(item, world))

    for group in config.get('agent_groups', []):
        name_prefix = group['name_prefix']
        count = int(group['count'])
        start_index = int(group.get('start_index', 1))
        delay_start = float(group.get('delay_start', 0.0))
        delay_step = float(group.get('delay_step', 0.3))
        speed_base = float(group.get('speed_mps', world.actor_speed_mps))
        speed_step = float(group.get('speed_step', 0.0))
        waypoint_sets = group.get('waypoint_sets', [])
        default_waypoints = tuple(group.get('waypoint_ids', ()))
        for offset in range(count):
            if waypoint_sets:
                waypoint_ids = tuple(waypoint_sets[offset % len(waypoint_sets)])
            else:
                waypoint_ids = default_waypoints
            expanded.append(make_agent_config({
                'name': f'{name_prefix}_{start_index + offset}',
                'type': group.get('type', 1),
                'group_id': group.get('group_id', -1),
                'profile': group.get('profile', 'default'),
                'behavior': group.get('behavior', 'walk'),
                'delay_start': delay_start + delay_step * offset,
                'speed_mps': speed_base + speed_step * offset,
                'loop': group.get('loop', True),
                'waypoint_ids': waypoint_ids,
                'desired_force_scale': group.get('desired_force_scale', 1.0),
                'obstacle_force_scale': group.get('obstacle_force_scale', 1.0),
                'social_force_scale': group.get('social_force_scale', 1.0),
                'preferred_distance': group.get('preferred_distance', world.social_force_radius),
            }, world))
    return tuple(expanded)


def load_scene(config: dict) -> Scene:
    world_raw = config['world']
    world = WorldConfig(
        name=world_raw['name'],
        ground_size=tuple(world_raw['ground_size']),
        actor_model_uri=world_raw['actor_model_uri'],
        actor_z=world_raw['actor_z'],
        actor_scale=world_raw.get('actor_scale', 1.0),
        actor_speed_mps=world_raw.get('actor_speed_mps', 0.9),
        actor_radius=world_raw.get('actor_radius', 0.34),
        trajectory_duration=world_raw.get('trajectory_duration', 28.0),
        sim_dt=world_raw.get('sim_dt', 0.2),
        waypoint_tolerance=world_raw.get('waypoint_tolerance', 0.45),
        desired_force_gain=world_raw.get('desired_force_gain', 1.6),
        obstacle_force_gain=world_raw.get('obstacle_force_gain', 1.8),
        obstacle_influence_radius=world_raw.get('obstacle_influence_radius', 0.95),
        social_force_gain=world_raw.get('social_force_gain', 0.55),
        social_force_radius=world_raw.get('social_force_radius', 0.9),
        along_wall_force_gain=world_raw.get('along_wall_force_gain', 0.85),
        along_wall_distance_threshold=world_raw.get('along_wall_distance_threshold', 0.7),
        along_wall_speed_threshold=world_raw.get('along_wall_speed_threshold', 0.18),
        random_force_gain=world_raw.get('random_force_gain', 0.06),
        random_force_time=world_raw.get('random_force_time', 1.2),
        velocity_damping=world_raw.get('velocity_damping', 0.72),
        max_turn_rate_rad_s=world_raw.get('max_turn_rate_rad_s', 2.4),
    )
    obstacles = tuple(
        Obstacle(
            name=item['name'],
            shape=item.get('shape', 'cylinder'),
            x=item.get('x', 0.0),
            y=item.get('y', 0.0),
            x1=item.get('x1', 0.0),
            y1=item.get('y1', 0.0),
            x2=item.get('x2', 0.0),
            y2=item.get('y2', 0.0),
            radius=item.get('radius', 0.0),
            width=item.get('width', 0.0),
            depth=item.get('depth', 0.0),
            length=item['length'],
            color=tuple(item['color']),
        )
        for item in config.get('obstacles', [])
    )
    waypoints = {
        item['id']: Waypoint(
            waypoint_id=item['id'],
            x=item['x'],
            y=item['y'],
            radius=item.get('r', world.waypoint_tolerance),
            kind=item.get('kind', 'route'),
            strength=item.get('strength', 1.0),
        )
        for item in config.get('waypoints', [])
    }
    agents = expand_agent_groups(config, world)
    return Scene(world=world, obstacles=obstacles, waypoints=waypoints, agents=agents)
