import math
import random
from dataclasses import dataclass

from depth_eval_bringup.pedestrian_scene import Scene, load_scene
from depth_eval_bringup.pedestrian_waypoint_planner import create_planner


def normalize_angle(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def unwrap_angle(target: float, reference: float) -> float:
    target = normalize_angle(target)
    while target - reference > math.pi:
        target -= 2.0 * math.pi
    while target - reference < -math.pi:
        target += 2.0 * math.pi
    return target


def length(x: float, y: float) -> float:
    return math.hypot(x, y)


def clamp_norm(x: float, y: float, max_norm: float) -> tuple[float, float]:
    norm = length(x, y)
    if norm <= max_norm or norm < 1e-9:
        return x, y
    scale = max_norm / norm
    return x * scale, y * scale


def distance_point_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    vv = vx * vx + vy * vy
    if vv <= 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def closest_point_on_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> tuple[float, float, float]:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    vv = vx * vx + vy * vy
    if vv <= 1e-9:
        return ax, ay, 0.0
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
    return ax + t * vx, ay + t * vy, t


def ccw(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> float:
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def segments_intersect(
    ax: float,
    ay: float,
    bx: float,
    by: float,
    cx: float,
    cy: float,
    dx: float,
    dy: float,
) -> bool:
    ab_c = ccw(ax, ay, bx, by, cx, cy)
    ab_d = ccw(ax, ay, bx, by, dx, dy)
    cd_a = ccw(cx, cy, dx, dy, ax, ay)
    cd_b = ccw(cx, cy, dx, dy, bx, by)
    if abs(ab_c) < 1e-9 and abs(ab_d) < 1e-9 and abs(cd_a) < 1e-9 and abs(cd_b) < 1e-9:
        return (
            max(min(ax, bx), min(cx, dx)) <= min(max(ax, bx), max(cx, dx)) + 1e-9
            and max(min(ay, by), min(cy, dy)) <= min(max(ay, by), max(cy, dy)) + 1e-9
        )
    return ab_c * ab_d <= 0.0 and cd_a * cd_b <= 0.0


def segment_hits_buffered_obstacle(
    start: tuple[float, float],
    goal: tuple[float, float],
    obstacle,
    actor_radius: float,
) -> bool:
    sx, sy = start
    gx, gy = goal
    margin = actor_radius + 0.08
    if obstacle.shape == 'box':
        hx = obstacle.width * 0.5 + margin
        hy = obstacle.depth * 0.5 + margin
        min_x = obstacle.x - hx
        max_x = obstacle.x + hx
        min_y = obstacle.y - hy
        max_y = obstacle.y + hy
        if min_x <= sx <= max_x and min_y <= sy <= max_y:
            return True
        if min_x <= gx <= max_x and min_y <= gy <= max_y:
            return True
        corners = (
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
        )
        edges = (
            (corners[0], corners[1]),
            (corners[1], corners[2]),
            (corners[2], corners[3]),
            (corners[3], corners[0]),
        )
        return any(
            segments_intersect(sx, sy, gx, gy, ax, ay, bx, by)
            for (ax, ay), (bx, by) in edges
        )
    if obstacle.shape == 'cylinder':
        return distance_point_to_segment(sx, sy, gx, gy, obstacle.x, obstacle.y) <= obstacle.radius + margin
    for (ax, ay), (bx, by) in obstacle.boundary_segments():
        if distance_point_to_segment(sx, sy, ax, ay, bx, by) <= margin:
            return True
        if segments_intersect(sx, sy, gx, gy, ax, ay, bx, by):
            return True
    return False


def bypass_candidates(obstacle, actor_radius: float) -> list[tuple[float, float]]:
    margin = actor_radius + 0.45
    if obstacle.shape == 'box':
        hx = obstacle.width * 0.5 + margin
        hy = obstacle.depth * 0.5 + margin
        return [
            (obstacle.x - hx, obstacle.y - hy),
            (obstacle.x + hx, obstacle.y - hy),
            (obstacle.x + hx, obstacle.y + hy),
            (obstacle.x - hx, obstacle.y + hy),
        ]
    if obstacle.shape == 'cylinder':
        radius = obstacle.radius + margin
        return [
            (obstacle.x + radius, obstacle.y),
            (obstacle.x - radius, obstacle.y),
            (obstacle.x, obstacle.y + radius),
            (obstacle.x, obstacle.y - radius),
        ]
    seg_dx = obstacle.x2 - obstacle.x1
    seg_dy = obstacle.y2 - obstacle.y1
    seg_norm = length(seg_dx, seg_dy)
    if seg_norm < 1e-6:
        return [(obstacle.x1, obstacle.y1)]
    nx = -seg_dy / seg_norm
    ny = seg_dx / seg_norm
    pad = actor_radius + 0.35
    return [
        (obstacle.x1 + nx * pad, obstacle.y1 + ny * pad),
        (obstacle.x2 + nx * pad, obstacle.y2 + ny * pad),
        (obstacle.x1 - nx * pad, obstacle.y1 - ny * pad),
        (obstacle.x2 - nx * pad, obstacle.y2 - ny * pad),
    ]


def choose_navigation_target(
    position: tuple[float, float],
    goal: tuple[float, float],
    obstacles,
    actor_radius: float,
    rejected_targets: tuple[tuple[float, float], ...] = (),
) -> tuple[float, float]:
    blocking = None
    best_dist = None
    for obstacle in obstacles:
        if not segment_hits_buffered_obstacle(position, goal, obstacle, actor_radius):
            continue
        px, py = position
        gx, gy = goal
        if obstacle.shape == 'box':
            cx = min(max(obstacle.x, min(px, gx)), max(px, gx))
            cy = min(max(obstacle.y, min(py, gy)), max(py, gy))
            dist = length(px - cx, py - cy)
        elif obstacle.shape == 'cylinder':
            dist = length(px - obstacle.x, py - obstacle.y)
        else:
            dist = distance_point_to_segment(px, py, obstacle.x1, obstacle.y1, obstacle.x2, obstacle.y2)
        if best_dist is None or dist < best_dist:
            best_dist = dist
            blocking = obstacle
    if blocking is None:
        return goal
    candidates = bypass_candidates(blocking, actor_radius)
    scored_candidates = []
    for candidate in candidates:
        if any(length(candidate[0] - rejected[0], candidate[1] - rejected[1]) <= 0.35 for rejected in rejected_targets):
            continue
        if any(
            segment_hits_buffered_obstacle(position, candidate, obstacle, actor_radius)
            or segment_hits_buffered_obstacle(candidate, goal, obstacle, actor_radius)
            for obstacle in obstacles
            if obstacle is not blocking
        ):
            continue
        score = length(candidate[0] - position[0], candidate[1] - position[1]) + length(goal[0] - candidate[0], goal[1] - candidate[1])
        scored_candidates.append((score, candidate))
    if scored_candidates:
        scored_candidates.sort(key=lambda item: item[0])
        return scored_candidates[0][1]
    return goal


def obstacle_force(position: tuple[float, float], velocity: tuple[float, float], obstacles,
                   influence_radius: float, gain: float) -> tuple[float, float]:
    px, py = position
    vx, vy = velocity
    force_x = 0.0
    force_y = 0.0
    for obstacle in obstacles:
        best = None
        for (ax, ay), (bx, by) in obstacle.boundary_segments():
            cx, cy, _ = closest_point_on_segment(px, py, ax, ay, bx, by)
            dx = px - cx
            dy = py - cy
            dist = length(dx, dy)
            if best is None or dist < best[0]:
                best = (dist, dx, dy, ax, ay, bx, by)
        if best is None:
            continue
        dist, dx, dy, ax, ay, bx, by = best
        if dist < 1e-6 or dist >= influence_radius:
            continue
        strength = gain * (influence_radius - dist) / influence_radius
        nx = dx / dist
        ny = dy / dist
        seg_dx = bx - ax
        seg_dy = by - ay
        seg_norm = length(seg_dx, seg_dy)
        if seg_norm < 1e-6:
            tangent_x, tangent_y = -ny, nx
        else:
            tangent_x, tangent_y = seg_dx / seg_norm, seg_dy / seg_norm
        tangent_sign = 1.0 if (tangent_x * vx + tangent_y * vy) >= 0.0 else -1.0
        force_x += nx * strength + tangent_x * strength * 0.35 * tangent_sign
        force_y += ny * strength + tangent_y * strength * 0.35 * tangent_sign
    return force_x, force_y


def agent_repulsion(agent_index: int, states: list[dict], social_radius: float, gain: float) -> tuple[float, float]:
    agent = states[agent_index]
    force_x = 0.0
    force_y = 0.0
    for other_index, other in enumerate(states):
        if other_index == agent_index:
            continue
        dx = agent['x'] - other['x']
        dy = agent['y'] - other['y']
        dist = length(dx, dy)
        active_radius = max(social_radius, agent['agent'].preferred_distance, other['agent'].preferred_distance)
        if dist < 1e-6 or dist >= active_radius:
            continue
        strength = gain * agent['agent'].social_force_scale * (active_radius - dist) / active_radius
        force_x += dx / dist * strength
        force_y += dy / dist * strength
    return force_x, force_y


def random_force(state: dict, sim_time: float, gain: float, fade_time: float) -> tuple[float, float]:
    if gain <= 0.0:
        return 0.0, 0.0
    if fade_time <= 1e-6:
        fade_time = 1.0
    progress = (sim_time - state['delay_start']) % fade_time
    if progress < state['dt'] + 1e-9:
        state['last_random'] = state['next_random']
        angle = state['rng'].uniform(-math.pi, math.pi)
        distance = state['rng'].gauss(0.0, 1.0)
        state['next_random'] = (math.cos(angle) * distance, math.sin(angle) * distance)
    alpha = max(0.0, min(1.0, progress / fade_time))
    rx = (1.0 - alpha) * state['last_random'][0] + alpha * state['next_random'][0]
    ry = (1.0 - alpha) * state['last_random'][1] + alpha * state['next_random'][1]
    return rx * gain, ry * gain


def along_wall_force(position: tuple[float, float], velocity: tuple[float, float], desired_velocity: tuple[float, float],
                     obstacles, gain: float, speed_threshold: float,
                     distance_threshold: float) -> tuple[float, float]:
    if gain <= 0.0:
        return 0.0, 0.0
    if length(velocity[0], velocity[1]) > speed_threshold:
        return 0.0, 0.0
    if length(desired_velocity[0], desired_velocity[1]) < 1e-6:
        return 0.0, 0.0
    best = None
    px, py = position
    dvx, dvy = desired_velocity
    for obstacle in obstacles:
        for (ax, ay), (bx, by) in obstacle.boundary_segments():
            cx, cy, _ = closest_point_on_segment(px, py, ax, ay, bx, by)
            dx = px - cx
            dy = py - cy
            dist = length(dx, dy)
            if dist >= distance_threshold or dist < 1e-6:
                continue
            nx = dx / dist
            ny = dy / dist
            if nx * dvx + ny * dvy > -0.15:
                continue
            if best is None or dist < best[0]:
                best = (dist, ax, ay, bx, by)
    if best is None:
        return 0.0, 0.0
    _, ax, ay, bx, by = best
    seg_dx = bx - ax
    seg_dy = by - ay
    seg_norm = length(seg_dx, seg_dy)
    if seg_norm < 1e-6:
        return 0.0, 0.0
    tx = seg_dx / seg_norm
    ty = seg_dy / seg_norm
    tangent_sign = 1.0 if (tx * dvx + ty * dvy) >= 0.0 else -1.0
    return tx * gain * tangent_sign, ty * gain * tangent_sign


def resolve_obstacle_penetration(
    position: tuple[float, float],
    velocity: tuple[float, float],
    obstacles,
    actor_radius: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    px, py = position
    vx, vy = velocity
    for _ in range(3):
        moved = False
        for obstacle in obstacles:
            min_dist = actor_radius + 0.05
            if obstacle.shape == 'box':
                hx = obstacle.width * 0.5 + min_dist
                hy = obstacle.depth * 0.5 + min_dist
                local_x = px - obstacle.x
                local_y = py - obstacle.y
                if abs(local_x) >= hx or abs(local_y) >= hy:
                    continue
                dx_face = hx - abs(local_x)
                dy_face = hy - abs(local_y)
                if dx_face < dy_face:
                    nx = 1.0 if local_x >= 0.0 else -1.0
                    ny = 0.0
                    tx, ty = 0.0, 1.0
                    px = obstacle.x + nx * hx
                else:
                    nx = 0.0
                    ny = 1.0 if local_y >= 0.0 else -1.0
                    tx, ty = 1.0, 0.0
                    py = obstacle.y + ny * hy
            elif obstacle.shape == 'cylinder':
                dx = px - obstacle.x
                dy = py - obstacle.y
                dist = length(dx, dy)
                limit = obstacle.radius + min_dist
                if dist >= limit:
                    continue
                if dist < 1e-6:
                    nx, ny = 1.0, 0.0
                    tx, ty = 0.0, 1.0
                else:
                    nx = dx / dist
                    ny = dy / dist
                    tx, ty = -ny, nx
                px = obstacle.x + nx * limit
                py = obstacle.y + ny * limit
            else:
                best = None
                for (ax, ay), (bx, by) in obstacle.boundary_segments():
                    cx, cy, _ = closest_point_on_segment(px, py, ax, ay, bx, by)
                    dx = px - cx
                    dy = py - cy
                    dist = length(dx, dy)
                    if best is None or dist < best[0]:
                        best = (dist, dx, dy, ax, ay, bx, by)
                if best is None:
                    continue
                dist, dx, dy, ax, ay, bx, by = best
                if dist >= min_dist:
                    continue
                if dist < 1e-6:
                    seg_dx = bx - ax
                    seg_dy = by - ay
                    seg_norm = length(seg_dx, seg_dy)
                    if seg_norm < 1e-6:
                        nx, ny = 1.0, 0.0
                        tx, ty = 0.0, 1.0
                    else:
                        tx = seg_dx / seg_norm
                        ty = seg_dy / seg_norm
                        nx, ny = -ty, tx
                else:
                    nx = dx / dist
                    ny = dy / dist
                    seg_dx = bx - ax
                    seg_dy = by - ay
                    seg_norm = length(seg_dx, seg_dy)
                    if seg_norm < 1e-6:
                        tx, ty = -ny, nx
                    else:
                        tx = seg_dx / seg_norm
                        ty = seg_dy / seg_norm
                correction = min_dist - dist
                px += nx * correction
                py += ny * correction
            tangent_speed = vx * tx + vy * ty
            outward_speed = vx * nx + vy * ny
            if outward_speed < 0.0:
                vx = tx * tangent_speed
                vy = ty * tangent_speed
            moved = True
        if not moved:
            break
    return (px, py), (vx, vy)


def integrate_with_collision_substeps(
    position: tuple[float, float],
    velocity: tuple[float, float],
    dt: float,
    obstacles,
    actor_radius: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    px, py = position
    vx, vy = velocity
    speed = length(vx, vy)
    step_distance_limit = max(0.04, actor_radius * 0.2)
    substeps = max(1, int(math.ceil((speed * dt) / step_distance_limit)))
    sub_dt = dt / substeps
    for _ in range(substeps):
        next_position = (px + vx * sub_dt, py + vy * sub_dt)
        next_velocity = (vx, vy)
        next_position, next_velocity = resolve_obstacle_penetration(
            next_position,
            next_velocity,
            obstacles,
            actor_radius,
        )
        px, py = next_position
        vx, vy = next_velocity
    return (px, py), (vx, vy)


def close_loop_trajectory(
    trajectory: list[tuple[float, float, float, float, float]],
    speed: float,
    dt: float,
    actor_z: float,
) -> list[tuple[float, float, float, float, float]]:
    if len(trajectory) < 2:
        return trajectory
    start_t, start_x, start_y, _, start_yaw = trajectory[0]
    end_t, end_x, end_y, _, end_yaw = trajectory[-1]
    dx = start_x - end_x
    dy = start_y - end_y
    distance = length(dx, dy)
    yaw = math.atan2(dy, dx) if distance > 1e-6 else end_yaw
    yaw = unwrap_angle(yaw, end_yaw)
    closed = list(trajectory)
    if abs(yaw - end_yaw) > 1e-3:
        end_t += dt
        closed.append((end_t, end_x, end_y, actor_z, yaw))
    if distance <= 1e-6:
        end_t += dt
        closed.append((end_t, start_x, start_y, actor_z, unwrap_angle(start_yaw, yaw)))
        return closed
    travel_time = max(distance / max(speed, 1e-3), dt)
    step_count = max(1, int(math.ceil(travel_time / dt)))
    for step_index in range(1, step_count + 1):
        alpha = step_index / step_count
        x = end_x + dx * alpha
        y = end_y + dy * alpha
        current_yaw = yaw if step_index < step_count else unwrap_angle(start_yaw, yaw)
        closed.append((end_t + dt * step_index, x, y, actor_z, current_yaw))
    return closed


@dataclass(frozen=True)
class AgentSnapshot:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float


class PedestrianSimulator:
    def __init__(self, scene: Scene) -> None:
        self.scene = scene
        self.world = scene.world
        self.sim_time = 0.0
        self.states = []
        for agent in scene.agents:
            planner = create_planner(agent=agent, scene=scene)
            start_wp = planner.start_waypoint()
            current_wp = planner.current_waypoint()
            yaw = math.atan2(current_wp.y - start_wp.y, current_wp.x - start_wp.x)
            self.states.append({
                'name': agent.name,
                'agent': agent,
                'planner': planner,
                'speed': agent.speed_mps,
                'delay_start': agent.delay_start,
                'x': start_wp.x,
                'y': start_wp.y,
                'vx': 0.0,
                'vy': 0.0,
                'yaw': yaw,
                'dt': self.world.sim_dt,
                'rng': random.Random(sum(ord(ch) for ch in agent.name)),
                'last_random': (0.0, 0.0),
                'next_random': (0.0, 0.0),
                'detour_target': None,
                'detour_waypoint_id': None,
                'detour_steps': 0,
                'detour_best_goal_distance': None,
                'detour_rejected_targets': [],
            })

    def snapshots(self) -> list[AgentSnapshot]:
        return [
            AgentSnapshot(
                name=state['name'],
                x=state['x'],
                y=state['y'],
                z=self.world.actor_z,
                yaw=state['yaw'],
                vx=state['vx'],
                vy=state['vy'],
            )
            for state in self.states
        ]

    def step(self) -> list[AgentSnapshot]:
        self.sim_time += self.world.sim_dt
        actor_radius = self.world.actor_radius
        force_cache = []
        for index, state in enumerate(self.states):
            if self.sim_time < state['delay_start']:
                force_cache.append((0.0, 0.0))
                continue
            if hasattr(state['planner'], 'update_state'):
                state['planner'].update_state(
                    state['x'],
                    state['y'],
                    self.sim_time,
                    self.world.sim_dt,
                )
            waypoint = state['planner'].current_waypoint()
            if state['planner'].has_completed_destination(state['x'], state['y'], self.sim_time):
                state['planner'].advance(state['x'], state['y'], self.sim_time)
                state['detour_target'] = None
                state['detour_waypoint_id'] = None
                state['detour_steps'] = 0
                state['detour_best_goal_distance'] = None
                state['detour_rejected_targets'] = []
                waypoint = state['planner'].current_waypoint()
            main_goal = (waypoint.x, waypoint.y)
            main_goal_distance = length(main_goal[0] - state['x'], main_goal[1] - state['y'])

            detour_target = state['detour_target']
            if state['detour_waypoint_id'] != waypoint.waypoint_id:
                detour_target = None
                state['detour_target'] = None
                state['detour_waypoint_id'] = waypoint.waypoint_id
                state['detour_steps'] = 0
                state['detour_best_goal_distance'] = None
                state['detour_rejected_targets'] = []

            if detour_target is not None:
                detour_dx = detour_target[0] - state['x']
                detour_dy = detour_target[1] - state['y']
                if length(detour_dx, detour_dy) <= max(0.28, actor_radius + 0.06):
                    detour_target = None
                    state['detour_target'] = None
                    state['detour_steps'] = 0
                    state['detour_best_goal_distance'] = None
                else:
                    state['detour_steps'] += 1
                    best_distance = state['detour_best_goal_distance']
                    if best_distance is None or main_goal_distance < best_distance - 0.08:
                        state['detour_best_goal_distance'] = main_goal_distance
                        best_distance = main_goal_distance
                    max_detour_steps = max(10, int(1.8 / max(self.world.sim_dt, 1e-3)))
                    stalled_steps = max(8, int(0.8 / max(self.world.sim_dt, 1e-3)))
                    if (
                        state['detour_steps'] > max_detour_steps
                        or (
                            state['detour_steps'] > stalled_steps
                            and best_distance is not None
                            and main_goal_distance >= best_distance - 0.04
                        )
                    ):
                        rejected = state['detour_rejected_targets']
                        rejected.append((detour_target[0], detour_target[1]))
                        state['detour_rejected_targets'] = rejected[-4:]
                        detour_target = None
                        state['detour_target'] = None
                        state['detour_steps'] = 0
                        state['detour_best_goal_distance'] = None

            if detour_target is None:
                candidate_goal = choose_navigation_target(
                    (state['x'], state['y']),
                    main_goal,
                    self.scene.obstacles,
                    actor_radius,
                    tuple(state['detour_rejected_targets']),
                )
                if length(candidate_goal[0] - main_goal[0], candidate_goal[1] - main_goal[1]) > 1e-6:
                    detour_target = candidate_goal
                    state['detour_target'] = candidate_goal
                    state['detour_waypoint_id'] = waypoint.waypoint_id
                    state['detour_steps'] = 0
                    state['detour_best_goal_distance'] = main_goal_distance

            gx, gy = detour_target if detour_target is not None else main_goal
            dx = gx - state['x']
            dy = gy - state['y']
            dist = length(dx, dy)
            desired_vx = 0.0 if dist < 1e-6 else dx / dist * state['speed']
            desired_vy = 0.0 if dist < 1e-6 else dy / dist * state['speed']
            force_x = (desired_vx - state['vx']) * self.world.desired_force_gain * state['agent'].desired_force_scale
            force_y = (desired_vy - state['vy']) * self.world.desired_force_gain * state['agent'].desired_force_scale
            obs_fx, obs_fy = obstacle_force(
                (state['x'], state['y']),
                (state['vx'], state['vy']),
                self.scene.obstacles,
                self.world.obstacle_influence_radius + actor_radius,
                self.world.obstacle_force_gain * state['agent'].obstacle_force_scale,
            )
            social_fx, social_fy = agent_repulsion(
                index,
                self.states,
                self.world.social_force_radius + actor_radius,
                self.world.social_force_gain,
            )
            wall_fx, wall_fy = along_wall_force(
                (state['x'], state['y']),
                (state['vx'], state['vy']),
                (desired_vx, desired_vy),
                self.scene.obstacles,
                self.world.along_wall_force_gain,
                self.world.along_wall_speed_threshold,
                self.world.along_wall_distance_threshold,
            )
            rand_fx, rand_fy = random_force(
                state,
                self.sim_time,
                self.world.random_force_gain,
                self.world.random_force_time,
            )
            force_cache.append((
                force_x + obs_fx + social_fx + wall_fx + rand_fx,
                force_y + obs_fy + social_fy + wall_fy + rand_fy,
            ))

        for index, state in enumerate(self.states):
            if self.sim_time < state['delay_start']:
                continue
            fx, fy = force_cache[index]
            dt = self.world.sim_dt
            state['vx'] = state['vx'] * self.world.velocity_damping + fx * dt
            state['vy'] = state['vy'] * self.world.velocity_damping + fy * dt
            state['vx'], state['vy'] = clamp_norm(state['vx'], state['vy'], state['speed'])
            next_position, next_velocity = integrate_with_collision_substeps(
                (state['x'], state['y']),
                (state['vx'], state['vy']),
                dt,
                self.scene.obstacles,
                actor_radius,
            )
            state['x'], state['y'] = next_position
            state['vx'], state['vy'] = next_velocity
            speed = length(state['vx'], state['vy'])
            if speed > 1e-3:
                state['yaw'] = math.atan2(state['vy'], state['vx'])
        return self.snapshots()


def step_agents(config: dict) -> dict[str, list[tuple[float, float, float, float, float]]]:
    scene = load_scene(config)
    return step_scene(scene)


def step_scene(scene: Scene) -> dict[str, list[tuple[float, float, float, float, float]]]:
    simulator = PedestrianSimulator(scene)
    world = scene.world
    actor_z = world.actor_z
    dt = world.sim_dt
    trajectories: dict[str, list[tuple[float, float, float, float, float]]] = {}
    initial_snapshots = simulator.snapshots()
    for snapshot in initial_snapshots:
        trajectories[snapshot.name] = [(0.0, snapshot.x, snapshot.y, snapshot.z, snapshot.yaw)]

    steps = int(world.trajectory_duration / dt)
    for _ in range(steps):
        snapshots = simulator.step()
        for snapshot in snapshots:
            trajectories[snapshot.name].append((
                simulator.sim_time,
                snapshot.x,
                snapshot.y,
                snapshot.z,
                snapshot.yaw,
            ))
    for state in simulator.states:
        if state['planner'].agent.loop:
            trajectories[state['name']] = close_loop_trajectory(
                trajectories[state['name']],
                state['speed'],
                dt,
                actor_z,
            )
    return trajectories
