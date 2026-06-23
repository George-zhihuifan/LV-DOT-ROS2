import argparse
import math
from pathlib import Path

import yaml

from depth_eval_bringup.pedestrian_sim_core import step_agents


def obstacle_block(obstacle: dict) -> str:
    r, g, b = obstacle['color']
    z = obstacle['length'] / 2.0
    shape = obstacle.get('shape', 'cylinder')
    if shape == 'segment':
        x1 = obstacle['x1']
        y1 = obstacle['y1']
        x2 = obstacle['x2']
        y2 = obstacle['y2']
        center_x = (x1 + x2) * 0.5
        center_y = (y1 + y2) * 0.5
        width = max(0.08, obstacle.get('width', 0.12))
        depth = max(0.08, ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5)
        yaw = math.atan2(y2 - y1, x2 - x1)
        geometry = f"""
            <box>
              <size>{depth:.3f} {width:.3f} {obstacle['length']:.3f}</size>
            </box>"""
        pose = f"{center_x:.3f} {center_y:.3f} {z:.3f} 0 0 {yaw:.5f}"
    elif shape == 'box':
        width = obstacle['width']
        depth = obstacle['depth']
        geometry = f"""
            <box>
              <size>{width:.3f} {depth:.3f} {obstacle['length']:.3f}</size>
            </box>"""
        pose = f"{obstacle['x']:.3f} {obstacle['y']:.3f} {z:.3f} 0 0 0"
    else:
        geometry = f"""
            <cylinder>
              <radius>{obstacle['radius']:.3f}</radius>
              <length>{obstacle['length']:.3f}</length>
            </cylinder>"""
        pose = f"{obstacle['x']:.3f} {obstacle['y']:.3f} {z:.3f} 0 0 0"
    return f"""
    <model name="{obstacle['name']}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
{geometry}
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
{geometry}
          </geometry>
          <material>
            <ambient>{r * 0.82:.3f} {g * 0.82:.3f} {b * 0.82:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def actor_block(agent: dict, actor_model_uri: str, actor_scale: float, actor_z: float,
                trajectories: dict[str, list[tuple[float, float, float, float, float]]]) -> str:
    waypoints = trajectories[agent['name']]
    start_x, start_y = waypoints[0][1], waypoints[0][2]
    start_yaw = waypoints[0][4]
    waypoint_lines = []
    for time_value, x, y, z, yaw in waypoints:
        waypoint_lines.append(
            f'          <waypoint><time>{time_value:.2f}</time><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.5f}</pose></waypoint>'
        )
    waypoints_text = '\n'.join(waypoint_lines)
    return f"""
    <actor name="{agent['name']}">
      <pose>{start_x:.3f} {start_y:.3f} {actor_z:.3f} 0 0 {start_yaw:.5f}</pose>
      <skin>
        <filename>{actor_model_uri}</filename>
        <scale>{actor_scale:.3f}</scale>
      </skin>
      <animation name="stand">
        <filename>{actor_model_uri}</filename>
        <scale>{actor_scale:.3f}</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>{str(agent.get('loop', True)).lower()}</loop>
        <delay_start>{agent.get('delay_start', 0.0):.2f}</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="stand">
{waypoints_text}
        </trajectory>
      </script>
    </actor>"""


def render_world(config: dict) -> str:
    world = config['world']
    obstacles = config['obstacles']
    agents = config['agents']
    trajectories = step_agents(config)
    obstacle_blocks = '\n'.join(obstacle_block(obstacle) for obstacle in obstacles)
    actor_blocks = '\n'.join(
        actor_block(
            agent,
            world['actor_model_uri'],
            world.get('actor_scale', 1.0),
            world['actor_z'],
            trajectories,
        )
        for agent in agents
    )
    ground_x, ground_y = world['ground_size']
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{world['name']}">
    <gravity>0 0 -9.8</gravity>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1.0</diffuse>
      <specular>0.2 0.2 0.2 1.0</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_x:.1f} {ground_y:.1f}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_x:.1f} {ground_y:.1f}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.28 0.3 0.32 1</ambient>
            <diffuse>0.32 0.34 0.36 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{obstacle_blocks}
{actor_blocks}
  </world>
</sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate a pedsim-style Gazebo world from scenario config.')
    parser.add_argument('--config', required=True, help='Path to the scenario YAML file.')
    parser.add_argument('--output', required=True, help='Path to the generated SDF world file.')
    args = parser.parse_args()

    config_path = Path(args.config)
    output_path = Path(args.output)
    with config_path.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    output_path.write_text(render_world(config), encoding='utf-8')


if __name__ == '__main__':
    main()
