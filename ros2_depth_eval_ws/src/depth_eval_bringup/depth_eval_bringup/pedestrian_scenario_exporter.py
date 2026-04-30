import argparse
from pathlib import Path
from xml.sax.saxutils import escape

import yaml

from depth_eval_bringup.pedestrian_scene import load_scene


def obstacle_lines(obstacle: dict) -> list[tuple[float, float, float, float]]:
    shape = obstacle.get('shape', 'cylinder')
    if shape == 'segment':
        return [
            (obstacle['x1'], obstacle['y1'], obstacle['x2'], obstacle['y2']),
        ]
    if shape == 'box':
        x = obstacle['x']
        y = obstacle['y']
        hx = obstacle['width'] * 0.5
        hy = obstacle['depth'] * 0.5
        return [
            (x - hx, y - hy, x + hx, y - hy),
            (x + hx, y - hy, x + hx, y + hy),
            (x + hx, y + hy, x - hx, y + hy),
            (x - hx, y + hy, x - hx, y - hy),
        ]
    x = obstacle['x']
    y = obstacle['y']
    r = obstacle['radius']
    return [
        (x - r, y, x + r, y),
        (x, y - r, x, y + r),
    ]


def render_scenario(config: dict) -> str:
    scene = load_scene(config)
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', '<scenario>']
    for obstacle in config.get('obstacles', []):
        for x1, y1, x2, y2 in obstacle_lines(obstacle):
            lines.append(f'  <obstacle x1="{x1:.3f}" y1="{y1:.3f}" x2="{x2:.3f}" y2="{y2:.3f}"/>')
    for waypoint in scene.waypoints.values():
        waypoint_id = escape(waypoint.waypoint_id)
        lines.append(
            f'  <waypoint id="{waypoint_id}" x="{waypoint.x:.3f}" y="{waypoint.y:.3f}" r="{waypoint.radius:.3f}" b="0"/>'
        )
    for agent in scene.agents:
        waypoint_ids = agent.waypoint_ids
        if not waypoint_ids:
            continue
        first = scene.waypoints.get(waypoint_ids[0])
        if first is None:
            continue
        lines.append(
            f'  <agent x="{first.x:.3f}" y="{first.y:.3f}" n="1" dx="0" dy="0" type="{agent.agent_type}">'
        )
        for waypoint_id in waypoint_ids:
            lines.append(f'    <addwaypoint id="{escape(waypoint_id)}"/>')
        lines.append('  </agent>')
    lines.append('</scenario>')
    return '\n'.join(lines) + '\n'


def main() -> None:
    parser = argparse.ArgumentParser(description='Export a pedsim-style scenario XML from the YAML scene config.')
    parser.add_argument('--config', required=True, help='Path to the scene YAML config.')
    parser.add_argument('--output', required=True, help='Path to the output XML scenario.')
    args = parser.parse_args()

    config_path = Path(args.config)
    output_path = Path(args.output)
    with config_path.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    output_path.write_text(render_scenario(config), encoding='utf-8')


if __name__ == '__main__':
    main()
