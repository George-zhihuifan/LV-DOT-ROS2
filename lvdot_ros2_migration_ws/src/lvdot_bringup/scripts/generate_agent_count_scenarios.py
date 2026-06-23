#!/usr/bin/env python3
import argparse
from copy import deepcopy
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True, help="Base scenario yaml")
    parser.add_argument("--out-dir", required=True, help="Output directory")
    parser.add_argument("--min-agents", type=int, default=1)
    parser.add_argument("--max-agents", type=int, default=6)
    args = parser.parse_args()

    base_path = Path(args.base)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    with base_path.open("r", encoding="utf-8") as f:
        base_cfg = yaml.safe_load(f)

    base_groups = base_cfg.get("agent_groups", [])
    flat_agents = []
    for group in base_groups:
        count = int(group.get("count", 0))
        for idx in range(count):
            agent_group = deepcopy(group)
            agent_group["count"] = 1
            agent_group["start_index"] = int(group.get("start_index", 1)) + idx
            if "waypoint_sets" in agent_group and isinstance(agent_group["waypoint_sets"], list):
                sets = agent_group["waypoint_sets"]
                if sets:
                    agent_group["waypoint_sets"] = [sets[idx % len(sets)]]
            flat_agents.append(agent_group)

    max_agents = min(args.max_agents, len(flat_agents))
    for n_agents in range(args.min_agents, max_agents + 1):
        cfg = deepcopy(base_cfg)
        cfg["agent_groups"] = flat_agents[:n_agents]
        out_path = out_dir / f"pedestrian_dense_{n_agents:02d}agents.yaml"
        with out_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=False)
        print(out_path)


if __name__ == "__main__":
    main()
