from depth_eval_bringup.pedestrian_agent_state_machine import PedestrianAgentStateMachine
from depth_eval_bringup.pedestrian_scene import AgentConfig, Scene


def create_planner(agent: AgentConfig, scene: Scene) -> PedestrianAgentStateMachine:
    return PedestrianAgentStateMachine(agent=agent, scene=scene)
