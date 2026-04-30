from dataclasses import dataclass


@dataclass(frozen=True)
class BehaviorProfile:
    area_radius: float
    hop_distance: float
    hop_sigma: float
    local_jitter_scale: float
    transit_jitter_scale: float
    helper_offset_sigma: float
    local_stop_min: int
    local_stop_max: int
    transit_dwell_min: float
    transit_dwell_max: float
    local_dwell_min: float
    local_dwell_max: float
    recency_window: int
    heading_bias: float
    attraction_probability: float
    attraction_max_distance: float
    shopping_min_duration: float
    shopping_release_probability: float
    shopping_cooldown_min: float
    shopping_cooldown_max: float


def profile_for_behavior(behavior: str) -> BehaviorProfile:
    if behavior == 'patrol':
        return BehaviorProfile(
            area_radius=2.60,
            hop_distance=6.0,
            hop_sigma=2.8,
            local_jitter_scale=0.75,
            transit_jitter_scale=0.65,
            helper_offset_sigma=1.10,
            local_stop_min=1,
            local_stop_max=2,
            transit_dwell_min=0.05,
            transit_dwell_max=0.20,
            local_dwell_min=0.10,
            local_dwell_max=0.35,
            recency_window=6,
            heading_bias=0.55,
            attraction_probability=0.003,
            attraction_max_distance=4.8,
            shopping_min_duration=3.0,
            shopping_release_probability=0.08,
            shopping_cooldown_min=10.0,
            shopping_cooldown_max=16.0,
        )
    if behavior == 'walk':
        return BehaviorProfile(
            area_radius=2.90,
            hop_distance=6.5,
            hop_sigma=3.0,
            local_jitter_scale=0.90,
            transit_jitter_scale=0.75,
            helper_offset_sigma=1.35,
            local_stop_min=1,
            local_stop_max=3,
            transit_dwell_min=0.05,
            transit_dwell_max=0.25,
            local_dwell_min=0.12,
            local_dwell_max=0.50,
            recency_window=6,
            heading_bias=0.42,
            attraction_probability=0.010,
            attraction_max_distance=6.2,
            shopping_min_duration=4.0,
            shopping_release_probability=0.06,
            shopping_cooldown_min=8.0,
            shopping_cooldown_max=14.0,
        )
    if behavior == 'browse':
        return BehaviorProfile(
            area_radius=3.40,
            hop_distance=6.0,
            hop_sigma=3.2,
            local_jitter_scale=1.30,
            transit_jitter_scale=0.95,
            helper_offset_sigma=1.65,
            local_stop_min=2,
            local_stop_max=4,
            transit_dwell_min=0.15,
            transit_dwell_max=0.40,
            local_dwell_min=0.45,
            local_dwell_max=1.40,
            recency_window=7,
            heading_bias=0.25,
            attraction_probability=0.030,
            attraction_max_distance=7.8,
            shopping_min_duration=6.5,
            shopping_release_probability=0.03,
            shopping_cooldown_min=4.0,
            shopping_cooldown_max=8.0,
        )
    if behavior == 'wander':
        return BehaviorProfile(
            area_radius=3.80,
            hop_distance=7.0,
            hop_sigma=3.5,
            local_jitter_scale=1.20,
            transit_jitter_scale=1.05,
            helper_offset_sigma=1.90,
            local_stop_min=2,
            local_stop_max=4,
            transit_dwell_min=0.10,
            transit_dwell_max=0.30,
            local_dwell_min=0.25,
            local_dwell_max=0.90,
            recency_window=8,
            heading_bias=0.18,
            attraction_probability=0.022,
            attraction_max_distance=8.6,
            shopping_min_duration=6.0,
            shopping_release_probability=0.028,
            shopping_cooldown_min=4.0,
            shopping_cooldown_max=7.0,
        )
    return BehaviorProfile(
        area_radius=2.8,
        hop_distance=6.5,
        hop_sigma=3.2,
        local_jitter_scale=1.0,
        transit_jitter_scale=0.8,
        helper_offset_sigma=1.4,
        local_stop_min=1,
        local_stop_max=3,
        transit_dwell_min=0.05,
        transit_dwell_max=0.25,
        local_dwell_min=0.15,
        local_dwell_max=0.60,
        recency_window=6,
        heading_bias=0.35,
        attraction_probability=0.010,
        attraction_max_distance=6.0,
        shopping_min_duration=4.0,
        shopping_release_probability=0.06,
        shopping_cooldown_min=7.0,
        shopping_cooldown_max=12.0,
    )
