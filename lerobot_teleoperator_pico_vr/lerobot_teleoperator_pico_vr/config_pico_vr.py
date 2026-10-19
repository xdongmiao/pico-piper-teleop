from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("pico_vr")
@dataclass(kw_only=True)
class PicoVrConfig(TeleoperatorConfig):
    bridge_host: str = "127.0.0.1"
    bridge_port: int = 8765
    bridge_timeout_s: float = 0.2
    require_bridge: bool = True
    max_cached_action_age_ms: float = 500.0
